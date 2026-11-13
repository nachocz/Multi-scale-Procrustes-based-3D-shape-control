// Checks of the simulation modules of the demos (docs/js/sim), with Node.js:
//     node tests/test_sim.mjs            (about 40 s)
// Procrustes and SE(3) against known transforms, fast marching against Euclidean
// distances on a flat sheet, ARAP against rigid motions, the closed loop of every
// scenario (the shape error must decrease, and the Lyapunov terms of Sec. III must be
// non-positive), and the planar piece of the 2D page.
import assert from 'node:assert/strict';
import { rng, rotationFromVector } from '../docs/js/sim/linalg.js';
import { procrustes } from '../docs/js/sim/procrustes.js';
import { aboutPoint, exp, inverse, log, power, applyAll, compose } from '../docs/js/sim/se3.js';
import { fastMarching, vertexTriangles, isoline } from '../docs/js/sim/geodesic.js';
import { makeStrip, perceived } from '../docs/js/sim/mesh.js';
import { MultiScaleProcrustes } from '../docs/js/sim/controller.js';
import { Session } from '../docs/js/sim/session.js';
import { PLANAR_SCENARIOS, SCENARIOS, World } from '../docs/js/sim/world.js';
import { lyapunov, lyapunovTerms } from '../docs/js/sim/stability.js';

let failures = 0;
function check(name, fn) {
    const t0 = performance.now();
    try { fn(); console.log(`ok    ${name} (${(performance.now() - t0).toFixed(0)} ms)`); }
    catch (e) { failures++; console.log(`FAIL  ${name}\n      ${e.stack.split('\n').slice(0, 3).join('\n      ')}`); }
}
const maxAbs = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));

check('Procrustes recovers a rigid transform (also for coplanar points)', () => {
    const r = rng(1);
    for (const planar of [false, true]) {
        const n = 40, X = new Float64Array(3 * n);
        for (let i = 0; i < 3 * n; i++) X[i] = (i % 3 === 2 && planar) ? 0 : 20 * r() - 10;
        const T = { R: rotationFromVector([0.7, -1.9, 0.4]), t: [3, -2, 5] };
        const Y = applyAll(T, X), P = procrustes(X, Y);
        assert.ok(maxAbs(P.R, T.R) < 1e-9, 'rotation');
        assert.ok(maxAbs(P.t, T.t) < 1e-8, 'translation');
        assert.ok(P.d < 1e-7, `residual ${P.d}`);
    }
});

check('SE(3): exp(log T) = T, and the geodesic path composes', () => {
    for (const w of [[0.2, -0.1, 0.3], [2.9, 0.5, -0.4], [1e-9, 0, 0], [0, 3.1415, 0]]) {
        const T = { R: rotationFromVector(w), t: [1, -4, 2] }, E = exp(log(T));
        assert.ok(maxAbs(E.R, T.R) < 1e-9 && maxAbs(E.t, T.t) < 1e-8, `angle ${Math.hypot(...w)}`);
        const H = compose(power(T, 0.3), power(T, 0.7));
        assert.ok(maxAbs(H.R, T.R) < 1e-9 && maxAbs(H.t, T.t) < 1e-8, 'H(0.3) H(0.7) = T');
    }
});

check('fast marching: geodesic distances on a flat sheet match Euclidean distances', () => {
    const s = makeStrip({ nu: 41, nv: 21, nz: 2, h: 1, center: (u) => [-10 + 20 * u, 0], wMinus: () => 5, wPlus: () => 5 });
    const X = perceived(s, s.rest), adj = vertexTriangles(s.topTris, s.M), src = 20 * 21 + 10;
    const D = fastMarching(X, s.topTris, adj, [src]);
    let worst = 0, at = 0, mean = 0;
    for (let m = 0; m < s.M; m++) {
        const e = Math.hypot(X[3 * m] - X[3 * src], X[3 * m + 1] - X[3 * src + 1]);
        const err = Math.abs(D[m] - e) / Math.max(e, 1);
        mean += err / s.M;
        if (err > worst) { worst = err; at = e; }
    }
    assert.ok(mean < 0.02, `mean relative error ${mean}`);   // first-order scheme
    assert.ok(worst < 0.05, `relative error ${worst} at distance ${at}`);
    const segs = isoline(X, s.topTris, D, 4);
    let rad = 0;
    for (let k = 0; k < segs.length; k += 3) rad = Math.max(rad, Math.abs(Math.hypot(segs[k] - X[3 * src], segs[k + 1] - X[3 * src + 1]) - 4));
    assert.ok(segs.length > 0 && rad < 0.15, `isoline radius error ${rad}`);
});

check('ARAP: rigid gripper motions give a rigid motion of the object', () => {
    const w = new World('bend');
    const T = { R: rotationFromVector([0.3, 0.2, -0.5]), t: [2, 1, -3] }, step = power(T, 1 / 20);
    w.reset();
    for (let s = 0; s < 20; s++) w.move(w.grippers.map(() => step));
    const X = w.X(), expected = applyAll(T, w.restX);
    assert.ok(maxAbs(X, expected) < 1e-2, `max deviation ${maxAbs(X, expected)}`);
});

check('ARAP: the accelerated iteration decreases the energy monotonically', () => {
    const w = new World('twist');
    w.reset();
    w.poses = w.poses.map((p, k) => compose({ R: rotationFromVector([k ? -0.6 : 0.6, 0, 0]), t: [0, 0, 0] }, p));
    w.placeHandles(w.P, w.poses);
    let previous = Infinity;
    for (let it = 0; it < 30; it++) {
        w.arap.solve(w.P, 4);
        w.arap.localStep(w.P);
        const E = w.arap.energy(w.P);
        assert.ok(E <= previous * (1 + 1e-9), `energy increased at ${it}: ${previous} -> ${E}`);
        previous = E;
    }
});

const runs = {};
for (const id of Object.keys(SCENARIOS)) {
    check(`closed loop "${id}": the shape error decreases`, () => {
        const t0 = performance.now();
        const w = new World(id), tPrep = performance.now() - t0;
        const ctrl = new MultiScaleProcrustes({ tris: w.surface.tris, adjacency: vertexTriangles(w.surface.tris, w.M) },
            w.grippers.map((g) => g.source), w.r0);
        ctrl.start(w.X(), w.Y);
        const e = [], r = [];
        let tCtrl = 0, tSim = 0, worstLyap = -Infinity;
        for (let k = 0; k < 250; k++) {
            const X = w.X(), a = performance.now();
            const out = ctrl.step(X);
            const b = performance.now();
            const terms = lyapunovTerms(X, ctrl.Y, out.balls, out.ballCounts, out.Tstar);
            for (const t of terms) worstLyap = Math.max(worstLyap, t.rotation, t.translation);
            w.move(out.U);
            tCtrl += b - a; tSim += performance.now() - b;
            e.push(out.e); r.push(out.rStar);
        }
        runs[id] = { e, r };
        const final = e[e.length - 1];
        console.log(`      e: ${e[0].toFixed(2)} -> ${e[49].toFixed(2)} (k=50) -> ${final.toFixed(3)} cm (k=250); r*: ${r[0].toFixed(1)} -> ${r[49].toFixed(1)} -> ${r[r.length - 1].toFixed(1)} cm of [${ctrl.scales[0].toFixed(1)}, ${ctrl.R.toFixed(1)}]`);
        console.log(`      prepare ${tPrep.toFixed(0)} ms, control ${(tCtrl / 250).toFixed(1)} ms/it, ARAP ${(tSim / 250).toFixed(1)} ms/it, M = ${w.M}, nodes = ${w.strip.n}, max Lyapunov term ${worstLyap.toExponential(1)}`);
        assert.ok(final < 0.15 * e[0], 'error reduced below 15 % of the initial error');
        assert.ok(worstLyap <= 1e-9, 'Lyapunov terms of Lemmas 2 and 3 are non-positive');
    });
}

check('measurement noise moves r* to larger balls once the error approaches the noise', () => {
    // the transforms of balls with few points become noisy; larger balls average the noise
    for (const id of ['twist', 'appendix']) {
        const s = new Session(new World(id), { noise: 1 });
        const r = [], e = [];
        for (let k = 0; k < 160; k++) { s.step(); r.push(s.out.rStar); e.push(s.trueError()); }
        const early = r.slice(0, 10).reduce((a, b) => a + b) / 10, late = r.slice(120).reduce((a, b) => a + b) / 40;
        console.log(`      ${id}: mean r* ${early.toFixed(1)} cm (k < 10), ${late.toFixed(1)} cm (k > 120); noise-free error ${e[159].toFixed(3)} cm`);
        assert.ok(late > early + 3, 'larger scales late in the run');
        assert.ok(e[159] < 0.3, 'the noise-free error converges');
    }
});

check('unreachable target: V stays far from zero, with non-positive terms (Remark of Sec. III)', () => {
    // a reachable target is driven to V = 0 (closed-loop checks above); with a bump that no
    // gripper motion produces, V drops to a few percent of its initial value and stays
    // there without settling, while the terms of the lemmas (LRB motion) remain negative
    const w = new World('bend'), bumped = w.bumpedTarget(2);
    const s = new Session(w, { noise: 0, target: bumped.Y });
    const V = [];
    let worst = -Infinity;
    for (let k = 0; k < 400; k++) {
        const terms = lyapunovTerms(s.X, s.ctrl.Y, s.out.balls, s.out.ballCounts, s.out.Tstar);
        for (const t of terms) worst = Math.max(worst, t.rotation, t.translation);
        V.push(lyapunov(w.X(), s.ctrl.Y));
        s.step();
    }
    const late = V.slice(100), lo = Math.min(...late), hi = Math.max(...late);
    console.log(`      V: ${V[0].toFixed(1)} -> [${lo.toFixed(2)}, ${hi.toFixed(2)}] cm^2 for k >= 100; largest term ${worst.toExponential(1)}`);
    assert.ok(lo > 0.01 * V[0] && hi < 0.06 * V[0], 'V stays between 1 % and 6 % of its initial value');
    assert.ok(worst <= 1e-9, 'terms of the lemmas non-positive');
});

// ---------------------------------------------------------------- planar piece (2D page)
const IDENTITY = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] };

check('planar Procrustes recovers a rotation about z (SE(2)), also for nearly mirror-symmetric sets', () => {
    const r = rng(5), n = 30, X = new Float64Array(3 * n);
    for (let i = 0; i < n; i++) { X[3 * i] = 10 * r() - 5; X[3 * i + 1] = 0.2 * (r() - 0.5); }      // nearly collinear
    const T = { R: rotationFromVector([0, 0, 2.5]), t: [1, -2, 0] }, P = procrustes(X, applyAll(T, X), null, true);
    assert.ok(Math.abs(P.R[0] - T.R[0]) < 1e-9 && Math.abs(P.R[3] - T.R[3]) < 1e-9 && P.R[8] === 1, 'rotation about z');
});

check('2D page: gripper increments T_target T^-1 over long runs keep the poses orthonormal', () => {
    const w = new World('ushape'), g1 = w.grippers[0];
    const pose = (t) => compose(aboutPoint(rotationFromVector([0, 0, 0.5 * Math.sin(t / 2)]), g1.restPose.t, [2.6 * Math.sin(t / 2 + 0.8), 2 * Math.sin(t), 0]), g1.restPose);
    for (let k = 0; k < 600; k++) w.move([compose(pose(0.1 * (k + 1)), inverse(w.poses[0])), IDENTITY]);
    const R = w.poses[0].R;
    assert.ok(Array.from(w.P).every(Number.isFinite), 'finite positions');
    assert.ok(Math.abs(R[0] * R[0] + R[3] * R[3] - 1) < 1e-12, 'orthonormal pose');
});

check('2D page: a stiff arm or a hinge moves the best-fitting scale of the LRB model outwards', () => {
    const best = (id) => {
        const w = new World(id);
        const X0 = w.X(), D = fastMarching(X0, w.surface.tris, vertexTriangles(w.surface.tris, w.M), [w.grippers[0].source]);
        const U = aboutPoint(rotationFromVector([0, 0, 15 * Math.PI / 180]), w.poses[0].t);
        for (let s = 0; s < 12; s++) w.move([power(U, 1 / 12), IDENTITY]);
        w.arap.solve(w.P, 60);
        const X = w.X(), Ux = applyAll(U, X0);
        let arg = 0, min = Infinity;
        for (let r = 0.5; r < Math.max(...D); r += 0.25) {
            let s2 = 0;
            for (let m = 0; m < w.M; m++) {
                const p = D[m] <= r ? Ux : X0;
                s2 += (X[3 * m] - p[3 * m]) ** 2 + (X[3 * m + 1] - p[3 * m + 1]) ** 2;
            }
            if (s2 < min) { min = s2; arg = r; }
        }
        return arg;
    };
    const uniform = best('ushape'), stiff = best('ustiff'), bar = best('bar'), hinge = best('hinge');
    console.log(`      turn of 15 deg: best r ${uniform.toFixed(1)} cm (U), ${stiff.toFixed(1)} cm (U with a stiff arm); ${bar.toFixed(1)} cm (bar), ${hinge.toFixed(1)} cm (hinge at 12 cm)`);
    assert.ok(stiff > uniform + 6, 'larger scale with the stiff arm');
    // the soft zone of the hinge begins about 10.3 cm from the centre of g1
    assert.ok(hinge > bar + 2.5 && hinge > 9, 'the link held by the gripper moves rigidly up to the hinge');
});

check('2D page: the planar closed loop converges and stays in the plane, on every piece', () => {
    for (const id of Object.keys(PLANAR_SCENARIOS)) {
        const w = new World(id);
        const s = new Session(w, { noise: 0.3 }), e0 = s.trueError();
        for (let k = 0; k < 200; k++) s.step();
        const z = Math.max(...Array.from(w.X()).filter((_, i) => i % 3 === 2).map(Math.abs));
        console.log(`      ${id}: e ${e0.toFixed(1)} -> ${s.trueError().toFixed(3)} cm, |z| <= ${z.toExponential(0)}`);
        assert.ok(s.trueError() < 0.01 * e0, 'error below 1 % of the initial error');
        assert.ok(z < 1e-9, 'planar');
    }
});

console.log(failures ? `\n${failures} check(s) failed` : '\nall checks passed');
process.exit(failures ? 1 : 0);

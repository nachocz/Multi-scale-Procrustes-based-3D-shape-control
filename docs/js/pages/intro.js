// The idea in 2D: planar pieces (ARAP, one layer of nodes in the plane z = 0) held by two
// grippers, with rigid motions in SE(2). Four steps: the geodesic ball of radius r, the
// LRB model at r against the simulated object for a motion of a gripper, the estimation
// of r from the observed responses to small motions, and the closed loop of the
// multi-scale Procrustes controller towards a target.
import { ErrorDeformationPlot, ScaleBars, ScaleTimeMap, cssColor, signedColor } from '../lib/charts.js';
import { Plane2D } from '../lib/plane2d.js';
import { LinePlot } from '../lib/plot.js';
import { screwPath } from '../lib/present.js';
import { initPage } from '../lib/shell.js';
import { alpha, mix, onThemeChange, palette } from '../lib/theme.js';
import { button, card, control, fmt, h, label, legend, segmented, slider } from '../lib/ui.js';
import { fastMarching, isoline, vertexTriangles } from '../sim/geodesic.js';
import { rotationFromVector } from '../sim/linalg.js';
import { aboutPoint, applyAll, compose, inverse, power } from '../sim/se3.js';
import { Session } from '../sim/session.js';
import { PLANAR_SCENARIOS, World } from '../sim/world.js';

await initPage();

const DEG = Math.PI / 180, IDENTITY = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] }, NS = 24;
const rotZ = (a) => rotationFromVector([0, 0, a]);
const PHASE = 12;                 // seconds of each motion in the sequence of step 3

// ---------------------------------------------------------------- the object (one per scenario)
const objects = {};
let O = null;                     // the current object: world, mesh, distances at rest, scales
function object(id) {
    if (objects[id]) return objects[id];
    const world = new World(id), { strip } = world, M = world.M, tris = world.surface.tris;
    const adjacency = vertexTriangles(tris, M), { nu, nv } = strip;
    const distances = (X, g) => fastMarching(X, tris, adjacency, [world.grippers[g].source]);
    const restD = [distances(world.restX, 0), distances(world.restX, 1)];
    const R1 = Math.max(...restD[0]);
    const scales = Float64Array.from({ length: NS }, (_, s) => world.r0 + (R1 - world.r0) * s / (NS - 1));
    const boundary = [];
    for (let i = 0; i < nu; i++) boundary.push(i * nv);
    for (let j = 1; j < nv; j++) boundary.push((nu - 1) * nv + j);
    for (let i = nu - 2; i >= 0; i--) boundary.push(i * nv + nv - 1);
    for (let j = nv - 2; j >= 1; j--) boundary.push(j);
    const arrows = [];
    for (let i = 2; i < nu; i += 3) for (const j of [0, nv - 1]) arrows.push(i * nv + j);
    // triangles of a stiff region (for drawing), from the stiffness of the material
    const stiffTri = new Uint8Array(tris.length / 3);
    if (world.stiffness) for (let k = 0; k < tris.length; k += 3) stiffTri[k / 3] = (world.stiffness[tris[k]] + world.stiffness[tris[k + 1]] + world.stiffness[tris[k + 2]]) / 3 > 5 ? 1 : 0;
    objects[id] = { id, world, strip, M, tris, nu, nv, distances, restD, R1, scales, boundary, arrows, stiffTri, sc: PLANAR_SCENARIOS[id] };
    return objects[id];
}
const scaleAt = (s) => { const { scales } = O, i = Math.max(0, Math.min(NS - 2, Math.floor(s))), f = Math.min(1, s - i); return (1 - f) * scales[i] + f * scales[i + 1]; };
const g1 = () => O.world.grippers[0];

const state = { step: 1, object: 'ushape', gripper: 0, r1: 9, r2: 4, speed: 1, motion: 'sequence', rhat: NS - 6, obs: null, hist: null, running: false, t: 0 };
let session = null, h4 = null, last = 0, budget = 0, frame = 0;

// ---------------------------------------------------------------- layout
const workspace = document.getElementById('workspace');
const objectPick = segmented({
    options: Object.entries(PLANAR_SCENARIOS).map(([value, s]) => ({ value, label: s.title })), value: state.object,
    onChange: (v) => { state.object = v; setObject(); },
});
const objectNote = h('span', { class: 'hint' });
const steps = [['1', 'The scale r cuts the object'], ['2', 'Object and model at r'], ['3', 'Estimate r from observations'], ['4', 'Close the loop']];
const tabs = h('div', { class: 'steps', role: 'tablist', style: 'margin:10px 0 0' }, steps.map(([n, title], k) => {
    const b = h('button', { class: 'step-tab', type: 'button', role: 'tab', 'aria-selected': String(k === 0) }, h('b', {}, `Step ${n}`), h('span', {}, title));
    b.addEventListener('click', () => setStep(k + 1));
    return b;
}));
const topCard = card('', '');
topCard.body.append(h('div', { class: 'toolbar' }, h('b', { style: 'font-size:0.8rem' }, 'Object'), objectPick.el, objectNote), tabs);
const statement = h('p', { class: 'statement' });
const viewCard = card('', '');
const canvas = h('canvas', { class: 'plane2d dash-fit' });
const legendEl = h('div');
viewCard.body.append(statement, canvas, legendEl);
const view = new Plane2D(canvas);

// charts
const chartCard = card('', '');
const chartBody = h('div'), chartLegend = h('div');
chartCard.body.append(chartBody, chartLegend);
const mismatchCanvas = h('canvas', { class: 'plot-canvas tall' });
const mismatchPlot = new LinePlot(mismatchCanvas, { xLabel: 'r [cm]', yIncludeZero: true });
const deltaCanvas = h('canvas', { class: 'chart-canvas' });
const deltaBars = new ScaleBars(deltaCanvas, { yLabel: 'normalized', xLabel: "r' [cm]", yRange: [0, 1] });
const rhatCanvas = h('canvas', { class: 'plot-canvas tall' });
const rhatPlot = new LinePlot(rhatCanvas, { xLabel: 't [s]', yIncludeZero: true });
const errCanvas = h('canvas', { class: 'chart-canvas', style: 'height:200px' });
const errPlot = new ErrorDeformationPlot(errCanvas);
const mapCanvas = h('canvas', { class: 'chart-canvas', style: 'height:200px' });
const qMap = new ScaleTimeMap(mapCanvas);
const qCanvas = h('canvas', { class: 'chart-canvas' });
const qBars = new ScaleBars(qCanvas, { yLabel: 'q / max|q|', signed: true });

// controls
const ctrlCard = card('Controls', ''), ctrlBody = h('div');
ctrlCard.body.append(ctrlBody);
const readCard = card('Values', ''), readBody = h('div', { class: 'readouts' });
readCard.body.append(readBody);
const r1Slider = slider({ label: 'scale $r$ [cm]', min: 0.5, max: 30, step: 0.1, value: state.r1, format: (v) => v.toFixed(1), onInput: (v) => { state.r1 = v; render(); } });
const gripperPick = segmented({ options: [{ value: 0, label: 'g1' }, { value: 1, label: 'g2' }], value: 0, onChange: (v) => { state.gripper = v; render(); } });
const r2Slider = slider({ label: 'scale $r$ of the LRB model [cm]', min: 0.5, max: 30, step: 0.1, value: state.r2, format: (v) => v.toFixed(1), onInput: (v) => { state.r2 = v; render(); } });
const bestButton = button('Set $r$ to the best fit', () => { state.r2 = state.best2 ?? state.r2; r2Slider.set(state.r2); render(); }, { small: true });
// motions of g1 in its own frame: t along the piece, n across it
const local = (angle, along, across) => {
    const P = g1().restPose, d = [along * P.R[0] + across * P.R[1], along * P.R[3] + across * P.R[4], 0];
    return aboutPoint(rotZ(angle), P.t, d);
};
const presets = h('div', { class: 'toolbar' },
    button('Turn 15°', () => preset(local(15 * DEG, 0, 0)), { small: true }),
    button('Move across 3 cm', () => preset(local(0, 0, 3)), { small: true }),
    button('Pull 2 cm', () => preset(local(0, -2, 0)), { small: true }),
    button('↺ Reset', () => { O.world.reset(); render(); }, { small: true }));
const runButton = button('▶ Run', () => setRunning(!state.running), { primary: true });
const restartButton = button('↺ Restart', () => { setRunning(false); enterStep(); });
const speedPick = segmented({ options: [1, 3, 8].map((v) => ({ value: v, label: `${v}×` })), value: 1, onChange: (v) => { state.speed = v; } });
const clock = h('span', { class: 'clock' }, 't = 0.0 s');
const runBar = h('div', { class: 'toolbar' }, runButton, restartButton, speedPick.el, clock);
const motionPick = segmented({
    options: [{ value: 'sequence', label: 'rotation, then translation' }, { value: 'rotation', label: 'rotation' }, { value: 'translation', label: 'translation' }],
    value: state.motion, onChange: (v) => { state.motion = v; setRunning(false); enterStep(); },
});
const groups = {
    1: () => [control('gripper', gripperPick.el), r1Slider.el],
    2: () => [r2Slider.el, h('div', { class: 'toolbar' }, bestButton), h('p', { class: 'group-label' }, 'Motion of g1 (or drag it)'), presets],
    3: () => [runBar, control('motion of g1', motionPick.el), h('p', { class: 'hint' }, 'You can also pause and drag g1: every small motion is an observation.')],
    4: () => [runBar],
};
workspace.append(topCard.el, h('div', { class: 'dash' },
    h('div', { class: 'dash-view' }, viewCard.el),
    h('div', { class: 'dash-side' }, ctrlCard.el, chartCard.el, readCard.el)));

// ---------------------------------------------------------------- object and steps
function setObject() {
    setRunning(false);
    O = object(state.object);
    objectNote.textContent = O.sc.note;
    r1Slider.input.max = Math.ceil(Math.max(...O.restD[0], ...O.restD[1]));
    r2Slider.input.max = Math.ceil(O.R1);
    enterStep();
}

function setStep(n) {
    setRunning(false);
    state.step = n;
    [...tabs.children].forEach((b, k) => b.setAttribute('aria-selected', String(k + 1 === n)));
    for (const el of document.querySelectorAll('.step-text')) el.classList.toggle('active', Number(el.dataset.step) === n);
    document.querySelector(`.step-text[data-step="${n}"]`)?.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    ctrlBody.replaceChildren(...groups[n]());
    const title = (t) => h('p', { class: 'chart-title' }, label(t));
    const charts = {
        1: [],
        2: [title('Mismatch between the object and the LRB model for this motion of g1, RMS distance between $\\mathbf x_m$ and $\\hat{\\mathbf x}_m(r)$ [mm]'), mismatchCanvas],
        3: [title("Prediction error of the last motion for every $r'$ (bars) and $w(t,r')$ (line)"), deltaCanvas,
            h('div', { style: 'height:10px' }), title('Estimate $\\hat r(t)$ and the best scale of each motion [cm]'), rhatCanvas],
        4: [title('Shape error and deformation of the object, as in Fig. 2 of the paper'), errCanvas,
            h('div', { style: 'height:10px' }), title('$q(t,r)$ and $r^*(t)$ (red), as in Fig. 2'), mapCanvas,
            h('div', { style: 'height:10px' }), title('$q(t,r)$ at the current iteration'), qCanvas],
    }[n];
    chartCard.el.hidden = !charts.length;
    chartBody.replaceChildren(...charts);
    enterStep();
}

function enterStep() {
    const n = state.step, { world } = O;
    world.reset();
    state.t = 0;
    if (n === 3) { state.rhat = NS - 6; state.obs = null; state.hist = { t: [], rhat: [], best: [], phase: [] }; }
    if (n === 4) {
        session = new Session(world, { noise: 0.3 });
        h4 = { t: [], e: [], mean: [], std: [], q: [], r: [], X0: world.X() };
        record4();
        view.fit([world.restX, applyAll(session.ctrl.alignment, world.target.P)], 0.15);
    } else view.fit([world.restX], n === 1 ? 0.12 : 0.3);
    statement.replaceChildren(label({
        1: 'Move the slider $r$: the tinted part is the geodesic ball $\\Omega(r)$, the part that the model moves rigidly with the gripper.',
        2: 'Drag g1 (or its round handle to rotate it), or use the buttons. Then change $r$ and compare the object with the model.',
        3: 'Press Run: g1 first rotates back and forth, then translates back and forth. After every small motion, $\\hat r$ moves down the gradient of the prediction error.',
        4: 'Press Run: both grippers act at the scale $r^*$ selected by $q(t,r)$ until the piece matches the target.',
    }[n]));
    render();
}

// ---------------------------------------------------------------- gripper motions (steps 2 and 3)
// Motions are applied in substeps (at most 0.3 cm or 3 degrees), as the simulator expects.
function moveG1(U) {
    const l = Math.hypot(U.t[0], U.t[1]), a = Math.abs(Math.atan2(U.R[3], U.R[0]));
    const n = Math.max(1, Math.ceil(Math.max(l / 0.3, a / (3 * DEG)))), sub = power(U, 1 / n);
    for (let k = 0; k < n; k++) {
        if (state.step === 3) observe(sub);
        else O.world.move([sub, IDENTITY]);
    }
}
function preset(T) {
    const { world } = O;
    world.reset();
    moveG1(compose(compose(T, g1().restPose), inverse(world.poses[0])));
    world.arap.solve(world.P, 40);
    render();
}

// RMS distance between matched points of two planar shapes. With g2 fixed, the frame of
// the comparison is fixed; eq. (8) uses the Procrustes distance d_P, which also discounts
// a rigid motion of the whole piece.
function rms(A, B) {
    let s = 0;
    for (let m = 0; m < O.M; m++) s += (A[3 * m] - B[3 * m]) ** 2 + (A[3 * m + 1] - B[3 * m + 1]) ** 2;
    return Math.sqrt(s / O.M);
}

// Step 3: one observation. The LRB prediction of the motion U for every scale r', the
// observed response of the object, and two gradient steps of r-hat.
function observe(U) {
    const { world, M, scales } = O;
    const Xp = world.X(), D = O.distances(Xp, 0), order = Int32Array.from({ length: M }, (_, m) => m).sort((a, b) => D[a] - D[b]);
    world.move([U, IDENTITY]);
    const X = world.X(), Ux = applyAll(U, Xp), pred = Xp.slice(), delta = new Float64Array(NS);
    let k = 0;
    for (let s = 0; s < NS; s++) {
        while (k < M && D[order[k]] <= scales[s]) { const m = order[k++]; pred[3 * m] = Ux[3 * m]; pred[3 * m + 1] = Ux[3 * m + 1]; }
        delta[s] = rms(X, pred);
    }
    const lo = Math.min(...delta), hi = Math.max(...delta), span = hi - lo || 1;
    const norm = Float64Array.from(delta, (d) => (d - lo) / span);
    const grad = (i) => (i <= 0 ? norm[1] - norm[0] : i >= NS - 1 ? norm[NS - 1] - norm[NS - 2] : (norm[i + 1] - norm[i - 1]) / 2);
    for (let it = 0; it < 2; it++) {
        const i0 = Math.max(0, Math.min(NS - 1, Math.floor(state.rhat))), f = state.rhat - i0;
        state.rhat = Math.max(0, Math.min(NS - 1, state.rhat - ((1 - f) * grad(i0) + f * grad(Math.min(NS - 1, i0 + 1)))));
    }
    let best = 0;
    for (let s = 1; s < NS; s++) if (delta[s] < delta[best]) best = s;
    const predHat = Xp.slice(), rh = scaleAt(state.rhat);
    for (let m = 0; m < M; m++) if (D[m] <= rh) { predHat[3 * m] = Ux[3 * m]; predHat[3 * m + 1] = Ux[3 * m + 1]; }
    // w(t, r') of eq. (8), beta = 1e4, distance (sqrt(M) times the RMS) divided by M; relative to its largest value
    const w = Array.from(delta, (d) => Math.exp(-1e4 * (d - lo) / Math.sqrt(M)));
    state.obs = { Xp, X, predHat, norm, w, best };
    state.t += 0.1;
    const hst = state.hist;
    hst.t.push(state.t); hst.rhat.push(rh); hst.best.push(scales[best]); hst.phase.push(phaseAt(state.t));
}

// the scripted motion of g1 in step 3: rotation (±20°) and translation across the piece (±2.5 cm)
function phaseAt(t) {
    if (state.motion !== 'sequence') return state.motion;
    return Math.floor(t / PHASE) % 2 ? 'translation' : 'rotation';
}
function scriptedPose(t) {
    const om = 2 * Math.PI / 4, ph = phaseAt(t), tt = state.motion === 'sequence' ? t % PHASE : t;
    const T = ph === 'rotation' ? local(20 * DEG * Math.sin(om * tt), 0, 0) : local(0, 0, 2.5 * Math.sin(om * tt));
    return compose(T, g1().restPose);
}

// pointer: drag the body of g1 to move it, its handle to rotate it
const handleOf = (pose) => { const L = g1().half[0] + 2.2; return [pose.t[0] - pose.R[0] * L, pose.t[1] - pose.R[3] * L]; };
const hit = (p) => {
    const pose = O.world.poses[0], hd = handleOf(pose), d = [p[0] - pose.t[0], p[1] - pose.t[1]];
    const lx = pose.R[0] * d[0] + pose.R[3] * d[1], ly = pose.R[1] * d[0] + pose.R[4] * d[1];
    if (Math.hypot(p[0] - hd[0], p[1] - hd[1]) < 1.1) return 'rotate';
    if (Math.abs(lx) < g1().half[0] + 0.6 && Math.abs(ly) < g1().half[1] + 0.6) return 'move';
    return null;
};
let drag = null;
const at = (e) => { const r = canvas.getBoundingClientRect(); return view.world(e.clientX - r.left, e.clientY - r.top); };
canvas.addEventListener('pointerdown', (e) => {
    if (state.step !== 2 && state.step !== 3) return;
    const p = at(e), mode = hit(p);
    if (!mode) return;
    if (state.running) setRunning(false);
    canvas.setPointerCapture(e.pointerId);
    const pose = O.world.poses[0];
    drag = { mode, start: p, pose0: { R: pose.R.slice(), t: pose.t.slice() } };
});
canvas.addEventListener('pointermove', (e) => {
    const p = at(e);
    if (!drag) { const m = (state.step === 2 || state.step === 3) && hit(p); canvas.style.cursor = m === 'rotate' ? 'grab' : m === 'move' ? 'move' : 'default'; return; }
    const { pose0, start } = drag;
    let target;
    if (drag.mode === 'move') target = { R: pose0.R, t: [pose0.t[0] + p[0] - start[0], pose0.t[1] + p[1] - start[1], 0] };
    else {
        const a = Math.atan2(p[1] - pose0.t[1], p[0] - pose0.t[0]) - Math.atan2(start[1] - pose0.t[1], start[0] - pose0.t[0]);
        target = { R: compose({ R: rotZ(a), t: [0, 0, 0] }, { R: pose0.R, t: [0, 0, 0] }).R, t: pose0.t };
    }
    moveG1(compose(target, inverse(O.world.poses[0])));
    render();
});
const endDrag = () => { if (!drag) return; drag = null; if (state.step === 2) O.world.arap.solve(O.world.P, 30); render(); };
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);

// ---------------------------------------------------------------- running (steps 3 and 4)
function setRunning(flag) {
    state.running = flag && (state.step === 3 || state.step === 4);
    runButton.replaceChildren(label(state.running ? '❚❚ Pause' : '▶ Run'));
    if (state.running) { last = performance.now(); budget = 0; requestAnimationFrame(loop); }
}
function loop(now) {
    if (!state.running) return;
    budget += Math.min((now - last) / 1000, 0.1) * 10 * state.speed;
    last = now;
    const t0 = performance.now();
    while (budget >= 1 && performance.now() - t0 < 25) {
        budget -= 1;
        if (state.step === 3) observe(compose(scriptedPose(state.t + 0.1), inverse(O.world.poses[0])));
        else if (state.step === 4) {
            if (h4.t.length >= 400) { setRunning(false); break; }
            session.step(); record4();
        }
    }
    budget = Math.min(budget, 2);
    render(++frame % 3 === 0);
    if (state.running) requestAnimationFrame(loop);
}
function record4() {
    const s = O.world.stretch(O.world.X(), h4.X0);
    h4.t.push(session.t); h4.e.push(session.out.e); h4.mean.push(s.mean); h4.std.push(s.std);
    h4.q.push(session.out.qn); h4.r.push(session.out.rStar);
}

// ---------------------------------------------------------------- drawing
function drawObject(X, tint = null) {
    const p = palette(), { tris, stiffTri } = O;
    view.mesh(X, tris, (k) => {
        const c = tint ? tint(k) : p.object;
        return stiffTri[k] ? mix(c, p.dark ? '#ffffff' : '#000000', 0.16) : c;
    });
    view.outline(X, O.boundary, { color: p.inkSoft, width: 1.3 });
}
const triMean = (D, k) => (D[O.tris[3 * k]] + D[O.tris[3 * k + 1]] + D[O.tris[3 * k + 2]]) / 3;
function ballTint(D, r, soft) { const p = palette(); return (k) => (triMean(D, k) <= r ? soft : p.object); }
function drawGrippers(poses, { anchored = [false, true], handle = false } = {}) {
    const p = palette();
    poses.forEach((pose, k) => {
        const g = O.world.grippers[k], color = p[`g${k + 1}`];
        view.gripper(pose, g.half, { color, fill: alpha(p.gripper, 0.28), anchored: anchored[k] });
        view.label(`g${k + 1}`, pose.t, { color, size: 12.5, align: 'center' });
    });
    if (handle) {
        const pose = poses[0], hd = handleOf(pose);
        view.polyline([pose.t, hd], { color: p.g1, width: 1.6 });
        view.dot(hd, 7, p.g1); view.dot(hd, 3, p.canvas);
    }
}
// tags of the material: the stiff arm or the soft hinge
function materialTag(X) {
    const { nu, nv, id } = O;
    if (id === 'ustiff') { const m = Math.round(0.14 * (nu - 1)) * nv; view.tag('stiff arm\n(30× stiffer)', [X[3 * m], X[3 * m + 1]], { dx: -40, dy: 0 }); }
    if (id === 'hinge') { const m = Math.round(0.5 * (nu - 1)) * nv + nv - 1; view.tag('soft hinge', [X[3 * m], X[3 * m + 1]], { dx: 24, dy: -46 }); }
}
// a point of the isoline far from the centre c, for its tag
function farthest(seg, c) {
    let best = null, bd = -1;
    for (let s = 0; s + 2 < seg.length; s += 3) { const d = Math.hypot(seg[s] - c[0], seg[s + 1] - c[1]); if (d > bd) { bd = d; best = [seg[s], seg[s + 1]]; } }
    return best;
}
// a point on an edge of the piece (j = 0 or nv - 1) where D is close to the value d
function edgePoint(X, D, d, j) {
    let best = 0, bd = Infinity;
    for (let i = 0; i < O.nu; i++) { const m = i * O.nv + j, e = Math.abs(D[m] - d); if (e < bd) { bd = e; best = m; } }
    return [X[3 * best], X[3 * best + 1]];
}
// which side of the piece faces outwards (left of the screen) near gripper k
const outerEdge = (X, k) => {
    const { nv, nu } = O, i = k ? nu - 2 : 1;
    return X[3 * (i * nv)] < X[3 * (i * nv + nv - 1)] ? 0 : nv - 1;
};

function render(full = true) {
    if (!O || !view.begin()) return;
    const p = palette();
    ({ 1: renderStep1, 2: renderStep2, 3: renderStep3, 4: renderStep4 })[state.step](p, full);
}

function renderStep1(p) {
    const { world, restD, M } = O, k = state.gripper, X = world.X(), D = restD[k], r = state.r1, soft = p[`g${k + 1}Soft`], color = p[`g${k + 1}`];
    drawObject(X, ballTint(D, r, soft));
    for (let v = 2; v < Math.max(...D); v += 2) view.segments(isoline(X, O.tris, D, v), { color: alpha(p.muted, 0.5), width: 0.8, halo: false });
    const iso = isoline(X, O.tris, D, r);
    view.segments(iso, { color, width: 3 });
    drawGrippers(world.poses, { anchored: [false, false] });
    materialTag(X);
    const c = world.grippers[k].restPose.t, f = farthest(iso, c);
    if (f) view.tag(`isoline D${k + 1} = r\n= ${r.toFixed(1)} cm`, f, { color, dx: 34, dy: -24 });
    const e = outerEdge(X, k);
    view.tag(`Ω${k + 1}(r): moves\nrigidly with g${k + 1}`, edgePoint(X, D, 0.55 * r, e), { color, dx: e ? 34 : -34, dy: 0 });
    if (Math.max(...D) > r + 4) view.tag('rest: does\nnot move', edgePoint(X, D, Math.min(Math.max(...D) - 1, r + 6), e), { color: p.inkSoft, dx: e ? 34 : -34, dy: 14 });
    legendEl.replaceChildren(legend([
        { label: `geodesic ball $\\Omega_${k + 1}(r)$`, color: soft, kind: 'band' }, { label: `isoline $D_${k + 1}=r$`, color },
        { label: 'isolines of $D$ every 2 cm', color: p.muted },
    ]));
    let inBall = 0;
    for (let m = 0; m < M; m++) if (D[m] <= r) inBall++;
    setValues([['$r$ [cm]', fmt(r, 1)], ['points in $\\Omega(r)$', `${inBall} of ${M}`], ['largest $D$ [cm]', fmt(Math.max(...D), 1)]]);
}

function renderStep2(p) {
    const { world, restD, M, tris, nu, nv } = O, X = world.X(), D = restD[0], r = state.r2;
    const U = compose(world.poses[0], inverse(g1().restPose)), Ux = applyAll(U, world.restX);
    const order = Int32Array.from({ length: M }, (_, m) => m).sort((a, b) => D[a] - D[b]);
    const rs = [], mis = [], pred = world.restX.slice();
    let k = 0, best = null;
    for (let s = 0; s <= 60; s++) {
        const rr = 0.3 + (O.R1 + 0.2 - 0.3) * s / 60;
        while (k < M && D[order[k]] <= rr) { const m = order[k++]; pred[3 * m] = Ux[3 * m]; pred[3 * m + 1] = Ux[3 * m + 1]; }
        const d = 10 * rms(X, pred);
        rs.push(rr); mis.push(d);
        if (!best || d < best.d) best = { r: rr, d };
    }
    state.best2 = best.r;
    const predR = world.restX.slice();
    for (let m = 0; m < M; m++) if (D[m] <= r) { predR[3 * m] = Ux[3 * m]; predR[3 * m + 1] = Ux[3 * m + 1]; }
    const dR = 10 * rms(X, predR);
    view.outline(world.restX, O.boundary, { color: alpha(p.muted, 0.7), width: 1, dash: [3, 4] });
    drawObject(X);
    view.mesh(predR, tris, (q) => (triMean(D, q) <= r ? alpha(p.estimate, 0.28) : null), { stroke: false });
    view.outline(predR, O.boundary, { color: p.estimate, width: 2 });
    view.outline(X, O.boundary, { color: p.ink, width: 1.6 });
    for (let i = 1; i < nu; i += 3) for (const j of [0, nv - 1]) {
        const m = i * nv + j;
        view.polyline([[predR[3 * m], predR[3 * m + 1]], [X[3 * m], X[3 * m + 1]]], { color: p.bad, width: 1.1 });
    }
    const iso = isoline(world.restX, tris, D, r);
    view.segments(applyAll(U, Float64Array.from(iso)), { color: p.g1, width: 2.4 });
    drawGrippers(world.poses, { handle: true });
    materialTag(X);
    const moved = Math.hypot(U.t[0], U.t[1]) > 0.05 || Math.abs(Math.atan2(U.R[3], U.R[0])) > 0.01;
    const e = outerEdge(world.restX, 0), mid = Math.floor(0.75 * nu) * nv + (nv - 1 - e);
    view.tag('object\n(ARAP)', [X[3 * mid], X[3 * mid + 1]], { color: p.ink, dx: e ? -34 : 34, dy: 18 });
    if (moved) {
        view.tag(`model: Ω1(r) moves\nrigidly, r = ${r.toFixed(1)} cm`, edgePoint(predR, D, 0.6 * r, e), { color: p.estimate, dx: e ? 36 : -36, dy: -8 });
        if (O.R1 > r + 4) view.tag('model: the rest\ndoes not move', edgePoint(predR, D, Math.min(O.R1 - 1, r + 6), e), { color: p.estimate, dx: e ? 36 : -36, dy: 12 });
    } else view.label('drag g1 or its handle', [g1().restPose.t[0], g1().restPose.t[1]], { color: p.muted, align: 'center', dy: -60, size: 12 });
    legendEl.replaceChildren(legend([
        { label: 'object (ARAP)', color: p.object, kind: 'band' }, { label: 'LRB model $\\hat{\\mathbf X}(r)$, its rigid part filled', color: p.estimate },
        { label: 'isoline $D_1=r$', color: p.g1 }, { label: 'mismatch', color: p.bad }, { label: 'rest shape', color: p.muted, kind: 'dashed' },
    ]));
    mismatchPlot.set({ series: [{ x: rs, y: mis, color: p.ink, width: 1.8 }], vlines: [{ x: r, color: p.rstar, dash: [] }, { x: best.r, color: p.good, dash: [4, 3] }],
        markers: [{ x: r, y: dR, color: p.rstar }, { x: best.r, y: best.d, color: p.good }] });
    chartLegend.replaceChildren(legend([{ label: 'selected $r$', color: p.rstar }, { label: 'best $r$ for this motion', color: p.good, kind: 'dashed' }]));
    setValues([['$r$ [cm]', fmt(r, 1)], ['mismatch at $r$ [mm]', fmt(dR, 2)], ['best $r$ [cm]', fmt(best.r, 1)], ['mismatch at best $r$ [mm]', fmt(best.d, 2)]]);
}

function renderStep3(p, full) {
    const { world, scales } = O, X = world.X(), D = O.distances(X, 0), rh = scaleAt(state.rhat);
    drawObject(X, ballTint(D, rh, p.g1Soft));
    const iso = isoline(X, O.tris, D, rh);
    view.segments(iso, { color: p.g1, width: 2.6 });
    const o = state.obs;
    if (o) {
        const k = 10;
        for (const m of O.arrows) {
            const a = [o.Xp[3 * m], o.Xp[3 * m + 1]];
            view.arrow(a, [a[0] + k * (o.X[3 * m] - a[0]), a[1] + k * (o.X[3 * m + 1] - a[1])], { color: p.ink, width: 1.2, head: 4 });
            view.arrow(a, [a[0] + k * (o.predHat[3 * m] - a[0]), a[1] + k * (o.predHat[3 * m + 1] - a[1])], { color: p.estimate, width: 1.2, head: 4 });
        }
    }
    drawGrippers(world.poses, { handle: true });
    materialTag(X);
    const f = farthest(iso, world.poses[0].t);
    if (f) view.tag(`r̂ = ${rh.toFixed(1)} cm`, f, { color: p.g1, dx: 32, dy: -22 });
    const ph = phaseAt(state.t);
    if (state.hist.t.length) view.caption(ph === 'rotation' ? 'now: g1 rotates back and forth (±20°)' : 'now: g1 translates back and forth across the piece (±2.5 cm)', { color: p.accentStrong });
    legendEl.replaceChildren(legend([
        { label: '$\\Omega_1(\\hat r)$ and the isoline $D_1=\\hat r$', color: p.g1 }, { label: 'last motion, observed (×10)', color: p.ink },
        { label: 'last motion, LRB model at $\\hat r$ (×10)', color: p.estimate },
    ]));
    clock.textContent = `t = ${state.t.toFixed(1)} s`;
    deltaBars.set(o ? { scales, values: Array.from(o.norm), color: alpha(p.accent, 0.75), marker: rh, markerLabel: 'r̂', curve: { values: o.w, color: p.good } }
        : { scales, values: Array(NS).fill(0), color: p.accent, marker: rh, markerLabel: 'r̂' });
    if (full || !state.running) {
        const hst = state.hist, spans = [];
        let start = null;
        hst.t.forEach((t, i) => {
            const rot = hst.phase[i] === 'rotation';
            if (rot && start === null) start = t - 0.1;
            if ((!rot || i === hst.t.length - 1) && start !== null) { spans.push({ x0: start, x1: t, color: alpha(p.g1, 0.1) }); start = null; }
        });
        rhatPlot.set({ series: [{ x: hst.t, y: hst.best, color: p.muted, width: 1.2, dash: [3, 3] }, { x: hst.t, y: hst.rhat, color: p.rstar, width: 2.2 }], spans });
        chartLegend.replaceChildren(legend([{ label: 'normalized prediction error $\\bar\\delta(r\')$', color: p.accent, kind: 'band' }, { label: "$w(t,r')$, relative", color: p.good },
            { label: '$\\hat r(t)$', color: p.rstar }, { label: 'best scale of each motion', color: p.muted, kind: 'dashed' }, { label: 'rotation phases', color: p.g1, kind: 'band' }]));
    }
    setValues([['$\\hat r$ [cm]', fmt(rh, 1)], ['best scale, last motion [cm]', o ? fmt(scales[o.best], 1) : '–'], ['motion', ph], ['observations', String(state.hist.t.length)]]);
}

function renderStep4(p, full) {
    const { world, M } = O, { out, ctrl } = session, X = world.X();
    const target = applyAll(ctrl.alignment, world.target.P);
    view.outline(target, O.boundary, { color: p.target, width: 2, dash: [6, 4], fill: alpha(p.target, 0.12) });
    const inBall = out.balls.map((b, g) => { const f = new Uint8Array(M); for (let k = 0; k < out.ballCounts[g]; k++) f[b.order[k]] = 1; return f; });
    drawObject(X, (k) => {
        const a = O.tris[3 * k], b = O.tris[3 * k + 1], c = O.tris[3 * k + 2];
        const in1 = inBall[0][a] + inBall[0][b] + inBall[0][c] >= 2, in2 = inBall[1][a] + inBall[1][b] + inBall[1][c] >= 2;
        return in1 && in2 ? mix(p.g1Soft, p.g2Soft, 0.5) : in1 ? p.g1Soft : in2 ? p.g2Soft : p.object;
    });
    if (out.sStar < out.nS - 1 - 1e-9) out.D.forEach((D, g) => view.segments(isoline(X, O.tris, D, out.rStar), { color: p[`g${g + 1}`], width: 2.4 }));
    world.poses.forEach((pose, g) => {
        const T = out.Tstar[g];
        view.gripper(compose(T, pose), world.grippers[g].half, { color: p[`g${g + 1}`], dash: [4, 3], width: 1.3 });
        view.polyline(screwPath(T, pose.t, 30), { color: p[`g${g + 1}`], width: 1.3, dash: [4, 3] });
    });
    drawGrippers(world.poses, { anchored: [false, false] });
    materialTag(X);
    const tm = Math.floor(0.62 * O.nu) * O.nv;
    view.tag('target Y', [target[3 * tm], target[3 * tm + 1]], { color: p.target, dx: 34, dy: 24 });
    legendEl.replaceChildren(legend([
        { label: '$\\Omega_1(t,r^*)$', color: p.g1Soft, kind: 'band' }, { label: '$\\Omega_2(t,r^*)$', color: p.g2Soft, kind: 'band' },
        { label: 'isolines $D_g=r^*$', color: p.g1 }, { label: 'target $\\mathbf Y$', color: p.target, kind: 'dashed' },
        { label: 'poses after $\\mathbf T_g(t,r^*)$ and the $\\mathrm{SE}(2)$ geodesics', color: p.muted, kind: 'dashed' },
    ]));
    clock.textContent = `t = ${session.t.toFixed(1)} s`;
    qBars.set({ scales: out.scales, values: Array.from(out.qn), colors: (v) => cssColor(signedColor(v)), marker: out.rStar, markerLabel: 'r*' });
    if (full || !state.running) {
        errPlot.set({ t: h4.t, e: h4.e, mean: h4.mean, std: h4.std });
        qMap.set({ times: h4.t, columns: h4.q, rStar: h4.r, scales: out.scales });
        chartLegend.replaceChildren(h('div', { class: 'colorbar signed' }, h('span', {}, 'q < 0: the error decreases'), h('i'), h('span', {}, 'q > 0')));
    }
    setValues([['$e(t)$ [cm]', fmt(h4.e[h4.e.length - 1], 2)], ['$r^*$ [cm]', fmt(out.rStar, 1)], ['iteration', String(h4.t.length - 1)]]);
}

function setValues(rows) {
    readBody.replaceChildren(...rows.map(([l, v]) => h('div', { class: 'readout' }, h('span', {}, label(l)), h('strong', {}, v))));
}

new ResizeObserver(() => render()).observe(canvas);
onThemeChange(() => render());
const params = new URLSearchParams(location.search);
if (PLANAR_SCENARIOS[params.get('object')]) { state.object = params.get('object'); objectPick.set(state.object); }
O = object(state.object);
objectNote.textContent = O.sc.note;
r1Slider.input.max = Math.ceil(Math.max(...O.restD[0], ...O.restD[1]));
r2Slider.input.max = Math.ceil(O.R1);
setStep(Number(params.get('step')) || 1);

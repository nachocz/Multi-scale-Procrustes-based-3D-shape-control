// The simulated scene: a cut-out object (js/sim/mesh.js), the grippers that hold it, and
// the ARAP deformation model (js/sim/arap.js), as in the simulations of the paper.
// Lengths are in cm. Initial and target shapes are produced by the same simulator: the
// grippers are moved from the rest shape to prescribed poses in small increments
// (quasi-static loading), and the target shape Y is reachable by construction.
//
// The surface map of the paper (functional maps, ZoomOut) is replaced here by the
// known correspondence of the simulator: point m of X(t) is matched to point m of Y.
import { Arap } from './arap.js';
import { mulVec3, rotationFromVector, transpose3 } from './linalg.js';
import { graspedNodes, makeStrip, meanEdge, perceived } from './mesh.js';
import { aboutPoint, apply, compose, inverse, orthonormalize, power } from './se3.js';

const DEG = Math.PI / 180;
const flatTop = (u, c, top, fall) => {
    const d = Math.abs(u - c);
    return d < top ? 1 : d < top + fall ? 0.5 * (1 + Math.cos(Math.PI * (d - top) / fall)) : 0;
};

// Gripper poses are given relative to the rest shape: rotation vector `rot` (rad) about
// the center of the gripper, then translation `move` (cm).
export const SCENARIOS = {
    bend: {
        title: 'Bending',
        note: 'a strip bent into an arch (Fig. 2, fourth column)',
        shape: { nu: 37, nv: 7, nz: 2, h: 1.0, center: (u) => [-12 + 24 * u, 0], wMinus: () => 2.5, wPlus: () => 2.5 },
        grippers: [{ i: 1, j: 3, di: 1, dj: 3 }, { i: 35, j: 3, di: 1, dj: 3 }],
        target: [{ rot: [0, -60 * DEG, 0], move: [1.96, 0, 0] }, { rot: [0, 60 * DEG, 0], move: [-1.96, 0, 0] }],
        view: { from: [10, -30, 18], at: [0, 0, 1.5] },
    },
    twist: {
        title: 'Twist',
        note: 'a plate twisted by opposite rotations of its ends (Fig. 2, third column)',
        shape: { nu: 31, nv: 11, nz: 2, h: 1.0, center: (u) => [-11 + 22 * u, 0], wMinus: () => 4, wPlus: () => 4 },
        grippers: [{ i: 1, j: 5, di: 1, dj: 5 }, { i: 29, j: 5, di: 1, dj: 5 }],
        target: [{ rot: [50 * DEG, 0, 0], move: [0.6, 0, 0] }, { rot: [-50 * DEG, 0, 0], move: [-0.6, 0, 0] }],
        view: { from: [16, -26, 18], at: [0, 0, 0] },
    },
    sbend: {
        title: 'Anti-symmetric bending',
        note: 'a bar bent into an S with a light twist (Fig. 2, first column)',
        shape: { nu: 37, nv: 5, nz: 3, h: 2.0, center: (u) => [-12 + 24 * u, 0], wMinus: () => 1.5, wPlus: () => 1.5 },
        grippers: [{ i: 1, j: 2, di: 1, dj: 2 }, { i: 35, j: 2, di: 1, dj: 2 }],
        target: [{ rot: [0, -45 * DEG, 0], move: [0, 0, 0] }, { rot: [20 * DEG, -45 * DEG, 0], move: [0, 0, 0] }],
        view: { from: [8, -32, 14], at: [0, 0, 0] },
    },
    appendix: {
        title: 'Beam with an appendix',
        note: 'a bent beam straightened by grippers at one end and at a small appendix (Fig. 2, second column)',
        shape: {
            nu: 41, nv: 9, nz: 2, h: 1.2, center: (u) => [-13 + 26 * u, 0],
            wMinus: () => 1.6, wPlus: (u) => 1.6 + 3.6 * flatTop(u, 0.7, 0.035, 0.04),
        },
        grippers: [{ i: 1, j: 4, di: 1, dj: 4 }, { i: 28, j: 8, di: 1, dj: 1 }],
        initial: [{ rot: [0, -55 * DEG, 0], move: [1.29, 0, 0] }, { rot: [0, 55 * DEG, 0], move: [-1.29, 0, 0] }],
        target: null,                                   // the straight rest shape
        view: { from: [8, -28, 22], at: [0, 1.5, 2] },
    },
    triangle: {
        title: 'Triangle',
        note: 'a triangular cut-out bent in 3D by its base corners (Fig. 3, experiment 4)',
        shape: {
            nu: 19, nv: 17, nz: 2, h: 1.0, center: (u) => [0, 7 - 14 * u],
            wMinus: (u) => 10 * (1 - u) + 1.2 * u, wPlus: (u) => 10 * (1 - u) + 1.2 * u,
        },
        grippers: [{ i: 1, j: 1, di: 1, dj: 1 }, { i: 1, j: 15, di: 1, dj: 1 }],
        target: [{ rot: [-15 * DEG, 40 * DEG, 0], move: [2.5, 0, 0] }, { rot: [-15 * DEG, -40 * DEG, 0], move: [-2.5, 0, 0] }],
        view: { from: [0, -26, 24], at: [0, 0, 0] },
    },
};

// Planar pieces (one layer of nodes in the plane z = 0, actions in SE(2)) for the 2D
// introduction, each held by a gripper at either end. `stiffness` (optional) gives the
// stiffness of the material along the piece, s(u, v); the controller does not know it.
const U_ARM = 10, U_RADIUS = 4, U_LENGTH = 2 * U_ARM + Math.PI * U_RADIUS;
const U_SHAPE = {
    nu: 61, nv: 5, nz: 1, h: 0,
    center: (u) => {
        const s = u * U_LENGTH;
        if (s <= U_ARM) return [-U_RADIUS, U_ARM - s];
        if (s <= U_ARM + Math.PI * U_RADIUS) { const a = Math.PI + (s - U_ARM) / U_RADIUS; return [U_RADIUS * Math.cos(a), U_RADIUS * Math.sin(a)]; }
        return [U_RADIUS, s - U_ARM - Math.PI * U_RADIUS];
    },
    wMinus: () => 1.2, wPlus: () => 1.2,
};
const HINGE = (u) => flatTop(u, 0.5, 0.02, 0.03);          // 1 at the hinge, 0 along the links
export const PLANAR_SCENARIOS = {
    ushape: {
        title: 'U-shaped piece',
        note: 'uniform material',
        planar: true,
        shape: U_SHAPE,
        grippers: [{ i: 1, j: 2, di: 1, dj: 2 }, { i: 59, j: 2, di: 1, dj: 2 }],
        target: [{ rot: [0, 0, -12 * DEG], move: [-5, -1, 0] }, { rot: [0, 0, 8 * DEG], move: [4, 1, 0] }],
    },
    ustiff: {
        title: 'U with a stiff arm',
        note: 'the arm held by g1 is 30 times stiffer',
        planar: true,
        shape: U_SHAPE,
        stiffness: (u) => (u < 0.28 ? 30 : 1),
        stiffU: 0.28,
        grippers: [{ i: 1, j: 2, di: 1, dj: 2 }, { i: 59, j: 2, di: 1, dj: 2 }],
        target: [{ rot: [0, 0, -12 * DEG], move: [-5, -1, 0] }, { rot: [0, 0, 8 * DEG], move: [4, 1, 0] }],
    },
    bar: {
        title: 'Straight bar',
        note: 'uniform material',
        planar: true,
        shape: { nu: 49, nv: 5, nz: 1, h: 0, center: (u) => [-12 + 24 * u, 0], wMinus: () => 1.2, wPlus: () => 1.2 },
        grippers: [{ i: 1, j: 2, di: 1, dj: 2 }, { i: 47, j: 2, di: 1, dj: 2 }],
        target: [{ rot: [0, 0, 20 * DEG], move: [3, 0, 0] }, { rot: [0, 0, -20 * DEG], move: [-3, 0, 0] }],      // an arch
    },
    hinge: {
        title: 'Hinge',
        note: 'two stiff links joined by a soft, narrow hinge',
        planar: true,
        shape: { nu: 49, nv: 5, nz: 1, h: 0, center: (u) => [-12 + 24 * u, 0], wMinus: (u) => 1.2 - 0.75 * HINGE(u), wPlus: (u) => 1.2 - 0.75 * HINGE(u) },
        stiffness: (u) => 1 + 49 * (1 - HINGE(u)),
        grippers: [{ i: 1, j: 2, di: 1, dj: 2 }, { i: 47, j: 2, di: 1, dj: 2 }],
        // the left link folded by 40 degrees about the hinge, at the origin
        target: [{ rot: [0, 0, 40 * DEG], move: [2.69, -7.39, 0] }, { rot: [0, 0, 0], move: [0, 0, 0] }],
    },
};

const relative = (spec, center) => aboutPoint(rotationFromVector(spec.rot || [0, 0, 0]), center, spec.move || [0, 0, 0]);

export class World {
    constructor(id, { arapIterations = 8 } = {}) {
        const sc = SCENARIOS[id] || PLANAR_SCENARIOS[id];
        this.id = id; this.scenario = sc; this.planar = !!sc.planar;
        this.arapIterations = arapIterations;
        const strip = makeStrip(sc.shape);
        this.strip = strip;
        // grippers: grasped nodes, rest pose (frame on the centerline axes), and box size
        this.grippers = sc.grippers.map((spec) => {
            const nodes = graspedNodes(strip, spec);
            const c = [0, 1, 2].map((e) => nodes.reduce((s, x) => s + strip.rest[3 * x + e], 0) / nodes.length);
            const { t, n } = strip.frame(spec.i / (strip.nu - 1));
            const R = [t[0], n[0], 0, t[1], n[1], 0, 0, 0, 1];               // columns t, n, z
            const pose = { R, t: c };
            const Rt = transpose3(R);
            const local = Array.from(nodes, (x) => mulVec3(Rt, [strip.rest[3 * x] - c[0], strip.rest[3 * x + 1] - c[1], strip.rest[3 * x + 2] - c[2]]));
            const half = [0, 1, 2].map((e) => Math.max(...local.map((p) => Math.abs(p[e]))) + (e === 2 ? 0.35 : 0.3));
            return { ...spec, nodes, local, restPose: pose, half, source: spec.i * strip.nv + spec.j };
        });
        const handles = new Set();
        for (const g of this.grippers) for (const x of g.nodes) handles.add(x);
        this.handles = handles;
        this.stiffness = sc.stiffness ? this.stiffnessOf(sc.stiffness) : null;
        this.arap = new Arap(strip.rest, strip.neighbors, handles, this.stiffness);
        this.blend = this.blendWeights();
        this.surface = { tris: strip.topTris };
        this.restX = perceived(strip, strip.rest);
        this.r0 = 2 * meanEdge(this.restX, strip.topTris);
        // initial configuration and target
        this.initialSpec = sc.initial || sc.grippers.map(() => ({ rot: [0, 0, 0], move: [0, 0, 0] }));
        this.targetSpec = sc.target || sc.grippers.map(() => ({ rot: [0, 0, 0], move: [0, 0, 0] }));
        this.initial = this.load(this.initialSpec);
        this.setTarget(this.targetSpec);
        this.reset();
    }

    get G() { return this.grippers.length; }
    get M() { return this.strip.M; }

    // Relative deformation of the perceived surface with respect to the shape ref, as in
    // Fig. 2 of the paper: the change of every edge length in percent (positive when the
    // object stretches, negative when it is compressed), as {mean, std}.
    stretch(X, ref) {
        if (!this.edges) {
            const set = new Set(), t = this.strip.topTris, list = [];
            for (let k = 0; k < t.length; k += 3) for (let e = 0; e < 3; e++) {
                const a = Math.min(t[k + e], t[k + (e + 1) % 3]), b = Math.max(t[k + e], t[k + (e + 1) % 3]);
                if (!set.has(a * 1e6 + b)) { set.add(a * 1e6 + b); list.push(a, b); }
            }
            this.edges = Int32Array.from(list);
        }
        const E = this.edges, len = (P, a, b) => Math.hypot(P[3 * a] - P[3 * b], P[3 * a + 1] - P[3 * b + 1], P[3 * a + 2] - P[3 * b + 2]);
        let s = 0, s2 = 0;
        const n = E.length / 2;
        for (let k = 0; k < E.length; k += 2) {
            const v = 100 * (len(X, E[k], E[k + 1]) / len(ref, E[k], E[k + 1]) - 1);
            s += v; s2 += v * v;
        }
        const mean = s / n;
        return { mean, std: Math.sqrt(Math.max(0, s2 / n - mean * mean)) };
    }

    // Stiffness per node from s(u, v) on the rest shape.
    stiffnessOf(fn) {
        const { strip } = this;
        return Float64Array.from({ length: strip.n }, (_, i) => fn(strip.uv[2 * strip.column[i]], strip.uv[2 * strip.column[i] + 1]));
    }

    // Stiffness of the material along the piece: s(u, v) (null: uniform). The initial
    // shape and the target are produced again with the new material.
    setStiffness(fn) {
        const { strip } = this;
        const s = fn ? this.stiffnessOf(fn) : null;
        this.stiffness = s;
        this.arap = new Arap(strip.rest, strip.neighbors, this.handles, s);
        this.initial = this.load(this.initialSpec);
        this.setTarget(this.targetSpec);
        this.reset();
    }

    // Place the grasped nodes of every gripper at its pose.
    placeHandles(P, poses) {
        this.grippers.forEach((g, k) => {
            g.nodes.forEach((x, a) => {
                const p = apply(poses[k], g.local[a]);
                P[3 * x] = p[0]; P[3 * x + 1] = p[1]; P[3 * x + 2] = p[2];
            });
        });
    }

    // Weights of the predictor: each node follows the grippers with weights that decay
    // with the distance along the lattice edges (rest shape) to the grasped nodes.
    blendWeights() {
        const { n, rest, neighbors } = this.strip, G = this.grippers.length, W = new Float64Array(n * G);
        this.grippers.forEach((g, k) => {
            const d = new Float64Array(n).fill(Infinity), done = new Uint8Array(n), queue = [];
            for (const x of g.nodes) { d[x] = 0; queue.push(x); }
            while (queue.length) {                         // Dijkstra (small graphs: linear scan)
                let best = 0;
                for (let q = 1; q < queue.length; q++) if (d[queue[q]] < d[queue[best]]) best = q;
                const v = queue[best]; queue[best] = queue[queue.length - 1]; queue.pop();
                if (done[v]) continue;
                done[v] = 1;
                for (const u of neighbors[v]) {
                    const l = Math.hypot(rest[3 * u] - rest[3 * v], rest[3 * u + 1] - rest[3 * v + 1], rest[3 * u + 2] - rest[3 * v + 2]);
                    if (d[v] + l < d[u]) { d[u] = d[v] + l; queue.push(u); }
                }
            }
            for (let i = 0; i < n; i++) W[i * G + k] = 1 / (d[i] + 0.5) ** 2;
        });
        for (let i = 0; i < n; i++) {
            let s = 0;
            for (let k = 0; k < G; k++) s += W[i * G + k];
            for (let k = 0; k < G; k++) W[i * G + k] /= s;
        }
        return W;
    }

    // Initial guess for the iterations after the grippers move by U (world frame): every
    // free node follows the blend of the gripper motions (exact for rigid motions).
    predict(P, U) {
        const { n } = this.strip, G = U.length, W = this.blend, isHandle = this.arap.isHandle;
        for (let i = 0; i < n; i++) {
            if (isHandle[i]) continue;
            const p = [P[3 * i], P[3 * i + 1], P[3 * i + 2]], q = [0, 0, 0];
            for (let k = 0; k < G; k++) {
                const a = apply(U[k], p), w = W[i * G + k];
                q[0] += w * a[0]; q[1] += w * a[1]; q[2] += w * a[2];
            }
            P[3 * i] = q[0]; P[3 * i + 1] = q[1]; P[3 * i + 2] = q[2];
        }
    }

    // Quasi-static loading from the rest shape to the gripper poses of `specs`.
    load(specs, steps = 24, iterations = 12) {
        const P = this.strip.rest.slice();
        const moves = specs.map((s, k) => relative(s, this.grippers[k].restPose.t));
        const saved = this.arap.snapshot();
        this.arap.quats.fill(0);
        for (let i = 0; i < this.arap.n; i++) this.arap.quats[4 * i] = 1;
        let poses = this.grippers.map((g) => g.restPose);
        for (let s = 1; s <= steps; s++) {
            const next = moves.map((D, k) => compose(power(D, s / steps), this.grippers[k].restPose));
            this.predict(P, next.map((T, k) => compose(T, inverse(poses[k]))));
            poses = next;
            this.placeHandles(P, poses);
            this.arap.solve(P, s === steps ? 4 * iterations : iterations);
        }
        const state = { P, poses, quats: this.arap.snapshot() };
        this.arap.restore(saved);
        return state;
    }

    setTarget(specs) {
        this.targetSpec = specs;
        this.target = this.load(specs);
        this.Y = perceived(this.strip, this.target.P);
    }

    reset() {
        this.P = this.initial.P.slice();
        this.poses = this.initial.poses.map((T) => ({ R: T.R.slice(), t: T.t.slice() }));
        this.arap.restore(this.initial.quats);
    }

    X() { return perceived(this.strip, this.P); }

    // Move every gripper by its rigid transform U_g (world frame) and let the object settle.
    move(U) {
        this.poses = this.poses.map((T, k) => orthonormalize(compose(U[k], T)));
        this.predict(this.P, U);
        this.placeHandles(this.P, this.poses);
        this.arap.solve(this.P, this.arapIterations);
    }

    // The simulated response to the gripper motions U (without changing the state).
    response(U, iterations = 30) {
        const P = this.P.slice(), saved = this.arap.snapshot();
        this.predict(P, U);
        this.placeHandles(P, this.poses.map((T, k) => compose(U[k], T)));
        this.arap.solve(P, iterations);
        this.arap.restore(saved);
        return P;
    }

    // An unreachable target: the target with a bump of `amplitude` cm along the normal of
    // its surface, centered on the piece. The grippers cannot produce it, since the ARAP
    // shape is determined by the gripper poses. Returns the perceived points Y and the
    // lattice positions P (for drawing).
    bumpedTarget(amplitude = 2) {
        const { strip } = this, Y = this.Y.slice(), N = new Float64Array(3 * strip.M), t = strip.topTris;
        for (let k = 0; k < t.length; k += 3) {
            const a = t[k], b = t[k + 1], c = t[k + 2];
            const u = [0, 1, 2].map((e) => Y[3 * b + e] - Y[3 * a + e]), v = [0, 1, 2].map((e) => Y[3 * c + e] - Y[3 * a + e]);
            const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            for (const i of [a, b, c]) for (let e = 0; e < 3; e++) N[3 * i + e] += n[e];
        }
        const disp = new Float64Array(3 * strip.M);
        for (let m = 0; m < strip.M; m++) {
            const l = Math.hypot(N[3 * m], N[3 * m + 1], N[3 * m + 2]) || 1, u = strip.uv[2 * m], v = strip.uv[2 * m + 1];
            const a = amplitude * Math.exp(-((u - 0.5) ** 2 / 0.018 + (v - 0.5) ** 2 / 0.08));
            for (let e = 0; e < 3; e++) disp[3 * m + e] = a * N[3 * m + e] / l;
        }
        const P = this.target.P.slice();
        for (let i = 0; i < strip.n; i++) for (let e = 0; e < 3; e++) P[3 * i + e] += disp[3 * strip.column[i] + e];
        for (let m = 0; m < strip.M; m++) for (let e = 0; e < 3; e++) Y[3 * m + e] += disp[3 * m + e];
        return { Y, P };
    }
}


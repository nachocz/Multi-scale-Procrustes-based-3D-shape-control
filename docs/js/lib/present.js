// Helpers shared by the pages: colors of the object (surface map, geodesic balls,
// error, geodesic distance), the SE(3) geodesic of a gripper, and gripper labels.
import { viridis, parula } from './charts.js';
import { palette } from './theme.js';
import { apply, compose, power } from '../sim/se3.js';

const hex = (c) => [1, 3, 5].map((i) => parseInt(c.slice(i, i + 2), 16) / 255);
const mixRgb = (a, b, t) => a.map((v, i) => v + (b[i] - v) * t);

// Color of the surface map: the paper colors matched points alike on X and Y. Here
// the color follows the position on the rest shape (u along, v across the piece).
export function matchColor(u, v) {
    const h = 0.42 + 0.55 * u, s = 0.62, l = 0.42 + 0.16 * v;
    const f = (n) => { const k = (n + h * 12) % 12, a = s * Math.min(l, 1 - l); return l - a * Math.max(-1, Math.min(k - 3, 9 - k, 1)); };
    return [f(0), f(8), f(4)];
}

// Per-node colors (Float32Array, 3 per lattice node) for the color modes:
//   'match'    surface-map colors,
//   'scales'   neutral, with the balls given by `inBall` (one Uint8Array per gripper),
//   'error'    distance between matched points |x_m - y_m| (X, Y, max for the scale),
//   'distance' geodesic distance D (Float64Array) with bands every `band` cm.
export function nodeColors(world, mode, data = {}) {
    const { strip } = world, n = strip.n, out = new Float32Array(3 * n), p = palette();
    const perPoint = new Array(strip.M);
    if (mode === 'match') {
        for (let m = 0; m < strip.M; m++) perPoint[m] = matchColor(strip.uv[2 * m], strip.uv[2 * m + 1]);
    } else if (mode === 'error') {
        const { X, Y } = data, e = new Float64Array(strip.M);
        let max = data.max || 0;
        for (let m = 0; m < strip.M; m++) { e[m] = Math.hypot(X[3 * m] - Y[3 * m], X[3 * m + 1] - Y[3 * m + 1], X[3 * m + 2] - Y[3 * m + 2]); if (!data.max) max = Math.max(max, e[m]); }
        for (let m = 0; m < strip.M; m++) perPoint[m] = viridis(e[m] / (max || 1));
    } else if (mode === 'distance') {
        const { D, band = 2, max } = data, top = max || Math.max(...D);
        for (let m = 0; m < strip.M; m++) {
            const t = D[m] / (top || 1), stripe = Math.floor(D[m] / band) % 2;
            perPoint[m] = parula(t).map((c) => c * (stripe ? 0.88 : 1));
        }
    } else {
        const base = hex(p.object), tints = [p.g1Soft, p.g2Soft, p.g3Soft].map(hex), { inBall = [] } = data;
        for (let m = 0; m < strip.M; m++) {
            let c = base, count = 0;
            inBall.forEach((flags, g) => { if (flags && flags[m]) { c = count ? mixRgb(c, tints[g], 0.5) : tints[g]; count++; } });
            perPoint[m] = c;
        }
    }
    for (let i = 0; i < n; i++) {
        const c = perPoint[strip.column[i]];
        out[3 * i] = c[0]; out[3 * i + 1] = c[1]; out[3 * i + 2] = c[2];
    }
    return out;
}

// Flags of the first `count` points of a ball order.
export function ballFlags(ball, count, M) {
    const flags = new Uint8Array(M);
    for (let k = 0; k < count; k++) flags[ball.order[k]] = 1;
    return flags;
}

// Points of the SE(3) geodesic exp(s log T) applied to the point c, s in [0, 1].
export function screwPath(T, c, samples = 40) {
    return Array.from({ length: samples }, (_, i) => apply(power(T, i / (samples - 1)), c));
}

// Gripper poses after their Procrustes transforms T_g (the poses they are driven towards).
export const ghostPoses = (poses, T) => poses.map((pose, k) => compose(T[k], pose));

export const gripperColor = (k) => palette()[`g${k + 1}`];
export const gripperName = (k) => `g${k + 1}`;

// A label position on an isoline: the segment point farthest from the center c.
export function isolineAnchor(segments, c) {
    let best = null, bd = -1;
    for (let s = 0; s + 2 < segments.length; s += 3) {
        const d = Math.hypot(segments[s] - c[0], segments[s + 1] - c[1], segments[s + 2] - c[2]);
        if (d > bd) { bd = d; best = [segments[s], segments[s + 1], segments[s + 2]]; }
    }
    return best;
}

// Multi-scale Procrustes shape control (Sec. II of the paper). At every iteration, with
// the current perceived points X(t) and their matched target points Y:
//   e(t)       = d_P(X(t), Y)                                         (2)
//   Omega_g(r) = geodesic ball of radius r about gripper g on X(t)
//   T_g(t, r)  = Procrustes transform of P(X_g(t, r), Y_g(r))         (1)
//   H_g(t, r)  = exp(dt log T_g(t, r))                                (3)
//   Xhat(r,r') = X + sum_g delta_g(m, r') (H_g(t, r) x_m - x_m)       (9), with (7)
//   ehat(r,r') = d_P(Xhat(r, r'), Y) - e(t)                           (10)
//   w(t, r')   = exp(-beta d_P(X(t), Xhat_{t-1}(r')) / M)              (8)
//   q(t, r)    = int ehat(t, r, r') w(t, r') dr'                      (11)
//   r*(t)      = r*(t - dt) - dq/dr (r*)                              (12)
//   U_g(t)     = exp(dt log T_g(t, r*(t)))                            (13)
// The scales are discretized into nScales values from r0 to R, as in the experiments;
// q is normalized by max |q| before the gradient step, as in the reference code, and
// the residual of (8) is divided by M, as in the reference code.
import { fastMarching } from './geodesic.js';
import { procrustes } from './procrustes.js';
import { applyAll, power } from './se3.js';

export const DEFAULTS = {
    nScales: 20,            // discrete scales from r0 to R
    beta: 1e4,              // relaxation of the LRB hypothesis, eq. (8)
    dt: 0.1,                // time step of the geodesic path, eqs. (3), (13)
    innerIterations: 10,    // gradient steps on the normalized q per iteration, eq. (12)
    superposition: 'sum',   // 'sum': x + sum_g delta_g (H_g x - x) (reference code); 'mean': eqs. (6), (9) with 1/G
    integrate: 'influence', // q integrates ehat over r' (eq. 11); 'action': over r (reference MATLAB code)
    planar: false,          // points in the plane z = 0 and actions in SE(2)
};

export class MultiScaleProcrustes {
    // surface: {tris, adjacency} of the perceived surface; sources: index of the point
    // at the center of each gripper; r0: smallest scale (the gripper size).
    constructor(surface, sources, r0, options = {}) {
        this.surface = surface;
        this.sources = Array.from(sources);
        this.G = this.sources.length;
        this.r0 = r0;
        this.options = { ...DEFAULTS, ...options };
    }

    set(options) { Object.assign(this.options, options); }

    // Align the target to the initial shape (T^{-1}(t0) applied to Y) and fix the scales.
    start(X0, Y) {
        const M = X0.length / 3, P = procrustes(Y, X0, null, this.options.planar);
        this.M = M;
        this.alignment = { R: P.R, t: P.t };          // T^{-1}(t0): maps Y onto X(t0)
        this.Y = applyAll(this.alignment, Y);
        const D = this.distances(X0);
        let R = 0;
        for (const d of D) for (let m = 0; m < M; m++) R = Math.max(R, d[m]);
        this.R = R;
        this.makeScales();
        this.k = 0;
        this.sStar = null;
        this.previous = null;
        return this.Y;
    }

    makeScales() {
        const n = this.options.nScales;
        this.scales = Float64Array.from({ length: n }, (_, s) => this.r0 + (this.R - this.r0) * s / (n - 1));
    }

    distances(X) {
        return this.sources.map((s) => fastMarching(X, this.surface.tris, this.surface.adjacency, [s]));
    }

    // Nested geodesic balls: the points of gripper g sorted by distance; Omega_g(r) is
    // the prefix of this order with D_g <= r (the largest scale holds every point).
    balls(D) {
        const { M } = this;
        return D.map((d) => {
            const order = Int32Array.from({ length: M }, (_, m) => m).sort((a, b) => d[a] - d[b]);
            const count = (r, last = false) => {
                if (last) return M;
                let lo = 0, hi = M;                    // first index with d > r
                while (lo < hi) { const mid = (lo + hi) >> 1; if (d[order[mid]] <= r) lo = mid + 1; else hi = mid; }
                return Math.max(1, lo);
            };
            return { order, count, d };
        });
    }

    // X + sum_g (A_g x_m - x_m) over the first counts[g] points of each order.
    superpose(X, increments, balls, counts, out) {
        out.set(X);
        const mean = this.options.superposition === 'mean', f = mean ? 1 / this.G : 1;
        for (let g = 0; g < this.G; g++) {
            const order = balls[g].order, inc = increments[g];
            for (let k = 0; k < counts[g]; k++) {
                const m = 3 * order[k];
                out[m] += f * inc[m]; out[m + 1] += f * inc[m + 1]; out[m + 2] += f * inc[m + 2];
            }
        }
        return out;
    }

    increments(X, transforms) {
        return transforms.map((T) => {
            const out = applyAll(T, X);
            for (let i = 0; i < out.length; i++) out[i] -= X[i];
            return out;
        });
    }

    // Full multi-scale analysis at the current state X (eqs. 1-11).
    analyze(X) {
        const { M, G, Y, scales, options } = this, nS = scales.length;
        const e = procrustes(X, Y, null, options.planar).d;
        const D = this.distances(X), balls = this.balls(D);
        const counts = Array.from({ length: nS }, (_, s) => balls.map((b) => b.count(scales[s], s === nS - 1)));
        // Procrustes analysis of every ball: T_g(t, r) and the residual d_P of the subset
        const T = counts.map((cs) => cs.map((c, g) => procrustes(X, Y, balls[g].order.subarray(0, c), options.planar)));
        const H = T.map((row) => row.map((P) => power(P, options.dt)));
        // error increment estimation ehat(r, r') (row: action scale r, column: influence scale r')
        const ehat = new Float64Array(nS * nS), Xhat = new Float64Array(3 * M);
        for (let s = 0; s < nS; s++) {
            const inc = this.increments(X, H[s]);
            for (let s2 = 0; s2 < nS; s2++) {
                this.superpose(X, inc, balls, counts[s2], Xhat);
                ehat[s * nS + s2] = procrustes(Xhat, Y, null, options.planar).d - e;
            }
        }
        // LRB validation w(t, r'): the previous action predicted with each influence scale r'
        const delta = new Float64Array(nS).fill(0);
        let validated = false;
        if (this.previous) {
            const p = this.previous, pred = new Float64Array(3 * M), inc = this.increments(p.X, p.U);
            for (let s2 = 0; s2 < nS; s2++) {
                this.superpose(p.X, inc, p.balls, p.counts[s2], pred);
                delta[s2] = procrustes(X, pred, null, options.planar).d / M;
            }
            validated = true;
        }
        const minDelta = Math.min(...delta);
        const w = Float64Array.from(delta, (d) => Math.exp(-options.beta * d));
        // relative weights (max 1): a common factor of w does not change the normalized q
        const wRel = Float64Array.from(delta, (d) => Math.exp(-options.beta * (d - minDelta)));
        // q(t, r) = int ehat(t, r, r') w(t, r') dr' (rectangle rule), eq. (11)
        const dr = nS > 1 ? scales[1] - scales[0] : 1, q = new Float64Array(nS);
        for (let s = 0; s < nS; s++) {
            let acc = 0;
            for (let s2 = 0; s2 < nS; s2++) {
                acc += options.integrate === 'action' ? ehat[s2 * nS + s] * wRel[s2] : ehat[s * nS + s2] * wRel[s2];
            }
            q[s] = acc * dr;
        }
        const qMax = Math.max(...q.map(Math.abs)) || 1;
        const qn = Float64Array.from(q, (v) => v / qMax);
        return { k: this.k, e, D, balls, counts, T, ehat, delta, w, wRel, validated, q, qn, scales, nS };
    }

    // Gradient of the normalized q at the fractional scale index s (central differences).
    gradient(qn, s) {
        const n = qn.length;
        const grad = (i) => (i <= 0 ? qn[1] - qn[0] : i >= n - 1 ? qn[n - 1] - qn[n - 2] : (qn[i + 1] - qn[i - 1]) / 2);
        const i0 = Math.max(0, Math.min(n - 1, Math.floor(s))), i1 = Math.min(n - 1, i0 + 1), f = s - i0;
        return (1 - f) * grad(i0) + f * grad(i1);
    }

    scaleAt(s) {
        const { scales } = this, n = scales.length;
        const i0 = Math.max(0, Math.min(n - 1, Math.floor(s))), i1 = Math.min(n - 1, i0 + 1), f = s - i0;
        return (1 - f) * scales[i0] + f * scales[i1];
    }

    // One control iteration: analysis, scale update (12), and the actions (13).
    step(X) {
        const A = this.analyze(X), { nS, qn, balls } = A, { options, M, Y, G } = this;
        let s;
        if (this.sStar === null) {
            s = 0;                                              // r*(t0) = argmin_r q(t0, r)
            for (let i = 1; i < nS; i++) if (qn[i] < qn[s]) s = i;
        } else {
            s = this.sStar;
            for (let it = 0; it < options.innerIterations; it++) {
                s = Math.max(0, Math.min(nS - 1, s - this.gradient(qn, s)));
            }
        }
        this.sStar = s;
        const rStar = this.scaleAt(s), last = s >= nS - 1 - 1e-9;
        const counts = balls.map((b) => b.count(rStar, last));
        const Tstar = counts.map((c, g) => procrustes(X, Y, balls[g].order.subarray(0, c), options.planar));
        const U = Tstar.map((T) => power(T, options.dt));
        // the estimated next shape at r = r' = r* (LRB model)
        const predicted = this.superpose(X, this.increments(X, U), balls, counts, new Float64Array(3 * M));
        this.previous = { X: X.slice(), U, balls, counts: A.counts };
        this.k += 1;
        return { ...A, sStar: s, rStar, ballCounts: counts, Tstar, U, predicted, G };
    }

    // The LRB estimate if each ball Omega_g(r) moved by the fraction lambda of its
    // Procrustes transform onto its target (lambda = 1: each ball lands on Y_g(r)).
    estimate(X, balls, countsAction, countsInfluence, lambda) {
        const T = countsAction.map((c, g) => procrustes(X, this.Y, balls[g].order.subarray(0, c), this.options.planar));
        const inc = this.increments(X, T.map((P) => power(P, lambda)));
        return { T, Xhat: this.superpose(X, inc, balls, countsInfluence, new Float64Array(X.length)) };
    }
}

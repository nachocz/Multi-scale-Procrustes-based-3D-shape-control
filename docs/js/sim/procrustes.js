// The Procrustes operator of eq. (1): (T, d_P) = P(X, Y). For two column-to-column
// matched point sets, T in SE(3) is the rigid transform that minimizes the residual and
// d_P = min_R ||R (X - Xbar) - (Y - Ybar)||_F is the Procrustes distance.
import { hornRotation } from './linalg.js';

// X, Y: flat arrays of matched points; idx: indices of the subset (null for all points);
// planar: points in the plane z = 0, with R a rotation about z (the SE(2) case; a 3D
// rotation could also flip a planar set over). Returns {R, t, d, cx, cy, n}: X is
// mapped to Y by p -> R p + t.
export function procrustes(X, Y, idx = null, planar = false) {
    const n = idx ? idx.length : X.length / 3;
    let cx0 = 0, cx1 = 0, cx2 = 0, cy0 = 0, cy1 = 0, cy2 = 0;
    for (let k = 0; k < n; k++) {
        const m = 3 * (idx ? idx[k] : k);
        cx0 += X[m]; cx1 += X[m + 1]; cx2 += X[m + 2];
        cy0 += Y[m]; cy1 += Y[m + 1]; cy2 += Y[m + 2];
    }
    cx0 /= n; cx1 /= n; cx2 /= n; cy0 /= n; cy1 /= n; cy2 /= n;
    const S = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (let k = 0; k < n; k++) {
        const m = 3 * (idx ? idx[k] : k);
        const a0 = X[m] - cx0, a1 = X[m + 1] - cx1, a2 = X[m + 2] - cx2;
        const b0 = Y[m] - cy0, b1 = Y[m + 1] - cy1, b2 = Y[m + 2] - cy2;
        S[0] += a0 * b0; S[1] += a0 * b1; S[2] += a0 * b2;
        S[3] += a1 * b0; S[4] += a1 * b1; S[5] += a1 * b2;
        S[6] += a2 * b0; S[7] += a2 * b1; S[8] += a2 * b2;
    }
    let R;
    if (planar) {               // angle of the 2D rotation: atan2(sum a x b, sum a . b)
        const th = Math.atan2(S[1] - S[3], S[0] + S[4]), c = Math.cos(th), s = Math.sin(th);
        R = [c, -s, 0, s, c, 0, 0, 0, 1];
    } else R = hornRotation(S);
    let d2 = 0;
    for (let k = 0; k < n; k++) {
        const m = 3 * (idx ? idx[k] : k);
        const a0 = X[m] - cx0, a1 = X[m + 1] - cx1, a2 = X[m + 2] - cx2;
        const r0 = R[0] * a0 + R[1] * a1 + R[2] * a2 - (Y[m] - cy0);
        const r1 = R[3] * a0 + R[4] * a1 + R[5] * a2 - (Y[m + 1] - cy1);
        const r2 = R[6] * a0 + R[7] * a1 + R[8] * a2 - (Y[m + 2] - cy2);
        d2 += r0 * r0 + r1 * r1 + r2 * r2;
    }
    const t = [cy0 - (R[0] * cx0 + R[1] * cx1 + R[2] * cx2), cy1 - (R[3] * cx0 + R[4] * cx1 + R[5] * cx2), cy2 - (R[6] * cx0 + R[7] * cx1 + R[8] * cx2)];
    return { R, t, d: Math.sqrt(d2), cx: [cx0, cx1, cx2], cy: [cy0, cy1, cy2], n };
}

// Procrustes distance d_P(X, Y) of eq. (1) (the shape error e(t) of eq. (2) for X(t), Y).
export const procrustesDistance = (X, Y, idx = null, planar = false) => procrustes(X, Y, idx, planar).d;

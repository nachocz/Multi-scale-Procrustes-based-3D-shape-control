// Terms of the Lyapunov analysis of Sec. III. With V(t) = 1/2 ||X(t) - Y||_F^2 and the
// points of Omega_g(t, r*) moving as dx/dt = log(T_g) x (eq. 5), expressed in the frame
// F_g at the centroid xbar_g of X_g (axes aligned with the world axes), the contribution
// of gripper g is
//     dV_g/dt = - sum_m y_m^T log(R_g) x_m  -  M_g ybar^T V_g ybar   (eq. 25)
// The first term is non-positive by Lemma 2 and the second by Lemma 3, for |theta| < pi.
import { mul3, mulVec3, skew, norm3 } from './linalg.js';
import { logSO3, rotationAngle } from './se3.js';

// V_g of eq. (17): I - log(R)/2 + (1/theta^2)(1 - theta (1 + cos theta) / (2 sin theta)) log(R)^2.
export function matrixV(R) {
    const w = logSO3(R), th = norm3(w), W = skew(w), W2 = mul3(W, W);
    const D = th < 1e-4 ? 1 / 12 + th * th / 720 : (1 - th * (1 + Math.cos(th)) / (2 * Math.sin(th))) / (th * th);
    return { V: [1, 0, 0, 0, 1, 0, 0, 0, 1].map((v, i) => v - 0.5 * W[i] + D * W2[i]), W, theta: th };
}

// theta / (2 sin theta) of eq. (16) and c(theta) of eq. (21).
export const rotationFactor = (th) => (Math.abs(th) < 1e-6 ? 0.5 : th / (2 * Math.sin(th)));
export const cFactor = (th) => (Math.abs(th) < 1e-6 ? 0 : 1 - th * (1 + Math.cos(th)) / (2 * Math.sin(th)));

// Per-gripper terms at the current state (X, Y: matched points; balls, counts: the
// balls at r*; T: the Procrustes transforms at r*). Rates per unit of time.
export function lyapunovTerms(X, Y, balls, counts, T) {
    return T.map((Tg, g) => {
        const idx = balls[g].order.subarray(0, counts[g]), Mg = idx.length;
        const xb = [0, 0, 0];
        for (const m of idx) { xb[0] += X[3 * m]; xb[1] += X[3 * m + 1]; xb[2] += X[3 * m + 2]; }
        xb[0] /= Mg; xb[1] /= Mg; xb[2] /= Mg;
        const { V, W, theta } = matrixV(Tg.R);
        let rot = 0;
        const yb = [0, 0, 0];
        for (const m of idx) {
            const x = [X[3 * m] - xb[0], X[3 * m + 1] - xb[1], X[3 * m + 2] - xb[2]];
            const y = [Y[3 * m] - xb[0], Y[3 * m + 1] - xb[1], Y[3 * m + 2] - xb[2]];
            const Wx = mulVec3(W, x);
            rot += y[0] * Wx[0] + y[1] * Wx[1] + y[2] * Wx[2];
            yb[0] += y[0]; yb[1] += y[1]; yb[2] += y[2];
        }
        yb[0] /= Mg; yb[1] /= Mg; yb[2] /= Mg;
        const Vy = mulVec3(V, yb);
        const translation = -Mg * (yb[0] * Vy[0] + yb[1] * Vy[1] + yb[2] * Vy[2]);
        return { rotation: -rot, translation, total: -rot + translation, theta: rotationAngle(Tg.R), Mg, ybar: yb };
    });
}

// V(t) = 1/2 ||X - Y||_F^2.
export function lyapunov(X, Y) {
    let s = 0;
    for (let i = 0; i < X.length; i++) s += (X[i] - Y[i]) ** 2;
    return 0.5 * s;
}

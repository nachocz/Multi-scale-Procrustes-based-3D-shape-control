// Objects of the demos: flat cut-outs (like the foam pieces of the experiments) of
// thickness h, built as a structured lattice of nu x nv x nz nodes. In the plane, node
// (i, j) lies at c(u) + s n(u), on the centerline c(u) of the piece (u = i / (nu - 1))
// and across its width s in [-wMinus(u), wPlus(u)]; layer k sets the height z.
//
// The lattice is the volumetric model simulated with ARAP (js/sim/arap.js). The top
// layer of nodes is the perceived surface: its M = nu * nv points form the matrix X(t)
// of the paper (the surface that a camera above the object reconstructs).

export function makeStrip(spec) {
    const { nu, nv, nz = 2, h } = spec;
    const n = nu * nv * nz, rest = new Float64Array(3 * n);
    const index = (i, j, k) => (i * nv + j) * nz + k;
    const frame = (u) => {
        const e = 1e-4, a = spec.center(Math.max(0, u - e)), b = spec.center(Math.min(1, u + e));
        const tx = b[0] - a[0], ty = b[1] - a[1], l = Math.hypot(tx, ty);
        return { c: spec.center(u), t: [tx / l, ty / l], n: [-ty / l, tx / l] };
    };
    const uv = new Float64Array(2 * nu * nv);
    for (let i = 0; i < nu; i++) {
        const u = i / (nu - 1), { c, n: nrm } = frame(u), wm = spec.wMinus(u), wp = spec.wPlus(u);
        for (let j = 0; j < nv; j++) {
            const v = j / (nv - 1), s = -wm + v * (wm + wp);
            uv[2 * (i * nv + j)] = u; uv[2 * (i * nv + j) + 1] = v;
            for (let k = 0; k < nz; k++) {
                const m = 3 * index(i, j, k);
                rest[m] = c[0] + s * nrm[0];
                rest[m + 1] = c[1] + s * nrm[1];
                rest[m + 2] = nz > 1 ? -h / 2 + h * k / (nz - 1) : 0;
            }
        }
    }

    // 26-neighborhood of the lattice: the edges of the ARAP cells
    const neighbors = [];
    for (let i = 0; i < nu; i++) for (let j = 0; j < nv; j++) for (let k = 0; k < nz; k++) {
        const list = [];
        for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) for (let dk = -1; dk <= 1; dk++) {
            if (!di && !dj && !dk) continue;
            const a = i + di, b = j + dj, c = k + dk;
            if (a < 0 || b < 0 || c < 0 || a >= nu || b >= nv || c >= nz) continue;
            list.push(index(a, b, c));
        }
        neighbors[index(i, j, k)] = Int32Array.from(list);
    }

    // closed boundary surface (for rendering), oriented outwards
    const surface = [];
    const quad = (a, b, c, d, inner) => {
        const tri = (p, q, r) => {
            const P = (x) => [rest[3 * x], rest[3 * x + 1], rest[3 * x + 2]];
            const A = P(p), B = P(q), C = P(r);
            const nx = (B[1] - A[1]) * (C[2] - A[2]) - (B[2] - A[2]) * (C[1] - A[1]);
            const ny = (B[2] - A[2]) * (C[0] - A[0]) - (B[0] - A[0]) * (C[2] - A[2]);
            const nzz = (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]);
            const o = [0, 1, 2].map((e) => (A[e] + B[e] + C[e]) / 3 - inner[e]);
            if (nx * o[0] + ny * o[1] + nzz * o[2] < 0) surface.push(p, r, q); else surface.push(p, q, r);
        };
        tri(a, b, c); tri(a, c, d);
    };
    const mean = (ids) => [0, 1, 2].map((e) => ids.reduce((s, x) => s + rest[3 * x + e], 0) / ids.length);
    // (a planar piece, nz = 1, has no closed surface: its surface is the sheet itself)
    for (let i = 0; i < nu - 1 && nz > 1; i++) for (let j = 0; j < nv - 1; j++) {
        for (const [k, kin] of [[nz - 1, nz - 2], [0, 1]]) {
            const q = [index(i, j, k), index(i + 1, j, k), index(i + 1, j + 1, k), index(i, j + 1, k)];
            const inner = mean([index(i, j, kin), index(i + 1, j, kin), index(i + 1, j + 1, kin), index(i, j + 1, kin)]);
            if ((i + j) % 2) quad(q[0], q[1], q[2], q[3], inner); else quad(q[1], q[2], q[3], q[0], inner);
        }
    }
    for (let k = 0; k < nz - 1; k++) {
        for (let i = 0; i < nu - 1; i++) for (const [j, jin] of [[0, 1], [nv - 1, nv - 2]]) {
            quad(index(i, j, k), index(i + 1, j, k), index(i + 1, j, k + 1), index(i, j, k + 1),
                mean([index(i, jin, k), index(i + 1, jin, k), index(i + 1, jin, k + 1), index(i, jin, k + 1)]));
        }
        for (let j = 0; j < nv - 1; j++) for (const [i, iin] of [[0, 1], [nu - 1, nu - 2]]) {
            quad(index(i, j, k), index(i, j + 1, k), index(i, j + 1, k + 1), index(i, j, k + 1),
                mean([index(iin, j, k), index(iin, j + 1, k), index(iin, j + 1, k + 1), index(iin, j, k + 1)]));
        }
    }

    // perceived surface: the top layer, with M = nu * nv points and its own triangles
    const M = nu * nv, top = new Int32Array(M), column = new Int32Array(n), topTris = [];
    for (let i = 0; i < nu; i++) for (let j = 0; j < nv; j++) {
        top[i * nv + j] = index(i, j, nz - 1);
        for (let k = 0; k < nz; k++) column[index(i, j, k)] = i * nv + j;
    }
    for (let i = 0; i < nu - 1; i++) for (let j = 0; j < nv - 1; j++) {
        const a = i * nv + j, b = (i + 1) * nv + j, c = (i + 1) * nv + j + 1, d = i * nv + j + 1;
        if ((i + j) % 2) topTris.push(a, b, c, a, c, d); else topTris.push(a, b, d, b, c, d);
    }
    if (nz === 1) for (const m of topTris) surface.push(top[m]);
    return {
        spec, nu, nv, nz, n, M, index, frame, rest, neighbors, uv,
        surface: Uint32Array.from(surface), top, column, topTris: Int32Array.from(topTris),
    };
}

// Positions of the perceived points (top layer) of the lattice positions P.
export function perceived(strip, P, out = new Float64Array(3 * strip.M)) {
    const { top } = strip;
    for (let m = 0; m < top.length; m++) {
        out[3 * m] = P[3 * top[m]]; out[3 * m + 1] = P[3 * top[m] + 1]; out[3 * m + 2] = P[3 * top[m] + 2];
    }
    return out;
}

// Lattice nodes held by a gripper centered at (i, j) of the top layer: |i' - i| <= di,
// |j' - j| <= dj, through the whole thickness.
export function graspedNodes(strip, { i, j, di, dj }) {
    const out = [];
    for (let a = Math.max(0, i - di); a <= Math.min(strip.nu - 1, i + di); a++) {
        for (let b = Math.max(0, j - dj); b <= Math.min(strip.nv - 1, j + dj); b++) {
            for (let k = 0; k < strip.nz; k++) out.push(strip.index(a, b, k));
        }
    }
    return Int32Array.from(out);
}

// Mean edge length of the perceived surface.
export function meanEdge(P, tris) {
    let s = 0, c = 0;
    for (let k = 0; k < tris.length; k += 3) for (let e = 0; e < 3; e++) {
        const a = tris[k + e], b = tris[k + (e + 1) % 3];
        s += Math.hypot(P[3 * a] - P[3 * b], P[3 * a + 1] - P[3 * b + 1], P[3 * a + 2] - P[3 * b + 2]); c++;
    }
    return s / c;
}

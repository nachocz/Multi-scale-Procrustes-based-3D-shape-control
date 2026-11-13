// Geodesic distances on a triangulated surface by fast marching (Kimmel and Sethian,
// 1998), and the isolines of a scalar field on the surface. The geodesic balls
// Omega_g(t, r) of the paper are the sublevel sets {D_g <= r} of the distance D_g to
// gripper g, computed on the current shape X(t); their boundaries are the isolines D_g = r.

// Vertex -> incident triangles, as CSR arrays.
export function vertexTriangles(tris, nVerts) {
    const count = new Int32Array(nVerts + 1);
    for (let k = 0; k < tris.length; k++) count[tris[k] + 1]++;
    for (let i = 0; i < nVerts; i++) count[i + 1] += count[i];
    const list = new Int32Array(tris.length), fill = count.slice(0, nVerts);
    for (let k = 0; k < tris.length; k++) list[fill[tris[k]]++] = (k / 3) | 0;
    return { start: count, list };
}

class Heap {                     // binary min-heap of (key, vertex), with lazy deletion
    constructor() { this.k = []; this.v = []; }
    get size() { return this.k.length; }
    push(key, vert) {
        const { k, v } = this;
        let i = k.length;
        k.push(key); v.push(vert);
        while (i > 0) {
            const p = (i - 1) >> 1;
            if (k[p] <= key) break;
            k[i] = k[p]; v[i] = v[p]; i = p;
        }
        k[i] = key; v[i] = vert;
    }
    pop() {
        const { k, v } = this, top = v[0], key = k.pop(), vert = v.pop();
        const n = k.length;
        if (n) {
            let i = 0;
            for (;;) {
                let c = 2 * i + 1;
                if (c >= n) break;
                if (c + 1 < n && k[c + 1] < k[c]) c++;
                if (k[c] >= key) break;
                k[i] = k[c]; v[i] = v[c]; i = c;
            }
            k[i] = key; v[i] = vert;
        }
        return top;
    }
}

// Update of the distance at x3 from the triangle (x1, x2, x3) with known t1, t2: the
// planar front through x1, x2 with unit slope, accepted only if its characteristic
// enters x3 from inside the triangle. Otherwise the edges give the update (Dijkstra).
function triangleUpdate(P, i1, i2, i3, t1, t2) {
    const e10 = P[3 * i1] - P[3 * i3], e11 = P[3 * i1 + 1] - P[3 * i3 + 1], e12 = P[3 * i1 + 2] - P[3 * i3 + 2];
    const e20 = P[3 * i2] - P[3 * i3], e21 = P[3 * i2 + 1] - P[3 * i3 + 1], e22 = P[3 * i2 + 2] - P[3 * i3 + 2];
    const a = e10 * e10 + e11 * e11 + e12 * e12, b = e10 * e20 + e11 * e21 + e12 * e22, c = e20 * e20 + e21 * e21 + e22 * e22;
    const edge = Math.min(t1 + Math.sqrt(a), t2 + Math.sqrt(c));
    const det = a * c - b * b;
    if (det <= 1e-12 * a * c) return edge;
    const A = (a + c - 2 * b) / det, B = ((c - b) * t1 + (a - b) * t2) / det, C = (c * t1 * t1 - 2 * b * t1 * t2 + a * t2 * t2) / det - 1;
    const disc = B * B - A * C;
    if (disc < 0) return edge;
    const t3 = (B + Math.sqrt(disc)) / A;
    const d1 = t3 - t1, d2 = t3 - t2;
    if (d1 < 0 || d2 < 0 || c * d1 - b * d2 < 0 || a * d2 - b * d1 < 0) return edge;
    return Math.min(t3, edge);
}

// Distances from the source vertices to every vertex (P: positions of the surface).
export function fastMarching(P, tris, adjacency, sources) {
    const n = P.length / 3, T = new Float64Array(n).fill(Infinity), alive = new Uint8Array(n), heap = new Heap();
    for (const s of sources) { T[s] = 0; heap.push(0, s); }
    const { start, list } = adjacency;
    // the two rings about a source get their straight-line distance: near a point source
    // the front is not planar, and the triangle update would overestimate these values
    for (const s of sources) {
        const ring = new Set([s]);
        for (let depth = 0; depth < 2; depth++) {
            for (const v of [...ring]) {
                for (let a = start[v]; a < start[v + 1]; a++) {
                    const f = 3 * list[a];
                    ring.add(tris[f]); ring.add(tris[f + 1]); ring.add(tris[f + 2]);
                }
            }
        }
        for (const u of ring) {
            const d = Math.hypot(P[3 * u] - P[3 * s], P[3 * u + 1] - P[3 * s + 1], P[3 * u + 2] - P[3 * s + 2]);
            if (d < T[u]) { T[u] = d; heap.push(d, u); }
        }
    }
    while (heap.size) {
        const v = heap.pop();
        if (alive[v]) continue;
        alive[v] = 1;
        for (let a = start[v]; a < start[v + 1]; a++) {
            const f = 3 * list[a], tri = [tris[f], tris[f + 1], tris[f + 2]];
            for (let r = 0; r < 3; r++) {
                const u = tri[r];
                if (alive[u]) continue;
                const p = tri[(r + 1) % 3], q = tri[(r + 2) % 3];
                let cand;
                if (alive[p] && alive[q]) cand = triangleUpdate(P, p, q, u, T[p], T[q]);
                else {
                    const k = alive[p] ? p : q;
                    if (!alive[k]) continue;
                    cand = T[k] + Math.hypot(P[3 * k] - P[3 * u], P[3 * k + 1] - P[3 * u + 1], P[3 * k + 2] - P[3 * u + 2]);
                }
                if (cand < T[u]) { T[u] = cand; heap.push(cand, u); }
            }
        }
    }
    return T;
}

// Isoline {f = level} by linear interpolation on each triangle: a list of segments
// (x0, y0, z0, x1, y1, z1, ...) as a Float32Array.
export function isoline(P, tris, f, level) {
    const out = [];
    const cross = (i, j) => {
        const s = (level - f[i]) / (f[j] - f[i]);
        out.push(P[3 * i] + s * (P[3 * j] - P[3 * i]), P[3 * i + 1] + s * (P[3 * j + 1] - P[3 * i + 1]), P[3 * i + 2] + s * (P[3 * j + 2] - P[3 * i + 2]));
    };
    for (let k = 0; k < tris.length; k += 3) {
        const a = tris[k], b = tris[k + 1], c = tris[k + 2];
        const ia = f[a] < level, ib = f[b] < level, ic = f[c] < level;
        if (ia === ib && ib === ic) continue;
        if (ia !== ib) cross(a, b);
        if (ib !== ic) cross(b, c);
        if (ic !== ia) cross(c, a);
    }
    return new Float32Array(out);
}

// Overview page: previews of the three interactive pages, drawn on canvases from the
// simulation modules (geodesic isolines on a piece, and a short closed-loop run).
import { cssColor, parula, signedColor } from '../lib/charts.js';
import { Plane2D } from '../lib/plane2d.js';
import { initPage } from '../lib/shell.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { fastMarching, isoline, vertexTriangles } from '../sim/geodesic.js';
import { Session } from '../sim/session.js';
import { lyapunov } from '../sim/stability.js';
import { World } from '../sim/world.js';

await initPage();

const painters = [];
function preview(canvas, draw) {
    const paint = () => {
        const dpr = Math.min(window.devicePixelRatio || 1, 2), w = canvas.clientWidth, h = canvas.clientHeight;
        if (!w || !h) return;
        canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.fillStyle = palette().canvas; ctx.fillRect(0, 0, w, h);
        ctx.lineJoin = 'round'; ctx.lineCap = 'round';
        draw(ctx, w, h, palette());
    };
    new ResizeObserver(paint).observe(canvas);
    onThemeChange(paint);
    painters.push(paint);
    return paint;
}
const font = () => getComputedStyle(document.body).fontFamily;
const caption = (ctx, text, p) => { ctx.fillStyle = p.muted; ctx.font = `600 10px ${font()}`; ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic'; ctx.fillText(text, 12, 16); };

// 0: the U-shaped piece of the 2D page with the geodesic ball of g1
const flat = new World('ushape'), flatTris = flat.surface.tris;
const flatD = fastMarching(flat.restX, flatTris, vertexTriangles(flatTris, flat.M), [flat.grippers[0].source]);
const plane = new Plane2D(document.getElementById('preview0'));
// the bounds of the piece and of the gripper boxes, with room for the caption above
plane.fit([flat.restX, Float64Array.from([0, flat.restX.reduce((m, v, i) => (i % 3 === 1 ? Math.max(m, v) : m), -Infinity) + 6, 0])], 0.12);
const paint0 = () => {
    if (!plane.begin()) return;
    const p = palette(), r = 9, mean = (k) => (flatD[flatTris[3 * k]] + flatD[flatTris[3 * k + 1]] + flatD[flatTris[3 * k + 2]]) / 3;
    plane.mesh(flat.restX, flatTris, (k) => (mean(k) <= r ? p.g1Soft : p.object));
    plane.segments(isoline(flat.restX, flatTris, flatD, r), { color: p.g1, width: 2.6 });
    flat.poses.forEach((pose, k) => plane.gripper(pose, flat.grippers[k].half, { color: p[`g${k + 1}`], fill: alpha(p.gripper, 0.3) }));
    plane.caption('2D: the geodesic ball of g1 at r = 9 cm', { color: p.muted, size: 10 });
};
new ResizeObserver(paint0).observe(document.getElementById('preview0'));
onThemeChange(paint0);

// 1: plan view of the piece with an appendix, banded by the geodesic distance to g2, isolines every 3 cm
const piece = new World('appendix');
const X0 = piece.restX, tris = piece.surface.tris, adjacency = vertexTriangles(tris, piece.M);
const D2 = fastMarching(X0, tris, adjacency, [piece.grippers[1].source]);
const Dmax = Math.max(...D2);
preview(document.getElementById('preview1'), (ctx, w, h, p) => {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let m = 0; m < piece.M; m++) { x0 = Math.min(x0, X0[3 * m]); x1 = Math.max(x1, X0[3 * m]); y0 = Math.min(y0, X0[3 * m + 1]); y1 = Math.max(y1, X0[3 * m + 1]); }
    const s = Math.min((w - 40) / (x1 - x0), (h - 46) / (y1 - y0)), ox = (w - s * (x1 + x0)) / 2, oy = (h + 10) / 2 + s * (y1 + y0) / 2;
    const P = (m) => [ox + s * X0[3 * m], oy - s * X0[3 * m + 1]];
    for (let k = 0; k < tris.length; k += 3) {
        const d = (D2[tris[k]] + D2[tris[k + 1]] + D2[tris[k + 2]]) / 3, band = Math.floor(d / 3) % 2;
        const c = parula(0.15 + 0.8 * d / Dmax).map((v) => v * (band ? 0.9 : 1));
        ctx.fillStyle = cssColor(c); ctx.strokeStyle = cssColor(c); ctx.lineWidth = 0.6;
        ctx.beginPath(); ctx.moveTo(...P(tris[k])); ctx.lineTo(...P(tris[k + 1])); ctx.lineTo(...P(tris[k + 2])); ctx.closePath(); ctx.fill(); ctx.stroke();
    }
    const project = (x, y) => [ox + s * x, oy - s * y];
    for (const [r, width] of [[4.5, 2.6], [9, 1.8], [13.5, 1.3]]) {
        const seg = isoline(X0, tris, D2, r);
        for (const [color, w2] of [[p.dark ? '#000000' : '#ffffff', width + 2.2], [p.dark ? '#ffffff' : '#1b1e22', width]]) {
            ctx.strokeStyle = color; ctx.lineWidth = w2; ctx.beginPath();
            for (let i = 0; i + 5 < seg.length; i += 6) { ctx.moveTo(...project(seg[i], seg[i + 1])); ctx.lineTo(...project(seg[i + 3], seg[i + 4])); }
            ctx.stroke();
        }
    }
    for (const g of piece.grippers) {
        const c = P(g.source), hw = s * g.half[0], hh = s * g.half[1];
        ctx.fillStyle = alpha(p.gripper, 0.45); ctx.strokeStyle = p.gripper; ctx.lineWidth = 1.2;
        ctx.fillRect(c[0] - hw, c[1] - hh, 2 * hw, 2 * hh); ctx.strokeRect(c[0] - hw, c[1] - hh, 2 * hw, 2 * hh);
    }
    caption(ctx, 'geodesic balls about the gripper of the appendix', p);
});

// 2 and 3: a short closed-loop run of the anti-symmetric bending, computed after the first paint
const run = { q: [], r: [], V: [], scales: null };
const paint2 = preview(document.getElementById('preview2'), (ctx, w, h, p) => {
    caption(ctx, 'q(t, r) and r*(t), anti-symmetric bending', p);
    if (!run.scales) { ctx.fillStyle = p.muted; ctx.font = `11px ${font()}`; ctx.textAlign = 'center'; ctx.fillText('computing…', w / 2, h / 2); return; }
    const L = 12, R = w - 12, T = 26, B = h - 12, n = run.scales.length, cols = run.q.length;
    run.q.forEach((col, k) => {
        for (let i = 0; i < n; i++) {
            ctx.fillStyle = cssColor(signedColor(col[i]));
            ctx.fillRect(L + k / cols * (R - L), B - (i + 1) / n * (B - T), (R - L) / cols + 0.8, (B - T) / n + 0.8);
        }
    });
    const r0 = run.scales[0], r1 = run.scales[n - 1];
    ctx.strokeStyle = p.rstar; ctx.lineWidth = 2.4; ctx.beginPath();
    run.r.forEach((r, k) => { const x = L + (k + 0.5) / cols * (R - L), y = B - ((r - r0) / (r1 - r0) * (n - 1) + 0.5) / n * (B - T); if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
});
const paint3 = preview(document.getElementById('preview3'), (ctx, w, h, p) => {
    caption(ctx, 'Lyapunov function V(t), log scale', p);
    if (!run.V.length) return;
    const L = 16, R = w - 14, T = 30, B = h - 16, lo = Math.log10(Math.max(1e-4, Math.min(...run.V))), hi = Math.log10(Math.max(...run.V));
    ctx.strokeStyle = p.grid; ctx.lineWidth = 1;
    for (let e = Math.ceil(lo); e <= hi; e++) { const y = B - (e - lo) / (hi - lo) * (B - T); ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(R, y); ctx.stroke(); }
    ctx.strokeStyle = p.ink; ctx.lineWidth = 2; ctx.beginPath();
    run.V.forEach((v, k) => { const x = L + k / (run.V.length - 1) * (R - L), y = B - (Math.log10(Math.max(v, 1e-4)) - lo) / (hi - lo) * (B - T); if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
    ctx.fillStyle = p.good; ctx.font = `700 11px ${font()}`; ctx.textAlign = 'right'; ctx.fillText('dV_g/dt ≤ 0', R, T + 4);
});

setTimeout(() => {
    const session = new Session(new World('sbend'), { noise: 0.5 });
    run.scales = session.out.scales;
    let k = 0;
    const slice = () => {
        const start = performance.now();
        while (k < 90 && performance.now() - start < 30) {
            run.q.push(session.out.qn); run.r.push(session.out.rStar); run.V.push(lyapunov(session.world.X(), session.ctrl.Y));
            session.step(); k++;
        }
        paint2(); paint3();
        if (k < 90) setTimeout(slice, 0);
    };
    slice();
}, 60);

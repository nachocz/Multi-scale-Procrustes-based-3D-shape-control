// 2D drawing of a planar piece on a canvas: world coordinates in cm (y up), a frame that
// fits given points, and helpers for meshes, outlines, isolines, grippers, arrows and
// labels with a halo, so that every layer of the 2D page is tagged on the drawing.
import { alpha, palette } from './theme.js';

export class Plane2D {
    constructor(canvas) {
        this.canvas = canvas;
        this.bounds = [-10, 10, -10, 10];
    }

    // Fit the bounds of the points (flat xyz arrays) with a margin (fraction of the size).
    fit(pointSets, margin = 0.12) {
        let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
        for (const P of pointSets) for (let i = 0; i < P.length; i += 3) {
            x0 = Math.min(x0, P[i]); x1 = Math.max(x1, P[i]); y0 = Math.min(y0, P[i + 1]); y1 = Math.max(y1, P[i + 1]);
        }
        const mx = margin * (x1 - x0 + y1 - y0) / 2;
        this.bounds = [x0 - mx, x1 + mx, y0 - mx, y1 + mx];
    }

    // Start a frame: size the canvas, clear it, and set the world-to-pixel mapping.
    begin() {
        const c = this.canvas, dpr = Math.min(window.devicePixelRatio || 1, 2), w = c.clientWidth, h = c.clientHeight;
        if (!w || !h) return null;
        if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
        const ctx = c.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const p = palette();
        ctx.fillStyle = p.canvas; ctx.fillRect(0, 0, w, h);
        const [x0, x1, y0, y1] = this.bounds, s = Math.min(w / (x1 - x0), h / (y1 - y0));
        this.scale = s;
        this.ox = (w - s * (x1 + x0)) / 2;
        this.oy = (h + s * (y1 + y0)) / 2;
        this.ctx = ctx; this.p = p; this.w = w; this.h = h;
        this.font = getComputedStyle(document.body).fontFamily;
        ctx.lineJoin = 'round'; ctx.lineCap = 'round';
        return ctx;
    }

    px(x, y) { return [this.ox + this.scale * x, this.oy - this.scale * y]; }
    world(px, py) { return [(px - this.ox) / this.scale, (this.oy - py) / this.scale]; }

    // Filled triangles; color(k) gives the CSS color of triangle k (index into tris / 3).
    mesh(P, tris, color, { stroke = true } = {}) {
        const { ctx } = this;
        for (let k = 0; k < tris.length; k += 3) {
            const c = color(k / 3);
            if (!c) continue;
            const a = this.px(P[3 * tris[k]], P[3 * tris[k] + 1]), b = this.px(P[3 * tris[k + 1]], P[3 * tris[k + 1] + 1]), d = this.px(P[3 * tris[k + 2]], P[3 * tris[k + 2] + 1]);
            ctx.fillStyle = c; ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.lineTo(...d); ctx.closePath(); ctx.fill();
            if (stroke) { ctx.strokeStyle = c; ctx.lineWidth = 0.7; ctx.stroke(); }
        }
    }

    // Closed polygon through the points idx of P.
    outline(P, idx, { color, width = 1.5, dash = [], fill = null } = {}) {
        const { ctx } = this;
        ctx.beginPath();
        idx.forEach((m, k) => { const q = this.px(P[3 * m], P[3 * m + 1]); if (k) ctx.lineTo(...q); else ctx.moveTo(...q); });
        ctx.closePath();
        if (fill) { ctx.fillStyle = fill; ctx.fill(); }
        ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); ctx.setLineDash([]);
    }

    // Segments (x0, y0, z0, x1, y1, z1, ...), with an optional halo below them.
    segments(S, { color, width = 2, halo = true, dash = [] } = {}) {
        const { ctx, p } = this;
        const path = () => {
            ctx.beginPath();
            for (let i = 0; i + 5 < S.length; i += 6) { ctx.moveTo(...this.px(S[i], S[i + 1])); ctx.lineTo(...this.px(S[i + 3], S[i + 4])); }
        };
        if (halo) { path(); ctx.strokeStyle = alpha(p.canvas, 0.9); ctx.lineWidth = width + 3; ctx.stroke(); }
        path(); ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); ctx.setLineDash([]);
    }

    circle(c, r, { color, width = 1.4, dash = [6, 5], fill = null } = {}) {
        const { ctx } = this, q = this.px(c[0], c[1]);
        ctx.beginPath(); ctx.arc(q[0], q[1], r * this.scale, 0, 2 * Math.PI);
        if (fill) { ctx.fillStyle = fill; ctx.fill(); }
        ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); ctx.setLineDash([]);
    }

    dot(c, r, color) { const { ctx } = this, q = this.px(c[0], c[1]); ctx.fillStyle = color; ctx.beginPath(); ctx.arc(q[0], q[1], r, 0, 2 * Math.PI); ctx.fill(); }

    polyline(pts, { color, width = 1.5, dash = [] } = {}) {
        const { ctx } = this;
        ctx.beginPath();
        pts.forEach((v, k) => { const q = this.px(v[0], v[1]); if (k) ctx.lineTo(...q); else ctx.moveTo(...q); });
        ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); ctx.setLineDash([]);
    }

    arrow(a, b, { color, width = 1.3, head = 5 } = {}) {
        const { ctx } = this, A = this.px(a[0], a[1]), B = this.px(b[0], b[1]);
        const dx = B[0] - A[0], dy = B[1] - A[1], l = Math.hypot(dx, dy);
        if (l < 0.5) return;
        ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
        ctx.beginPath(); ctx.moveTo(...A); ctx.lineTo(...B); ctx.stroke();
        const ux = dx / l, uy = dy / l, hh = Math.min(head, l * 0.6);
        ctx.beginPath(); ctx.moveTo(...B); ctx.lineTo(B[0] - hh * ux + 0.5 * hh * uy, B[1] - hh * uy - 0.5 * hh * ux);
        ctx.lineTo(B[0] - hh * ux - 0.5 * hh * uy, B[1] - hh * uy + 0.5 * hh * ux); ctx.closePath(); ctx.fill();
    }

    // Gripper: a box at pose {R, t} with half sizes half[0], half[1]; anchored grippers are hatched.
    gripper(pose, half, { color, fill, dash = [], anchored = false, width = 1.6 } = {}) {
        const { ctx } = this, [c, s] = [pose.R[0], pose.R[3]];
        const corner = (sx, sy) => this.px(pose.t[0] + c * sx * half[0] - s * sy * half[1], pose.t[1] + s * sx * half[0] + c * sy * half[1]);
        const pts = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)];
        const box = () => { ctx.beginPath(); pts.forEach((q, k) => (k ? ctx.lineTo(...q) : ctx.moveTo(...q))); ctx.closePath(); };
        box();
        if (fill) { ctx.fillStyle = fill; ctx.fill(); }
        if (anchored) {
            ctx.save(); ctx.clip();
            ctx.strokeStyle = alpha(color, 0.55); ctx.lineWidth = 1;
            const [cx, cy] = this.px(pose.t[0], pose.t[1]), R = 3 * this.scale * Math.max(half[0], half[1]);
            for (let d = -R; d < R; d += 5) { ctx.beginPath(); ctx.moveTo(cx + d - R, cy + R); ctx.lineTo(cx + d + R, cy - R); ctx.stroke(); }
            ctx.restore();
            box();                                       // the path is not part of the saved state
        }
        ctx.setLineDash(dash); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); ctx.setLineDash([]);
    }

    // Text with a halo, at a world point plus a pixel offset. Lines are separated by '\n',
    // and the block is shifted, when needed, so that it stays inside the canvas.
    label(text, at, { color, size = 11.5, weight = 700, dx = 0, dy = 0, align = 'left', baseline = 'middle' } = {}) {
        const { ctx, p } = this, q = this.px(at[0], at[1]), lines = String(text).split('\n'), lh = size * 1.25;
        ctx.font = `${weight} ${size}px ${this.font}`; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
        const width = Math.max(...lines.map((l) => ctx.measureText(l).width)), height = lh * lines.length;
        let x = q[0] + dx - (align === 'right' ? width : align === 'center' ? width / 2 : 0);
        let y = q[1] + dy - (baseline === 'middle' ? height / 2 : baseline === 'bottom' ? height : 0) + lh / 2;
        x = Math.max(4, Math.min(this.w - width - 4, x));
        y = Math.max(lh / 2 + 2, Math.min(this.h - height + lh / 2 - 2, y));
        lines.forEach((l, k) => {
            ctx.lineWidth = 4; ctx.strokeStyle = alpha(p.canvas, 0.92); ctx.strokeText(l, x, y + k * lh);
            ctx.fillStyle = color || p.ink; ctx.fillText(l, x, y + k * lh);
        });
    }

    // A caption in the top-left corner of the canvas.
    caption(text, { color, size = 12.5 } = {}) {
        if (text) this.label(text, this.world(12, 12), { color, size, baseline: 'top' });
    }

    // A tagged leader: a short line from the point to the text.
    tag(text, at, { color, dx = 28, dy = -22, size = 11 } = {}) {
        const { ctx, p } = this, q = this.px(at[0], at[1]);
        ctx.strokeStyle = color || p.inkSoft; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(q[0] + dx * 0.85, q[1] + dy * 0.85); ctx.stroke();
        const w = this.world(q[0] + dx, q[1] + dy);
        this.label(text, w, { color, size, align: dx < 0 ? 'right' : 'left', dx: dx < 0 ? -3 : 3 });
    }
}

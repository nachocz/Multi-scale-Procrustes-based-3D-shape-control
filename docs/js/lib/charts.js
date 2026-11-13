// Charts of the multi-scale analysis on canvases: the map q(t, r) with the trace of
// r*(t) (last row of Fig. 5 of the paper), the matrix ehat(r, r'), and bar charts.
// Color maps: parula-like for q and ehat (negative blue, zero green, positive yellow,
// as in the paper), and sequential maps for errors and distances.
import { alpha, onThemeChange, palette } from './theme.js';

const PARULA = [[53, 42, 135], [15, 92, 221], [18, 125, 216], [7, 156, 207], [21, 177, 180], [89, 189, 140], [165, 190, 107], [225, 185, 82], [249, 251, 14]];
const VIRIDIS = [[68, 1, 84], [72, 40, 120], [62, 74, 137], [49, 104, 142], [38, 130, 142], [31, 158, 137], [53, 183, 121], [109, 205, 89], [180, 222, 44], [253, 231, 37]];
function ramp(stops, t) {
    const x = Math.max(0, Math.min(1, t)) * (stops.length - 1), i = Math.min(stops.length - 2, Math.floor(x)), f = x - i;
    return stops[i].map((v, k) => (v + (stops[i + 1][k] - v) * f) / 255);
}
export const parula = (t) => ramp(PARULA, t);
export const viridis = (t) => ramp(VIRIDIS, t);
// signed value in [-1, 1] to parula (0 -> green)
export const signedColor = (v) => parula(0.5 + 0.5 * v);
const css = ([r, g, b]) => `rgb(${Math.round(r * 255)}, ${Math.round(g * 255)}, ${Math.round(b * 255)})`;
export const cssColor = css;

function setup(canvas) {
    const dpr = Math.min(window.devicePixelRatio || 1, 2), w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return null;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
        canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h, p: palette(), font: getComputedStyle(document.body).fontFamily };
}
const fmtTick = (v) => (Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(0) : v.toFixed(1));

// q(t, r): one column per iteration (values normalized to [-1, 1]), r*(t) in red.
export class ScaleTimeMap {
    constructor(canvas, { xLabel = 't [s]', yLabel = 'r [cm]' } = {}) {
        this.canvas = canvas; this.xLabel = xLabel; this.yLabel = yLabel;
        this.data = { times: [], columns: [], rStar: [], scales: null };
        new ResizeObserver(() => this.draw()).observe(canvas);
        onThemeChange(() => this.draw());
    }
    set(data) { this.data = data; this.draw(); }
    draw() {
        const s = setup(this.canvas);
        if (!s) return;
        const { ctx, w, h, p, font } = s, { times, columns, rStar, scales } = this.data;
        const L = 44, R = w - 10, T = 8, B = h - 28;
        ctx.fillStyle = p.canvas; ctx.fillRect(L, T, R - L, B - T);
        if (scales && columns.length) {
            const n = scales.length, r0 = scales[0], r1 = scales[n - 1], dr = (r1 - r0) / Math.max(1, n - 1);
            const t1 = Math.max(times[times.length - 1], 1e-9), t0 = 0;
            const X = (t) => L + (t - t0) / (t1 - t0) * (R - L), Yp = (r) => B - (r - (r0 - dr / 2)) / (r1 - r0 + dr) * (B - T);
            const cw = Math.max(1, (R - L) / columns.length + 0.6);
            columns.forEach((col, k) => {
                const x = L + k / columns.length * (R - L);
                for (let i = 0; i < n; i++) {
                    if (!Number.isFinite(col[i])) continue;
                    ctx.fillStyle = css(signedColor(col[i]));
                    const y0 = Yp(scales[i] + dr / 2), y1 = Yp(scales[i] - dr / 2);
                    ctx.fillRect(x, y0, cw, y1 - y0 + 0.6);
                }
            });
            ctx.strokeStyle = p.rstar; ctx.lineWidth = 2.2; ctx.lineJoin = 'round';
            ctx.beginPath();
            rStar.forEach((r, k) => { const x = L + (k + 0.5) / columns.length * (R - L), y = Yp(r); if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
            ctx.stroke();
            ctx.fillStyle = p.muted; ctx.font = `10px ${font}`; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
            const step = Math.max(1, Math.round(n / 5));
            for (let i = 0; i < n; i += step) ctx.fillText(fmtTick(scales[i]), L - 5, Yp(scales[i]));
            ctx.textAlign = 'center'; ctx.textBaseline = 'top';
            const tickStep = niceStep(t1 / 5);
            for (let t = 0; t <= t1 + 1e-9; t += tickStep) ctx.fillText(fmtTick(t), X(t), B + 4);
        } else {
            ctx.fillStyle = p.muted; ctx.font = `11px ${font}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
            ctx.fillText('run the controller to fill q(t, r)', (L + R) / 2, (T + B) / 2);
        }
        ctx.strokeStyle = p.lineStrong; ctx.lineWidth = 1; ctx.strokeRect(L, T, R - L, B - T);
        ctx.fillStyle = p.muted; ctx.font = `600 10px ${font}`; ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
        ctx.fillText(this.xLabel, (L + R) / 2, h - 2);
        ctx.save(); ctx.translate(11, (T + B) / 2); ctx.rotate(-Math.PI / 2); ctx.textBaseline = 'middle'; ctx.fillText(this.yLabel, 0, 0); ctx.restore();
    }
}
function niceStep(raw) {
    const mag = 10 ** Math.floor(Math.log10(Math.max(raw, 1e-9))), x = raw / mag;
    return (x < 1.5 ? 1 : x < 3 ? 2 : x < 7 ? 5 : 10) * mag;
}

// Shape error and deformation over time, as in the fourth row of Fig. 2 of the paper:
// e(t) in red on the left axis, and the relative deformation of the object (mean of the
// edge-length change, with a band of one standard deviation) in blue on the right axis.
export class ErrorDeformationPlot {
    constructor(canvas) {
        this.canvas = canvas;
        this.data = null;
        new ResizeObserver(() => this.draw()).observe(canvas);
        onThemeChange(() => this.draw());
    }
    // data: {t, e, eAlt (optional, dashed), mean, std}
    set(data) { this.data = data; this.draw(); }
    draw() {
        const s = setup(this.canvas);
        if (!s || !this.data) return;
        const { ctx, w, h, p, font } = s, { t, e, eAlt, mean, std } = this.data;
        const L = 46, R = w - 46, T = 10, B = h - 28;
        const red = p.dark ? '#ff6b6b' : '#d62728', blue = p.dark ? '#7aa7ff' : '#1f5fd1';
        const t1 = Math.max(t[t.length - 1] || 0, 1e-6);
        let eMax = 0;
        for (const v of [...e, ...(eAlt || [])]) if (Number.isFinite(v)) eMax = Math.max(eMax, v);
        eMax = niceUp(eMax || 1);
        let dMax = 1;
        mean.forEach((m, i) => { dMax = Math.max(dMax, Math.abs(m) + std[i]); });
        dMax = niceUp(dMax);
        const X = (v) => L + v / t1 * (R - L), Ye = (v) => B - v / eMax * (B - T), Yd = (v) => (T + B) / 2 - v / dMax * (B - T) / 2;
        ctx.strokeStyle = p.grid; ctx.lineWidth = 1;
        for (let k = 0; k <= 4; k++) { const y = T + k * (B - T) / 4; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(R, y); ctx.stroke(); }
        // deformation band and mean
        if (t.length > 1) {
            ctx.fillStyle = alpha(blue, 0.14); ctx.beginPath();
            t.forEach((v, i) => { const x = X(v), y = Yd(mean[i] + std[i]); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
            for (let i = t.length - 1; i >= 0; i--) ctx.lineTo(X(t[i]), Yd(mean[i] - std[i]));
            ctx.closePath(); ctx.fill();
            const line = (ys, Y, color, width, dash = []) => {
                ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.beginPath();
                t.forEach((v, i) => { if (!Number.isFinite(ys[i])) return; const x = X(v), y = Y(ys[i]); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
                ctx.stroke(); ctx.setLineDash([]);
            };
            line(mean, Yd, blue, 1.3);
            if (eAlt) line(eAlt, Ye, red, 1.3, [5, 4]);
            line(e, Ye, red, 2);
        }
        ctx.strokeStyle = p.lineStrong; ctx.lineWidth = 1; ctx.strokeRect(L, T, R - L, B - T);
        ctx.font = `10px ${font}`; ctx.textBaseline = 'middle';
        for (let k = 0; k <= 4; k++) {
            const y = T + k * (B - T) / 4;
            ctx.fillStyle = red; ctx.textAlign = 'right'; ctx.fillText(fmtTick(eMax * (1 - k / 4)), L - 5, y);
            ctx.fillStyle = blue; ctx.textAlign = 'left'; ctx.fillText(fmtTick(dMax * (1 - k / 2)), R + 5, y);
        }
        ctx.fillStyle = p.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        const step = niceStep(t1 / 5);
        for (let v = 0; v <= t1 + 1e-9; v += step) ctx.fillText(fmtTick(v), X(v), B + 4);
        ctx.font = `600 10px ${font}`; ctx.textBaseline = 'alphabetic'; ctx.fillText('t [s]', (L + R) / 2, h - 2);
        ctx.save(); ctx.translate(11, (T + B) / 2); ctx.rotate(-Math.PI / 2); ctx.textBaseline = 'middle'; ctx.fillStyle = red; ctx.fillText('shape error e(t) [cm]', 0, 0); ctx.restore();
        ctx.save(); ctx.translate(w - 9, (T + B) / 2); ctx.rotate(Math.PI / 2); ctx.textBaseline = 'middle'; ctx.fillStyle = blue; ctx.fillText('object deformation [%]', 0, 0); ctx.restore();
    }
}
function niceUp(v) { const st = niceStep(v / 4); return Math.ceil(v / st - 1e-9) * st; }

// ehat(r, r'): rows r (action scale, vertical), columns r' (influence scale, horizontal).
export class MatrixMap {
    constructor(canvas, { onPick = null, xLabel = "r' [cm]", yLabel = 'r [cm]' } = {}) {
        this.canvas = canvas; this.onPick = onPick; this.xLabel = xLabel; this.yLabel = yLabel;
        this.data = null;
        new ResizeObserver(() => this.draw()).observe(canvas);
        onThemeChange(() => this.draw());
        if (onPick) {
            canvas.style.cursor = 'crosshair';
            const pick = (event) => {
                if (!this.frame || !this.data) return;
                const r = canvas.getBoundingClientRect(), { L, R, T, B } = this.frame, n = this.data.scales.length;
                const x = event.clientX - r.left, y = event.clientY - r.top;
                if (x < L || x > R || y < T || y > B) return;
                const s2 = Math.min(n - 1, Math.floor((x - L) / (R - L) * n)), s = Math.min(n - 1, Math.floor((B - y) / (B - T) * n));
                this.onPick(s, s2);
            };
            canvas.addEventListener('pointerdown', (e) => { canvas.setPointerCapture(e.pointerId); pick(e); });
            canvas.addEventListener('pointermove', (e) => { if (e.buttons) pick(e); });
        }
    }
    // data: {values (n*n, row-major [s * n + s2]), scales, sel: [s, s2] | null, row: s | null}
    set(data) { this.data = data; this.draw(); }
    draw() {
        const s = setup(this.canvas);
        if (!s) return;
        const { ctx, w, h, p, font } = s;
        const L = 40, R = w - 8, T = 8, B = h - 28;
        this.frame = { L, R, T, B };
        if (!this.data) return;
        const { values, scales, sel, row } = this.data, n = scales.length;
        let m = 0;
        for (const v of values) if (Number.isFinite(v)) m = Math.max(m, Math.abs(v));
        m = m || 1;
        const cw = (R - L) / n, ch = (B - T) / n;
        for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
            ctx.fillStyle = css(signedColor(values[i * n + j] / m));
            ctx.fillRect(L + j * cw, B - (i + 1) * ch, cw + 0.5, ch + 0.5);
        }
        if (row !== null && row !== undefined) {
            ctx.strokeStyle = p.rstar; ctx.lineWidth = 2;
            const y = B - (row + 1) * ch;
            ctx.strokeRect(L + 1, y, R - L - 2, ch);
        }
        if (sel) {
            ctx.strokeStyle = p.dark ? '#ffffff' : '#111111'; ctx.lineWidth = 2;
            ctx.strokeRect(L + sel[1] * cw, B - (sel[0] + 1) * ch, cw, ch);
        }
        ctx.strokeStyle = p.lineStrong; ctx.lineWidth = 1; ctx.strokeRect(L, T, R - L, B - T);
        ctx.fillStyle = p.muted; ctx.font = `10px ${font}`;
        const step = Math.max(1, Math.round(n / 4));
        ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
        for (let i = 0; i < n; i += step) ctx.fillText(fmtTick(scales[i]), L - 4, B - (i + 0.5) * ch);
        ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        for (let j = 0; j < n; j += step) ctx.fillText(fmtTick(scales[j]), L + (j + 0.5) * cw, B + 4);
        ctx.font = `600 10px ${font}`; ctx.textBaseline = 'alphabetic';
        ctx.fillText(this.xLabel, (L + R) / 2, h - 2);
        ctx.save(); ctx.translate(10, (T + B) / 2); ctx.rotate(-Math.PI / 2); ctx.textBaseline = 'middle'; ctx.fillText(this.yLabel, 0, 0); ctx.restore();
    }
}

// Bars over the scales, with an optional marker (e.g. r*) and an optional overlaid curve.
export class ScaleBars {
    constructor(canvas, { yLabel = '', xLabel = 'r [cm]', signed = false, yRange = null } = {}) {
        this.canvas = canvas; this.yLabel = yLabel; this.xLabel = xLabel; this.signed = signed; this.yRange = yRange;
        this.data = null;
        new ResizeObserver(() => this.draw()).observe(canvas);
        onThemeChange(() => this.draw());
    }
    // data: {scales, values, color | colors(fn), marker: r | null, markerLabel, curve: {values, color} | null}
    set(data) { this.data = data; this.draw(); }
    draw() {
        const s = setup(this.canvas);
        if (!s || !this.data) return;
        const { ctx, w, h, p, font } = s, { scales, values, marker, markerLabel, curve } = this.data, n = scales.length;
        const L = 40, R = w - 8, T = 10, B = h - 26;
        let lo = this.signed ? Math.min(0, ...values) : 0, hi = Math.max(1e-12, ...values);
        if (curve) { lo = Math.min(lo, ...curve.values); hi = Math.max(hi, ...curve.values); }
        if (this.yRange) [lo, hi] = this.yRange;
        if (this.signed) { const m = Math.max(Math.abs(lo), Math.abs(hi)) || 1; lo = -m; hi = m; }
        const Y = (v) => B - (v - lo) / (hi - lo || 1) * (B - T), bw = (R - L) / n;
        ctx.strokeStyle = p.grid; ctx.lineWidth = 1;
        for (const v of [lo, (lo + hi) / 2, hi]) { ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(R, Y(v)); ctx.stroke(); }
        values.forEach((v, i) => {
            ctx.fillStyle = this.data.colors ? this.data.colors(v, i) : this.data.color;
            const y0 = Y(Math.max(0, lo)), y1 = Y(v);
            ctx.fillRect(L + i * bw + 1, Math.min(y0, y1), Math.max(1, bw - 2), Math.abs(y1 - y0) || 1);
        });
        if (curve) {
            ctx.strokeStyle = curve.color; ctx.lineWidth = 1.8; ctx.beginPath();
            curve.values.forEach((v, i) => { const x = L + (i + 0.5) * bw; if (i) ctx.lineTo(x, Y(v)); else ctx.moveTo(x, Y(v)); });
            ctx.stroke();
        }
        if (marker !== null && marker !== undefined) {
            const r0 = scales[0], r1 = scales[n - 1], x = L + ((marker - r0) / (r1 - r0 || 1) * (n - 1) + 0.5) * bw;
            ctx.strokeStyle = p.rstar; ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
            ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, B); ctx.stroke(); ctx.setLineDash([]);
            if (markerLabel) { ctx.fillStyle = p.rstar; ctx.font = `700 10px ${font}`; ctx.textAlign = x > (L + R) / 2 ? 'right' : 'left'; ctx.textBaseline = 'top'; ctx.fillText(markerLabel, x + (x > (L + R) / 2 ? -4 : 4), T); }
        }
        ctx.strokeStyle = p.lineStrong; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(L, T); ctx.lineTo(L, B); ctx.lineTo(R, B); ctx.stroke();
        if (this.signed && lo < 0) { ctx.strokeStyle = alpha(p.ink, 0.35); ctx.beginPath(); ctx.moveTo(L, Y(0)); ctx.lineTo(R, Y(0)); ctx.stroke(); }
        ctx.fillStyle = p.muted; ctx.font = `10px ${font}`; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
        for (const v of [lo, hi]) ctx.fillText(Math.abs(v) < 1e-3 && v !== 0 ? v.toExponential(0) : Number(v.toPrecision(2)).toString(), L - 4, Y(v));
        ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        const step = Math.max(1, Math.round(n / 5));
        for (let i = 0; i < n; i += step) ctx.fillText(fmtTick(scales[i]), L + (i + 0.5) * bw, B + 4);
        ctx.font = `600 10px ${font}`; ctx.textBaseline = 'alphabetic'; ctx.fillText(this.xLabel, (L + R) / 2, h - 1);
        if (this.yLabel) { ctx.save(); ctx.translate(10, (T + B) / 2); ctx.rotate(-Math.PI / 2); ctx.textBaseline = 'middle'; ctx.fillText(this.yLabel, 0, 0); ctx.restore(); }
    }
}

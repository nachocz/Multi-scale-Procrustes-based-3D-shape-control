// Multi-scale analysis (Secs. II-A to II-D) at one instant of the closed loop: geodesic
// balls, Procrustes transforms of the balls, the LRB estimate Xhat(t, r, r'), the error
// increments ehat(t, r, r'), the validation w(t, r'), and q(t, r).
import { MatrixMap, ScaleBars, cssColor, signedColor } from '../lib/charts.js';
import { LinePlot } from '../lib/plot.js';
import { ballFlags, ghostPoses, gripperColor, isolineAnchor, nodeColors, screwPath } from '../lib/present.js';
import { initPage } from '../lib/shell.js';
import { onThemeChange, palette } from '../lib/theme.js';
import { button, card, control, fmt, h, label, legend, readouts, segmented, select, slider, toggle } from '../lib/ui.js';
import { ObjectView } from '../lib/view3d.js';
import { isoline } from '../sim/geodesic.js';
import { applyAll, rotationAngle } from '../sim/se3.js';
import { Session } from '../sim/session.js';
import { SCENARIOS, World } from '../sim/world.js';

await initPage();

const K = 120;
const params = new URLSearchParams(location.search);
const settings = {
    scenario: SCENARIOS[params.get('scenario')] ? params.get('scenario') : 'sbend',
    k: 0, s: null, s2: null, lambda: 1, beta: 1e4, colors: 'scales', target: true, estimate: true, poses: true,
};
let world = null, session = null, frames = [], computing = null;
const cache = {};

// ---------------------------------------------------------------- layout
const workspace = document.getElementById('workspace');
const topCard = card('', '');
const scenarioPick = select({
    options: Object.entries(SCENARIOS).map(([value, s]) => ({ value, label: s.title })),
    value: settings.scenario, onChange: (v) => { settings.scenario = v; load(); },
});
scenarioPick.el.style.width = 'auto';
const kSlider = slider({ label: 'instant $k$ of the closed loop (page 3)', min: 0, max: K, step: 1, value: 0, format: (v) => `k = ${v.toFixed(0)}`,
    onInput: (v) => { settings.k = Math.round(v); ensure(settings.k); } });
const progress = h('span', { class: 'hint' });
topCard.body.append(h('div', { class: 'toolbar' }, h('b', { style: 'font-size:0.8rem' }, 'Scenario'), scenarioPick.el, progress),
    h('div', { style: 'margin-top:10px' }, kSlider.el));

const viewCard = card('Balls, transforms, and the LRB estimate', 'drag to orbit, scroll to zoom');
const view = h('div', { class: 'view3d dash-fit' });
const busy = h('div', { class: 'busy' }, 'Preparing the scenario…');
const viewTools = h('div', { class: 'view-toolbar' });
view.append(viewTools, busy);
const legendEl = h('div');
viewCard.body.append(view, legendEl);
const scene = new ObjectView(view);
viewTools.append(button('Reset view', () => scene.frameView(world.scenario.view, [world.initial.P, world.target.P]), { small: true }));
function drawLegend() {
    const p = palette();
    legendEl.replaceChildren(legend([
        { label: "balls $\\Omega_g(t,r')$ (tinted)", color: p.g1, kind: 'band' }, { label: 'isolines $D_g=r$', color: p.g2 },
        { label: "isolines $D_g=r'$", color: p.dark ? '#ffffff' : '#20242a' }, { label: 'target $\\mathbf Y$', color: p.target, kind: 'band' },
        { label: "estimate $\\hat{\\mathbf X}(t,r,r')$", color: p.estimate, kind: 'band' }, { label: 'poses after $\\mathbf T_g(t,r)$', color: p.muted, kind: 'dashed' },
    ]));
}

const chartCard = card('Analysis at the selected instant', '');
const eCanvas = h('canvas', { class: 'chart-canvas square' }), qCanvas = h('canvas', { class: 'chart-canvas' }), wCanvas = h('canvas', { class: 'chart-canvas' });
const eMap = new MatrixMap(eCanvas, { onPick: (s, s2) => { settings.s = s; settings.s2 = s2; syncScaleSliders(); render(); } });
const qBars = new ScaleBars(qCanvas, { yLabel: 'q / max|q|', signed: true });
const wBars = new ScaleBars(wCanvas, { yLabel: 'w / max w', xLabel: "r' [cm]", yRange: [0, 1] });
chartCard.body.append(
    h('p', { class: 'chart-title' }, label("$\\hat e(t,r,r')$ (10) [cm]: click to select $r$ and $r'$")), h('div', { class: 'matrix-small' }, eCanvas),
    h('div', { class: 'colorbar signed' }, h('span', {}, 'negative (error decreases)'), h('i'), h('span', {}, 'positive')),
    h('div', { class: 'side-pair', style: 'margin-top:12px' },
        h('div', {}, h('p', { class: 'chart-title' }, label('$q(t,r)$ (11) for the selected $\\beta$')), qCanvas),
        h('div', {}, h('p', { class: 'chart-title' }, label("$w(t,r')$ (8)")), wCanvas)));
const ballCard = card('Procrustes analysis of the balls', 'per gripper');
ballCard.body.append(h('div', { class: 'side-pair' },
    h('div', {}, h('p', { class: 'chart-title' }, label('residual, $d_{\\mathcal P}/\\sqrt{M_g}$ [mm]')), h('canvas', { class: 'chart-canvas', id: 'resCanvas' })),
    h('div', {}, h('p', { class: 'chart-title' }, label('rotation of $\\mathbf T_g(t,r)$ [°]')), h('canvas', { class: 'chart-canvas', id: 'rotCanvas' }))));
const resPlot = new LinePlot(ballCard.body.querySelector('#resCanvas'), { xLabel: 'r [cm]', yIncludeZero: true });
const rotPlot = new LinePlot(ballCard.body.querySelector('#rotCanvas'), { xLabel: 'r [cm]', yIncludeZero: true });

// rail
const readCard = card('Values', '');
const values = readouts([
    { key: 'e', label: '$e(t)$ [cm]' }, { key: 'eh', label: "$\\hat e(t,r,r')$ [cm]", title: 'estimated error increment for one step dt = 0.1' },
    { key: 'q', label: '$q(t,r)/\\max|q|$' }, { key: 'rs', label: '$r^*(t)$ [cm]' },
    { key: 'm1', label: "$M_1(r')$" }, { key: 'm2', label: "$M_2(r')$" },
]);
readCard.body.append(values.el);
const scaleCard = card('Scales', '');
const sSlider = slider({ label: 'action scale $r$', min: 0, max: 19, step: 1, value: 0, format: (v) => '–', onInput: (v) => { settings.s = Math.round(v); render(); } });
const s2Slider = slider({ label: "influence scale $r'$", min: 0, max: 19, step: 1, value: 0, format: (v) => '–', onInput: (v) => { settings.s2 = Math.round(v); render(); } });
const linkToggle = toggle({ label: "$r'=r$", value: true, onChange: () => { if (linkToggle.get()) settings.s2 = settings.s; syncScaleSliders(); render(); } });
const lambdaSlider = slider({ label: 'step $\\lambda$ of the estimate', description: '$\\lambda=1$: each ball moves by the whole transform; the controller uses $\\Delta t=0.1$',
    min: 0.05, max: 1, step: 0.05, value: 1, onInput: (v) => { settings.lambda = v; render(); } });
const rStarButton = button('Select $r=r\'=r^*(t)$', () => { const f = frames[settings.k]; if (!f) return; settings.s = settings.s2 = Math.round(f.out.sStar); syncScaleSliders(); render(); }, { small: true });
scaleCard.body.append(sSlider.el, linkToggle.el, s2Slider.el, lambdaSlider.el, h('div', { class: 'toolbar' }, rStarButton));
const wCard = card('Relaxation', 'at the selected instant');
const betaSlider = slider({ label: 'relaxation $\\beta$ (8)', min: 0.01, max: 1e6, log: true, value: settings.beta, format: (v) => (v >= 100 ? v.toExponential(0) : v.toPrecision(2)),
    onInput: (v) => { settings.beta = v; render(); } });
wCard.body.append(betaSlider.el, h('p', { class: 'hint' }, label('At $k=0$, $w=1$: no action has been validated yet.')));
const layerCard = card('Layers', '');
const colorPick = segmented({
    options: [{ value: 'scales', label: "balls $r'$" }, { value: 'match', label: 'surface map' }, { value: 'error', label: 'error' }, { value: 'distance', label: '$D_1$' }],
    value: settings.colors, onChange: (v) => { settings.colors = v; render(); },
});
layerCard.body.append(control('color of the object', colorPick.el),
    toggle({ label: 'target $\\mathbf Y$', value: true, onChange: (v) => { settings.target = v; render(); } }).el,
    toggle({ label: "estimate $\\hat{\\mathbf X}(t,r,r')$", value: true, onChange: (v) => { settings.estimate = v; render(); } }).el,
    toggle({ label: 'Procrustes poses and $\\mathrm{SE}(3)$ geodesics', value: true, onChange: (v) => { settings.poses = v; render(); } }).el);
// the view stays in place; the controls and the plots are beside it
workspace.append(topCard.el, h('div', { class: 'dash' },
    h('div', { class: 'dash-view' }, viewCard.el),
    h('div', { class: 'dash-side' }, scaleCard.el, readCard.el, chartCard.el, ballCard.el, wCard.el, layerCard.el)));

function syncScaleSliders() {
    const f = frames[settings.k];
    if (!f) return;
    const sc = f.out.scales, n = sc.length;
    if (linkToggle.get()) settings.s2 = settings.s;
    for (const [sl, s] of [[sSlider, settings.s], [s2Slider, settings.s2]]) {
        sl.input.max = n - 1;
        sl.set(s);
        sl.el.querySelector('output').textContent = `${sc[s].toFixed(1)} cm`;
    }
    s2Slider.disable(linkToggle.get());
}

// ---------------------------------------------------------------- the closed loop up to k
function load() {
    busy.hidden = false;
    progress.textContent = '';
    setTimeout(() => {
        world = cache[settings.scenario] || (cache[settings.scenario] = new World(settings.scenario));
        scene.setWorld(world);
        session = new Session(world, { noise: 0.5 });
        frames = [snapshot()];
        settings.s = settings.s2 = Math.round(frames[0].out.sStar);
        syncScaleSliders();
        busy.hidden = true;
        ensure(settings.k);
    }, 30);
}

function snapshot() {
    return { P: world.P.slice(), poses: world.poses.map((T) => ({ R: T.R.slice(), t: T.t.slice() })), X: session.X, out: session.out, t: session.t };
}

// Run the loop (in slices, without blocking the page) until frame k exists, then draw it.
function ensure(k) {
    if (frames[k]) { if (!computing) render(); return; }
    if (computing) return;
    computing = true;
    const slice = () => {
        const start = performance.now();
        while (frames.length <= Math.max(k, settings.k) && performance.now() - start < 40) { session.step(); frames.push(snapshot()); }
        progress.textContent = frames.length <= Math.max(k, settings.k) ? `running the closed loop: k = ${frames.length - 1}` : '';
        if (frames.length <= Math.max(k, settings.k)) setTimeout(slice, 0);
        else { computing = null; render(); }
    };
    slice();
}

// ---------------------------------------------------------------- view
function render() {
    const f = frames[settings.k];
    if (!f || !world) return;
    const p = palette(), { out, X } = f, ctrl = session.ctrl, n = out.nS, M = world.M;
    const s = Math.min(settings.s, n - 1), s2 = Math.min(settings.s2, n - 1), sc = out.scales;
    // w and q for the selected beta (stored residuals of the validation)
    const minDelta = Math.min(...out.delta);
    const w = Array.from(out.delta, (d) => Math.exp(-settings.beta * (d - minDelta)));
    const q = Array.from({ length: n }, (_, i) => w.reduce((acc, wi, j) => acc + out.ehat[i * n + j] * wi, 0));
    const qMax = Math.max(...q.map(Math.abs)) || 1;
    // object at instant k
    scene.setObject(f.P);
    const inBall = out.balls.map((b, g) => ballFlags(b, out.counts[s2][g], M));
    const top = sliceTop(f.P), labels = [];
    scene.setColors(nodeColors(world, settings.colors, { inBall, X: top, Y: ctrl.Y, D: out.D[0], band: sc[1] - sc[0] }));
    scene.setTarget(settings.target ? applyAll(ctrl.alignment, world.target.P) : null);
    out.D.forEach((D, g) => {
        const a = s < n - 1 ? isoline(top, world.surface.tris, D, sc[s]) : null;
        const b = s2 !== s && s2 < n - 1 ? isoline(top, world.surface.tris, D, sc[s2]) : null;
        scene.setIsoline(g, a, 'primary');
        scene.setIsoline(g, b, 'secondary');
        const anchor = a && a.length ? isolineAnchor(a, f.poses[g].t) : null;
        if (anchor) labels.push({ p: anchor, text: `r = ${sc[s].toFixed(1)} cm`, color: gripperColor(g), cls: 'small', dy: -8 - 14 * g });
        const anchor2 = b && b.length ? isolineAnchor(b, f.poses[g].t) : null;
        if (anchor2) labels.push({ p: anchor2, text: `r' = ${sc[s2].toFixed(1)} cm`, cls: 'small', dy: 6 + 14 * g });
    });
    const T = out.T[s];
    scene.setGrippers(f.poses, settings.poses ? ghostPoses(f.poses, T) : null, settings.poses ? T.map((Tg, g) => screwPath(Tg, f.poses[g].t)) : null);
    f.poses.forEach((P, g) => labels.push({ p: P.t, text: `g${g + 1}`, color: gripperColor(g), dx: 10, dy: -22 }));
    scene.setLabels(labels);
    scene.setSheet('estimate', settings.estimate ? ctrl.estimate(X, out.balls, out.counts[s], out.counts[s2], settings.lambda).Xhat : null);
    scene.setSheet('response', null);

    // readouts and charts
    values.set('e', fmt(out.e, 2));
    values.set('eh', fmt(out.ehat[s * n + s2], 3), out.ehat[s * n + s2] < 0 ? 'good' : 'bad');
    values.set('q', fmt(q[s] / qMax, 2), q[s] < 0 ? 'good' : 'bad');
    values.set('rs', fmt(out.rStar, 2));
    values.set('m1', String(out.counts[s2][0]));
    values.set('m2', String(out.counts[s2][1] ?? '–'));
    eMap.set({ values: out.ehat, scales: sc, sel: [s, s2], row: Math.round(out.sStar) });
    qBars.set({ scales: sc, values: q.map((v) => v / qMax), colors: (v) => cssColor(signedColor(v)), marker: sc[s], markerLabel: 'r' });
    wBars.set({ scales: sc, values: w, color: p.accent, marker: sc[s2], markerLabel: "r'" });
    const xs = Array.from(sc);
    resPlot.set({
        series: out.T[0].map((_, g) => ({ x: xs, y: out.T.map((row) => 10 * row[g].d / Math.sqrt(row[g].n)), color: gripperColor(g), width: 1.8 })),
        vlines: [{ x: sc[s], color: p.rstar }],
    });
    rotPlot.set({
        series: out.T[0].map((_, g) => ({ x: xs, y: out.T.map((row) => rotationAngle(row[g].R) * 180 / Math.PI), color: gripperColor(g), width: 1.8 })),
        vlines: [{ x: sc[s], color: p.rstar }],
    });
    syncScaleSliders();
}

// Top-layer positions (the perceived surface without noise) of lattice positions P.
function sliceTop(P) {
    const { top } = world.strip, out = new Float64Array(3 * top.length);
    for (let m = 0; m < top.length; m++) { out[3 * m] = P[3 * top[m]]; out[3 * m + 1] = P[3 * top[m] + 1]; out[3 * m + 2] = P[3 * top[m] + 2]; }
    return out;
}

onThemeChange(() => { scene.refreshTheme(); drawLegend(); render(); });
drawLegend();
load();

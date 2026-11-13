// Stability (Sec. III): the terms of Lemmas 2 and 3 and the Lyapunov function along the
// closed loop, the functions theta / (2 sin theta) and c(theta), and an unreachable target
// for the remark on under-actuation.
import { LinePlot, niceRange } from '../lib/plot.js';
import { ballFlags, gripperColor, isolineAnchor, nodeColors } from '../lib/present.js';
import { initPage } from '../lib/shell.js';
import { onThemeChange, palette } from '../lib/theme.js';
import { button, card, fmt, h, label, readouts, segmented, select, seriesLegend, toggle } from '../lib/ui.js';
import { ObjectView } from '../lib/view3d.js';
import { isoline } from '../sim/geodesic.js';
import { applyAll } from '../sim/se3.js';
import { Session } from '../sim/session.js';
import { cFactor, lyapunov, lyapunovTerms, rotationFactor } from '../sim/stability.js';
import { SCENARIOS, World } from '../sim/world.js';

await initPage();

const K_MAX = 500, BUMP = 2.0;
const settings = { scenario: 'bend', unreachable: false };
let world = null, session = null, history = null, terms = null, running = false, speed = 1, lastWall = 0, budget = 0, frame = 0, bumped = null;
const cache = {};

// ---------------------------------------------------------------- layout
const workspace = document.getElementById('workspace');
const runCard = card('', '');
const scenarioPick = select({
    options: Object.entries(SCENARIOS).map(([value, s]) => ({ value, label: s.title })),
    value: settings.scenario, onChange: (v) => { settings.scenario = v; load(); },
});
scenarioPick.el.style.width = 'auto';
const runButton = button('▶ Run', () => setRunning(!running), { primary: true });
const restartButton = button('↺ Restart', () => { setRunning(false); restart(); });
const speedPick = segmented({ options: [1, 4, 16].map((v) => ({ value: v, label: `${v}×` })), value: 1, onChange: (v) => { speed = v; } });
const unreachable = toggle({ label: 'unreachable target (2 cm bump)', value: false, onChange: (v) => { settings.unreachable = v; setRunning(false); restart(); } });
const clockEl = h('span', { class: 'clock' }, 't = 0.0 s');
runCard.body.append(h('div', { class: 'toolbar' }, h('b', { style: 'font-size:0.8rem' }, 'Scenario'), scenarioPick.el, runButton, restartButton,
    h('span', { class: 'toolbar-divider' }), speedPick.el, clockEl), h('div', { style: 'margin-top:10px' }, unreachable.el));

const viewCard = card('Object, balls at $r^*$, and target', 'no measurement noise on this page');
const view = h('div', { class: 'view3d dash-fit' });
const busy = h('div', { class: 'busy' }, 'Preparing the scenario…');
view.append(busy);
viewCard.body.append(view);
const scene = new ObjectView(view);

const plotsGrid = h('div', { class: 'dash-side' });
const mk = (title, note, opts, legendItems) => {
    const c = card(title, note), canvas = h('canvas', { class: 'plot-canvas' }), lg = h('div');
    c.body.append(canvas, lg);
    plotsGrid.append(c.el);
    return { plot: new LinePlot(canvas, opts), legend: lg, items: legendItems };
};
const vPlot = mk('Lyapunov function $\\mathcal V=\\tfrac12\\|\\mathbf X-\\mathbf Y\\|_F^2$ [cm²]', '', { xLabel: 't [s]', logY: true, yMin: 1e-6 });
const ratePlot = mk('Rate of $\\mathcal V$ [cm²/s]', 'range after the first seconds', { xLabel: 't [s]' });
const termPlot = mk('Magnitude of the terms of (25) [cm²/s]', 'both terms are $\\le 0$', { xLabel: 't [s]', logY: true, yMin: 1e-8 });
const funPlot = mk('$\\theta/(2\\sin\\theta)$ (16) and $c(\\theta)$ (21)', 'marks: angles $\\theta_g$ of $\\mathbf T_g(t,r^*)$', { xLabel: 'θ [rad]', x: [-3.0, 3.0], y: [-2, 3], yIncludeZero: true });
// the view stays in place; the readouts and the plots are beside it

const readCard = card('Now', '');
const values = readouts([
    { key: 'V', label: '$\\mathcal V$ [cm²]' }, { key: 'meas', label: 'measured rate', title: '(V(t) - V(t - dt)) / dt' },
    { key: 'lrb', label: '$\\sum_g\\dot{\\mathcal V}_g$', title: 'sum of the terms of (25), the rate under the LRB motion of the balls' },
    { key: 'th1', label: '$\\theta_1$ [°]' }, { key: 'th2', label: '$\\theta_2$ [°]' }, { key: 'max', label: 'largest term so far', title: 'the lemmas state that every term is non-positive' },
]);
const table = h('table', { class: 'value-table' });
readCard.body.append(values.el, h('p', { class: 'group-label', style: 'margin-top:12px' }, 'Terms of (25) [cm²/s]'), table);
plotsGrid.prepend(readCard.el);
workspace.append(runCard.el, h('div', { class: 'dash' }, h('div', { class: 'dash-view' }, viewCard.el), plotsGrid));

// ---------------------------------------------------------------- simulation
function load() {
    setRunning(false);
    busy.hidden = false;
    setTimeout(() => {
        world = cache[settings.scenario] || (cache[settings.scenario] = new World(settings.scenario));
        scene.setWorld(world);
        restart();
        busy.hidden = true;
    }, 30);
}

function restart() {
    bumped = settings.unreachable ? world.bumpedTarget(BUMP) : null;
    session = new Session(world, { noise: 0, target: bumped ? bumped.Y : null });
    history = { t: [], V: [], meas: [], lrb: [], rot: world.grippers.map(() => []), tr: world.grippers.map(() => []), max: -Infinity };
    record();
    render(true);
}

function record() {
    const { out, ctrl } = session;
    terms = lyapunovTerms(session.X, ctrl.Y, out.balls, out.ballCounts, out.Tstar);
    const V = lyapunov(world.X(), ctrl.Y), n = history.t.length;
    history.t.push(session.t);
    history.V.push(V);
    history.meas.push(n ? (V - history.V[n - 1]) / (session.t - history.t[n - 1]) : NaN);
    history.lrb.push(terms.reduce((s, x) => s + x.total, 0));
    terms.forEach((x, g) => { history.rot[g].push(-x.rotation); history.tr[g].push(-x.translation); history.max = Math.max(history.max, x.rotation, x.translation); });
}

function setRunning(flag) {
    running = flag && history && history.t.length < K_MAX;
    runButton.replaceChildren(label(running ? '❚❚ Pause' : '▶ Run'));
    if (running) { lastWall = performance.now(); budget = 0; requestAnimationFrame(loop); }
}

function loop(now) {
    if (!running) return;
    budget += Math.min((now - lastWall) / 1000, 0.1) * 10 * speed;
    lastWall = now;
    const start = performance.now();
    while (budget >= 1 && performance.now() - start < 28 && history.t.length < K_MAX) { session.step(); record(); budget -= 1; }
    budget = Math.min(budget, 2);
    render(++frame % 3 === 0);
    if (history.t.length >= K_MAX) setRunning(false);
    else requestAnimationFrame(loop);
}

// ---------------------------------------------------------------- views
function render(full = false) {
    if (!session) return;
    const p = palette(), { out, ctrl } = session, X = world.X(), M = world.M, k = history.t.length - 1;
    scene.setObject(world.P);
    scene.setColors(nodeColors(world, 'scales', { inBall: out.balls.map((b, g) => ballFlags(b, out.ballCounts[g], M)) }));
    scene.setTarget(applyAll(ctrl.alignment, bumped ? bumped.P : world.target.P));
    const labels = [];
    out.D.forEach((D, g) => {
        const segs = out.sStar < out.nS - 1 - 1e-9 ? isoline(X, world.surface.tris, D, out.rStar) : null;
        scene.setIsoline(g, segs);
        const a = segs && segs.length ? isolineAnchor(segs, world.poses[g].t) : null;
        if (a) labels.push({ p: a, text: `r* = ${out.rStar.toFixed(1)} cm`, color: gripperColor(g), cls: 'small', dy: -8 - 14 * g });
    });
    scene.setGrippers(world.poses);
    world.poses.forEach((T, g) => labels.push({ p: T.t, text: `g${g + 1}`, color: gripperColor(g), dx: 10, dy: -22 }));
    scene.setLabels(labels);

    clockEl.textContent = `t = ${history.t[k].toFixed(1)} s`;
    values.set('V', fmt(history.V[k], history.V[k] < 1 ? 4 : 2));
    values.set('meas', Number.isFinite(history.meas[k]) ? history.meas[k].toExponential(1) : '–', history.meas[k] > 0 ? 'bad' : '');
    values.set('lrb', history.lrb[k].toExponential(1), history.lrb[k] > 0 ? 'bad' : 'good');
    values.set('th1', fmt(terms[0].theta * 180 / Math.PI, 2));
    values.set('th2', fmt((terms[1]?.theta ?? NaN) * 180 / Math.PI, 2));
    values.set('max', history.max.toExponential(1), history.max > 1e-9 ? 'bad' : 'good');
    table.replaceChildren(h('tr', {}, h('th', {}, ''), h('th', {}, 'rotation (L. 2)'), h('th', {}, 'translation (L. 3)'), h('th', {}, label('$M_g$'))),
        ...terms.map((x, g) => h('tr', {}, h('td', { class: `gripper-tag g${g + 1}` }, `g${g + 1}`), h('td', {}, x.rotation.toExponential(2)), h('td', {}, x.translation.toExponential(2)), h('td', {}, String(x.Mg)))));
    if (!full && running) return;
    const t = history.t;
    vPlot.plot.set({ series: [{ x: t, y: history.V, color: p.ink, width: 1.8 }] });
    // the range of the rates after the initial transient, so that their sign stays readable
    const from = Math.max(5, Math.floor(0.12 * t.length));
    const tail = [...history.meas.slice(from), ...history.lrb.slice(from)].filter(Number.isFinite);
    ratePlot.plot.opts.y = tail.length > 4 ? niceRange(Math.min(0, ...tail), Math.max(0, ...tail)) : null;
    ratePlot.plot.set({ series: [{ x: t, y: history.meas, color: p.ink, width: 1.6 }, { x: t, y: history.lrb, color: p.accent, width: 1.6, dash: [5, 4] }] });
    ratePlot.legend.replaceChildren(seriesLegend([{ label: 'measured', color: p.ink }, { label: '$\\sum_g\\dot{\\mathcal V}_g$ (LRB motion)', color: p.accent, dash: 'dashed' }]));
    termPlot.plot.set({ series: terms.flatMap((_, g) => [
        { x: t, y: history.rot[g], color: gripperColor(g), width: 1.6 },
        { x: t, y: history.tr[g], color: gripperColor(g), width: 1.6, dash: [5, 4] }]) });
    termPlot.legend.replaceChildren(seriesLegend([{ label: 'rotation, $-$(Lemma 2)', color: p.muted }, { label: 'translation, $-$(Lemma 3)', color: p.muted, dash: 'dashed' },
        { label: 'g1', color: gripperColor(0) }, { label: 'g2', color: gripperColor(1) }]));
    const th = Array.from({ length: 241 }, (_, i) => -3.0 + 6.0 * i / 240);
    funPlot.plot.set({
        series: [{ x: th, y: th.map(rotationFactor), color: p.accent, width: 1.8 }, { x: th, y: th.map(cFactor), color: p.warn, width: 1.8 }],
        hlines: [{ y: 1, color: p.muted, dash: [4, 4], label: 'c = 1' }],
        markers: terms.flatMap((x, g) => [{ x: x.theta, y: rotationFactor(x.theta), color: gripperColor(g) }, { x: x.theta, y: cFactor(x.theta), color: gripperColor(g) }]),
    });
    funPlot.legend.replaceChildren(seriesLegend([{ label: '$\\theta/(2\\sin\\theta)>0$', color: p.accent }, { label: '$c(\\theta)<1$', color: p.warn }]));
}

onThemeChange(() => { scene.refreshTheme(); render(true); });
load();

// Shape control (Secs. II-E and IV-A): the closed loop of the multi-scale Procrustes
// controller (js/sim/controller.js) on an object simulated with ARAP (js/sim/world.js).
import { ErrorDeformationPlot, MatrixMap, ScaleBars, ScaleTimeMap, cssColor, signedColor } from '../lib/charts.js';
import { LinePlot } from '../lib/plot.js';
import { ballFlags, ghostPoses, gripperColor, isolineAnchor, nodeColors, screwPath } from '../lib/present.js';
import { initPage } from '../lib/shell.js';
import { onThemeChange, palette } from '../lib/theme.js';
import { button, card, control, fmt, h, label, legend, readouts, segmented, select, seriesLegend, slider, toggle } from '../lib/ui.js';
import { ObjectView } from '../lib/view3d.js';
import { DEFAULTS } from '../sim/controller.js';
import { isoline } from '../sim/geodesic.js';
import { apply, applyAll, eulerZYX, power } from '../sim/se3.js';
import { Session } from '../sim/session.js';
import { SCENARIOS, World } from '../sim/world.js';

await initPage();

const DEG = Math.PI / 180, K_MAX = 600;
const params = new URLSearchParams(location.search);
const settings = {
    scenario: SCENARIOS[params.get('scenario')] ? params.get('scenario') : 'sbend',
    colors: 'scales', target: true, isolines: true, poses: true, vectors: false, compare: false, lambda: 1, noise: 0.5,
    ...DEFAULTS,
};
let world = null, session = null, ctrl = null, out = null, history = null, running = false, speed = 1, lastWall = 0, budget = 0, frame = 0;
const cache = {};                                       // worlds by scenario (their preparation takes ~0.3 s)

// ---------------------------------------------------------------- layout
const workspace = document.getElementById('workspace');
const runCard = card('', '');
const scenarioPick = select({
    options: Object.entries(SCENARIOS).map(([value, s]) => ({ value, label: s.title })),
    value: settings.scenario, onChange: (v) => { settings.scenario = v; load(); },
});
const runButton = button('▶ Run', () => setRunning(!running), { primary: true });
const stepButton = button('Step', () => { setRunning(false); iterate(1); render(true); });
const restartButton = button('↺ Restart', () => { setRunning(false); restart(); });
const speedPick = segmented({ options: [1, 4, 16].map((v) => ({ value: v, label: `${v}×` })), value: 1, onChange: (v) => { speed = v; } });
const clockEl = h('span', { class: 'clock' }, 't = 0.0 s');
scenarioPick.el.style.width = 'auto';
runCard.body.append(h('div', { class: 'toolbar' }, h('b', { style: 'font-size:0.8rem' }, 'Scenario'), scenarioPick.el, runButton, stepButton, restartButton,
    h('span', { class: 'toolbar-divider' }), speedPick.el, clockEl), h('p', { class: 'hint', style: 'margin:8px 0 0' }, h('span', { id: 'scenarioNote' })));

const viewCard = card('Simulated object and target', 'drag to orbit, scroll to zoom');
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
        { label: '$\\Omega_1(t,r^*)$, isoline $D_1=r^*$', color: p.g1 }, { label: '$\\Omega_2(t,r^*)$, isoline $D_2=r^*$', color: p.g2 },
        { label: 'target $\\mathbf{Y}$', color: p.target, kind: 'band' }, { label: 'poses after $\\mathbf{T}_g(t,r^*)$', color: p.muted, kind: 'dashed' },
        { label: 'LRB estimate $\\hat{\\mathbf{X}}$', color: p.estimate, kind: 'band' }, { label: 'ARAP response', color: p.response, kind: 'band' },
    ]));
}

// multi-scale analysis at the current iteration
const nowCard = card('Multi-scale analysis at the current iteration', '');
const qCanvas = h('canvas', { class: 'chart-canvas' }), wCanvas = h('canvas', { class: 'chart-canvas' }), eCanvas = h('canvas', { class: 'chart-canvas' });
const qBars = new ScaleBars(qCanvas, { yLabel: 'q / max|q|', signed: true });
const wBars = new ScaleBars(wCanvas, { yLabel: "w / max w", xLabel: "r' [cm]", yRange: [0, 1] });
const eMap = new MatrixMap(eCanvas);
eCanvas.classList.add('square');
nowCard.body.append(h('div', { class: 'side-pair' },
    h('div', {}, h('p', { class: 'chart-title' }, label('$q(t,r)$ (11) and $r^*(t)$')), qCanvas),
    h('div', {}, h('p', { class: 'chart-title' }, label("LRB validation $w(t,r')$ (8)")), wCanvas)),
h('p', { class: 'chart-title', style: 'margin-top:10px' }, label("$\\hat e(t,r,r')$ (10), row $r^*$ in red")), h('div', { class: 'matrix-small' }, eCanvas),
h('div', { class: 'colorbar signed' }, h('span', {}, 'negative (error decreases)'), h('i'), h('span', {}, 'positive')));

// time plots, as in Fig. 2 of the paper
const errCard = card('Shape error and deformation of the object', 'as in Fig. 2');
const errCanvas = h('canvas', { class: 'chart-canvas', style: 'height:200px' });
const errPlot = new ErrorDeformationPlot(errCanvas);
const errLegend = h('div');
errCard.body.append(errCanvas, errLegend);
const mapCard = card('$q(t,r)$ and $r^*(t)$', 'normalized per iteration, as in Fig. 2');
const mapCanvas = h('canvas', { class: 'chart-canvas', style: 'height:200px' });
const qMap = new ScaleTimeMap(mapCanvas);
mapCard.body.append(mapCanvas);
const uCard = card('Translation $\\mathbf u_g$ [cm/s]', 'gripper center');
const uCanvas = h('canvas', { class: 'plot-canvas' });
const uPlot = new LinePlot(uCanvas, { xLabel: 't [s]' });
const wCard = card('Rotation $\\omega_g$ [rad/s]', 'roll, pitch, yaw');
const oCanvas = h('canvas', { class: 'plot-canvas' });
const oPlot = new LinePlot(oCanvas, { xLabel: 't [s]' });
const axisLegend = () => seriesLegend([
    { label: 'x', color: palette().axisX }, { label: 'y', color: palette().axisY }, { label: 'z', color: palette().axisZ },
    { label: 'g1', color: palette().muted }, { label: 'g2', color: palette().muted, dash: 'dashed' },
]);
const uLegend = h('div'), oLegend = h('div');
uCard.body.append(uCanvas, uLegend);
wCard.body.append(oCanvas, oLegend);

// rail
const readCard = card('Now', '');
const values = readouts([
    { key: 'k', label: 'iteration $k$' }, { key: 't', label: '$t$ [s]' }, { key: 'e', label: '$e(t)$ [cm]' },
    { key: 'rms', label: 'RMS [mm]', title: 'root mean square distance between matched points after Procrustes alignment, e(t) / sqrt(M)' },
    { key: 'r', label: '$r^*$ [cm]' }, { key: 'ms', label: 'ms / iteration', title: 'controller and ARAP step in this browser' },
]);
const chips = h('div', { class: 'chip-row' });
readCard.body.append(values.el, chips);

const layerCard = card('Layers', '');
const colorPick = segmented({
    options: [{ value: 'scales', label: 'balls' }, { value: 'match', label: 'surface map' }, { value: 'error', label: 'error' }, { value: 'distance', label: '$D_1$' }],
    value: settings.colors, onChange: (v) => { settings.colors = v; render(true); },
});
const layerToggles = [
    toggle({ label: 'target $\\mathbf Y$', value: settings.target, onChange: (v) => { settings.target = v; render(true); } }),
    toggle({ label: 'isolines $D_g=r^*$', value: settings.isolines, onChange: (v) => { settings.isolines = v; render(true); } }),
    toggle({ label: 'Procrustes poses and $\\mathrm{SE}(3)$ geodesics', value: settings.poses, onChange: (v) => { settings.poses = v; render(true); } }),
    toggle({ label: 'error vectors $\\mathbf y_m-\\mathbf x_m$', value: settings.vectors, onChange: (v) => { settings.vectors = v; render(true); } }),
    toggle({ label: 'LRB estimate and ARAP response', value: settings.compare, onChange: (v) => { settings.compare = v; render(true); } }),
];
const lambdaSlider = slider({ label: 'step $\\lambda$ of the comparison', description: 'fraction of $\\mathbf T_g(t,r^*)$ applied; $\\lambda=\\Delta t$ is the actual step',
    min: 0.05, max: 1, step: 0.05, value: settings.lambda, onInput: (v) => { settings.lambda = v; render(true); } });
layerCard.body.append(control('color of the object', colorPick.el), ...layerToggles.map((t) => t.el), lambdaSlider.el);

const ctrlCard = card('Controller', 'applies at once');
const betaSlider = slider({ label: 'relaxation $\\beta$ (8)', min: 0.01, max: 1e6, log: true, value: settings.beta, format: (v) => (v >= 100 ? v.toExponential(0) : v.toPrecision(2)),
    onInput: (v) => { settings.beta = v; ctrl?.set({ beta: v }); } });
const dtSlider = slider({ label: 'step $\\Delta t$ (13)', min: 0.02, max: 0.5, step: 0.01, value: settings.dt, onInput: (v) => { settings.dt = v; ctrl?.set({ dt: v }); } });
const innerSlider = slider({ label: 'gradient steps of (12) per iteration', min: 1, max: 20, step: 1, value: settings.innerIterations, format: (v) => v.toFixed(0),
    onInput: (v) => { settings.innerIterations = v; ctrl?.set({ innerIterations: v }); } });
const scalesSlider = slider({ label: 'number of scales (restarts)', min: 6, max: 30, step: 1, value: settings.nScales, format: (v) => v.toFixed(0),
    onChange: (v) => { settings.nScales = v; restart(); } });
const superPick = segmented({ options: [{ value: 'sum', label: 'sum (code)' }, { value: 'mean', label: '$\\frac1G\\sum$ (9)' }], value: settings.superposition,
    onChange: (v) => { settings.superposition = v; ctrl?.set({ superposition: v }); } });
const integratePick = segmented({ options: [{ value: 'influence', label: "over $r'$ (11)" }, { value: 'action', label: 'over $r$ (code)' }], value: settings.integrate,
    onChange: (v) => { settings.integrate = v; ctrl?.set({ integrate: v }); } });
ctrlCard.body.append(betaSlider.el, dtSlider.el, innerSlider.el, scalesSlider.el,
    h('details', { class: 'variants' }, h('summary', {}, 'Implementation variants'),
        control('superposition of the gripper increments', superPick.el), control("integration variable of $q$", integratePick.el)));

const targetCard = card('Target', 'gripper poses from the rest shape');
let editGripper = 0;
const gripperPick = segmented({ options: [{ value: 0, label: 'g1' }, { value: 1, label: 'g2' }], value: 0, onChange: (v) => { editGripper = v; syncTargetSliders(); } });
const poseSliders = [
    ...['x', 'y', 'z'].map((a, i) => slider({ label: `rot. ${a} [°]`, min: -90, max: 90, step: 1, value: 0, format: (v) => v.toFixed(0), onInput: (v) => editPose('rot', i, v * DEG) })),
    ...['x', 'y', 'z'].map((a, i) => slider({ label: `move ${a} [cm]`, min: -6, max: 6, step: 0.1, value: 0, format: (v) => v.toFixed(1), onInput: (v) => editPose('move', i, v) })),
];
let draftTarget = null;
const applyTarget = button('Apply target', () => { if (draftTarget) { setRunning(false); world.setTarget(draftTarget); restart(); } }, { primary: true });
const resetTarget = button('Scenario target', () => { draftTarget = clonePoses(scenarioTarget()); setRunning(false); world.setTarget(draftTarget); syncTargetSliders(); restart(); });
targetCard.body.append(control('gripper', gripperPick.el), h('div', { class: 'target-grid' }, ...poseSliders.map((s) => s.el)), h('div', { class: 'toolbar' }, applyTarget, resetTarget));

const senseCard = card('Perception', 'applies at once');
const noiseSlider = slider({ label: 'measurement noise $\\sigma$ [mm]', description: 'Gaussian noise added to every perceived point $\\mathbf x_m$ at every iteration',
    min: 0, max: 5, step: 0.1, value: settings.noise, format: (v) => v.toFixed(1), onInput: (v) => { settings.noise = v; if (session) session.noise = v; } });
senseCard.body.append(noiseSlider.el);

// the view stays in place; the plots and the settings are beside it
workspace.append(runCard.el, h('div', { class: 'dash' },
    h('div', { class: 'dash-view' }, viewCard.el),
    h('div', { class: 'dash-side' }, readCard.el, errCard.el, mapCard.el, nowCard.el, uCard.el, wCard.el, layerCard.el, ctrlCard.el, senseCard.el, targetCard.el)));

// ---------------------------------------------------------------- target editing
function scenarioTarget() {
    const s = SCENARIOS[settings.scenario];
    return s.target || s.grippers.map(() => ({ rot: [0, 0, 0], move: [0, 0, 0] }));
}
const clonePoses = (specs) => specs.map((s) => ({ rot: [...(s.rot || [0, 0, 0])], move: [...(s.move || [0, 0, 0])] }));
function syncTargetSliders() {
    if (!draftTarget) return;
    const s = draftTarget[editGripper];
    poseSliders.forEach((sl, i) => sl.set(i < 3 ? s.rot[i] / DEG : s.move[i - 3]));
}
function editPose(kind, axis, v) { if (draftTarget) draftTarget[editGripper][kind][axis] = v; }

// ---------------------------------------------------------------- simulation
function load() {
    setRunning(false);
    busy.hidden = false;
    document.getElementById('scenarioNote').textContent = `${SCENARIOS[settings.scenario].title}: ${SCENARIOS[settings.scenario].note}.`;
    setTimeout(() => {                                   // let the overlay paint first
        world = cache[settings.scenario] || (cache[settings.scenario] = new World(settings.scenario));
        draftTarget = clonePoses(world.targetSpec);
        syncTargetSliders();
        scene.setWorld(world);
        restart();
        busy.hidden = true;
    }, 30);
}

function restart() {
    session = new Session(world, {
        noise: settings.noise, nScales: settings.nScales, beta: settings.beta, dt: settings.dt, innerIterations: settings.innerIterations,
        superposition: settings.superposition, integrate: settings.integrate,
    });
    ctrl = session.ctrl; out = session.out;
    history = { t: [], e: [], eTrue: [], mean: [], std: [], r: [], q: [], u: world.grippers.map(() => [[], [], []]), o: world.grippers.map(() => [[], [], []]), ms: 0, X0: world.X() };
    record(0);
    render(true);
}

function record(ms) {
    history.t.push(session.t); history.e.push(out.e); history.r.push(out.rStar); history.q.push(out.qn);
    history.eTrue.push(session.trueError());
    const s = world.stretch(world.X(), history.X0);
    history.mean.push(s.mean); history.std.push(s.std);
    history.ms = ms;
    out.U.forEach((U, g) => {
        const c = world.poses[g].t, a = apply(U, c), eul = eulerZYX(U.R), dt = ctrl.options.dt;
        for (let i = 0; i < 3; i++) { history.u[g][i].push((a[i] - c[i]) / dt); history.o[g][i].push(eul[i] / dt); }
    });
}

// one iteration: the grippers execute the pending actions, then the controller analyzes the new shape
function iterate(n) {
    for (let i = 0; i < n && history.t.length < K_MAX; i++) {
        const t0 = performance.now();
        session.step();
        out = session.out;
        record(performance.now() - t0);
    }
}

function setRunning(flag) {
    running = flag && history && history.t.length < K_MAX;
    runButton.replaceChildren(label(running ? '❚❚ Pause' : '▶ Run'));
    if (running) { lastWall = performance.now(); budget = 0; requestAnimationFrame(loop); }
}

function loop(now) {
    if (!running) return;
    budget += Math.min((now - lastWall) / 1000, 0.1) * 10 * speed;   // 1x: ten iterations (one second of t) per second
    lastWall = now;
    const start = performance.now();
    while (budget >= 1 && performance.now() - start < 28) { iterate(1); budget -= 1; }
    budget = Math.min(budget, 2);
    render(++frame % 3 === 0);
    if (history.t.length >= K_MAX) setRunning(false);
    else requestAnimationFrame(loop);
}

// ---------------------------------------------------------------- views
function render(full = false) {
    if (!world || !out) return;
    const p = palette(), X = world.X(), Y = ctrl.Y, M = world.M, k = history.t.length - 1;
    // object and its colors
    scene.setObject(world.P);
    const inBall = out.balls.map((b, g) => ballFlags(b, out.ballCounts[g], M));
    scene.setColors(nodeColors(world, settings.colors, { inBall, X, Y, D: out.D[0], band: out.scales[1] - out.scales[0] }));
    // target: the simulated target lattice, aligned to X(t0) as Y is
    scene.setTarget(settings.target ? alignedTarget() : null);
    // isolines at r*
    const labels = [];
    out.D.forEach((D, g) => {
        const segs = settings.isolines && out.sStar < out.nS - 1 - 1e-9 ? isoline(X, world.surface.tris, D, out.rStar) : null;
        scene.setIsoline(g, segs);
        const anchor = segs && segs.length ? isolineAnchor(segs, world.poses[g].t) : null;
        if (anchor) labels.push({ p: anchor, text: `r* = ${out.rStar.toFixed(1)} cm`, color: gripperColor(g), cls: 'small', dy: -8 - 14 * g });
    });
    // grippers, their Procrustes poses and the SE(3) geodesics towards them
    const ghosts = settings.poses ? ghostPoses(world.poses, out.Tstar) : null;
    const paths = settings.poses ? out.Tstar.map((T, g) => screwPath(T, world.poses[g].t)) : null;
    scene.setGrippers(world.poses, ghosts, paths);
    world.poses.forEach((T, g) => labels.push({ p: T.t, text: `g${g + 1}`, color: gripperColor(g), dx: 10, dy: -22 }));
    scene.setLabels(labels);
    scene.setErrorVectors(settings.vectors ? X : null, settings.vectors ? Y : null);
    // LRB estimate and ARAP response after the fraction lambda of T_g(t, r*)
    if (settings.compare && (full || !running || frame % 6 === 0)) {
        const motions = out.Tstar.map((T) => power(T, settings.lambda));
        const est = ctrl.estimate(X, out.balls, out.ballCounts, out.ballCounts, settings.lambda).Xhat;
        scene.setSheet('estimate', est);
        const P = world.response(motions);
        const resp = new Float64Array(3 * M);
        for (let m = 0; m < M; m++) for (let e = 0; e < 3; e++) resp[3 * m + e] = P[3 * world.strip.top[m] + e];
        scene.setSheet('response', resp);
    } else if (!settings.compare) { scene.setSheet('estimate', null); scene.setSheet('response', null); }

    // readouts
    clockEl.textContent = `t = ${history.t[k].toFixed(1)} s`;
    values.set('k', String(k));
    values.set('t', history.t[k].toFixed(1));
    values.set('e', fmt(out.e, out.e < 1 ? 3 : 2));
    values.set('rms', fmt(10 * out.e / Math.sqrt(M), 2));
    values.set('r', fmt(out.rStar, 2));
    values.set('ms', history.ms ? history.ms.toFixed(1) : '–');
    chips.replaceChildren(
        h('span', { class: 'chip', style: `color:${p.rstar}` }, h('i'), `r* index ${out.sStar.toFixed(2)} of ${out.nS - 1}`),
        h('span', { class: 'chip' }, `M = ${M} points, ${world.strip.n} nodes`),
        h('span', { class: 'chip' }, out.validated ? 'w from the last action' : 'w = 1 (first iteration)'));

    // multi-scale analysis now
    qBars.set({ scales: out.scales, values: Array.from(out.qn), colors: (v) => cssColor(signedColor(v)), marker: out.rStar, markerLabel: 'r*' });
    wBars.set({ scales: out.scales, values: Array.from(out.wRel), color: p.accent, marker: null });
    eMap.set({ values: out.ehat, scales: out.scales, row: Math.round(out.sStar), sel: null });
    if (!full && running) return;
    // time plots (throttled while running)
    const noisy = history.e.some((e, i) => Math.abs(e - history.eTrue[i]) > 1e-9);
    errPlot.set({ t: history.t, e: history.e, eAlt: noisy ? history.eTrue : null, mean: history.mean, std: history.std });
    const red = p.dark ? '#ff6b6b' : '#d62728', blue = p.dark ? '#7aa7ff' : '#1f5fd1';
    errLegend.replaceChildren(seriesLegend([{ label: 'shape error $e(t)$, measured', color: red },
        ...(noisy ? [{ label: 'without the noise', color: red, dash: 'dashed' }] : []),
        { label: 'deformation, mean', color: blue }, { label: '± one standard deviation', color: blue, dash: 'band' }]));
    qMap.set({ times: history.t, columns: history.q, rStar: history.r, scales: out.scales });
    const series = (data) => data.flatMap((comp, g) => comp.map((y, i) => ({ x: history.t, y, color: [p.axisX, p.axisY, p.axisZ][i], dash: g ? [5, 4] : [], width: 1.4 })));
    uPlot.set({ series: series(history.u) });
    oPlot.set({ series: series(history.o) });
}

// the rendered target lattice, moved by the transform that aligned Y to X(t0)
let alignedCache = null;
function alignedTarget() {
    if (!alignedCache || alignedCache.ctrl !== ctrl) alignedCache = { ctrl, P: applyAll(ctrl.alignment, world.target.P) };
    return alignedCache.P;
}

onThemeChange(() => { scene.refreshTheme(); drawLegend(); uLegend.replaceChildren(axisLegend()); oLegend.replaceChildren(axisLegend()); render(true); });
drawLegend();
uLegend.append(axisLegend()); oLegend.append(axisLegend());
load();

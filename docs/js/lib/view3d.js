// 3D view (three.js) of the scene: the simulated object X(t) (closed surface of the
// ARAP lattice, with per-node colors), the target shape Y as a translucent ghost, the
// estimate of the LRB model and the simulated response on the perceived surface, the
// geodesic isolines {D_g = r} as thin tubes, the gripper boxes with the poses that
// their Procrustes transforms prescribe and the SE(3) geodesic towards them, and the
// error vectors between matched points. Labels are HTML placed at projected points.
import * as THREE from '../../vendor/three/three.module.min.js';
import { OrbitControls } from '../../vendor/three/OrbitControls.js';
import { palette } from './theme.js';

const UP = new THREE.Vector3(0, 1, 0);
const MAX_SEGMENTS = 6000;

export class ObjectView {
    constructor(container, { lineRadius = 0.075 } = {}) {
        this.container = container;
        this.lineRadius = lineRadius;
        try {
            this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
        } catch {
            container.append(Object.assign(document.createElement('p'), { className: 'view3d-hint', textContent: 'WebGL is not available in this browser.' }));
            this.failed = true;
            return;
        }
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        container.prepend(this.renderer.domElement);
        this.labelLayer = Object.assign(document.createElement('div'), { className: 'view3d-labels' });
        container.append(this.labelLayer);
        this.labels = [];
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(30, 1, 0.5, 600);
        this.camera.up.set(0, 0, 1);
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.12;
        this.scene.add(new THREE.HemisphereLight(0xffffff, 0x4a4f58, 1.9));
        const sun = new THREE.DirectionalLight(0xffffff, 1.6);
        sun.position.set(-20, -30, 50);
        this.scene.add(sun);
        const fill = new THREE.DirectionalLight(0xffffff, 0.5);
        fill.position.set(30, 20, -10);
        this.scene.add(fill);
        this.mat = {};
        this.isolines = [];
        this.dirty = true;
        new ResizeObserver(() => this.resize()).observe(container);
        this.controls.addEventListener('change', () => { this.dirty = true; });
        const loop = () => {
            if (this.disposed) return;
            if (this.controls.update() || this.dirty) { this.renderer.render(this.scene, this.camera); this.placeLabels(); this.dirty = false; }
            requestAnimationFrame(loop);
        };
        requestAnimationFrame(loop);
    }

    resize() {
        if (this.failed) return;
        const w = this.container.clientWidth, h = this.container.clientHeight;
        if (!w || !h) return;
        this.renderer.setSize(w, h, false);
        const first = !this.sized;
        this.sized = true;
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        if (first && this.world) this.frameView(this.world.scenario.view);
        this.dirty = true;
    }

    // ---------------------------------------------------------------- scene for a world
    setWorld(world) {
        if (this.failed) return;
        this.world = world;
        if (this.root) this.scene.remove(this.root);
        const root = new THREE.Group();
        this.root = root; this.scene.add(root);
        const p = palette(), { strip } = world;
        this.scene.background = new THREE.Color(p.scene);
        const surfaceGeometry = (count, index) => {
            const g = new THREE.BufferGeometry();
            g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * count), 3));
            g.setIndex(new THREE.BufferAttribute(index, 1));
            return g;
        };
        // object (closed surface of the lattice) with vertex colors
        const og = surfaceGeometry(strip.n, strip.surface);
        og.setAttribute('color', new THREE.BufferAttribute(new Float32Array(3 * strip.n).fill(0.8), 3));
        this.object = new THREE.Mesh(og, new THREE.MeshLambertMaterial({ vertexColors: true, polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }));
        this.object.frustumCulled = false;
        root.add(this.object);
        // target ghost
        this.target = new THREE.Mesh(surfaceGeometry(strip.n, strip.surface), this.material('target', p.target, { transparent: true, opacity: 0.26, depthWrite: false }));
        this.target.frustumCulled = false;
        this.target.renderOrder = 2;
        root.add(this.target);
        this.targetEdges = null;
        // perceived-surface sheets: LRB estimate and simulated response
        const topIndex = Uint32Array.from(strip.topTris);
        this.sheets = {};
        for (const [name, color, opacity] of [['estimate', p.estimate, 0.45], ['response', p.response, 0.4]]) {
            const mesh = new THREE.Mesh(surfaceGeometry(strip.M, topIndex), this.material(name, color, { transparent: true, opacity, depthWrite: false, side: THREE.DoubleSide }));
            mesh.frustumCulled = false; mesh.visible = false; mesh.renderOrder = 3;
            root.add(mesh);
            this.sheets[name] = mesh;
        }
        // isolines: tubes per gripper (primary) and thinner ones (secondary)
        const cyl = new THREE.CylinderGeometry(1, 1, 1, 7, 1, true);
        this.isolines = world.grippers.map((_, k) => {
            const make = (radius, key, color) => {
                const m = new THREE.InstancedMesh(cyl, this.material(key, color), MAX_SEGMENTS);
                m.count = 0; m.frustumCulled = false; m.userData.radius = radius;
                root.add(m);
                return m;
            };
            return { primary: make(this.lineRadius, `iso${k}`, p[`g${k + 1}`]), secondary: make(this.lineRadius * 0.55, `isoSoft${k}`, p.dark ? '#ffffff' : '#20242a') };
        });
        // grippers: box, edges, ghost box at the Procrustes pose, SE(3) path
        const box = new THREE.BoxGeometry(2, 2, 2), edges = new THREE.EdgesGeometry(box);
        this.grippers = world.grippers.map((g, k) => {
            const solid = new THREE.Mesh(box, this.material('gripper', p.gripper, { transparent: true, opacity: 0.32, depthWrite: false }));
            solid.renderOrder = 4;
            const outline = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: new THREE.Color(p.gripper) }));
            const ghost = new THREE.LineSegments(edges, new THREE.LineDashedMaterial({ color: new THREE.Color(p[`g${k + 1}`]), dashSize: 0.35, gapSize: 0.25 }));
            ghost.computeLineDistances();
            const path = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 64), 3)),
                new THREE.LineDashedMaterial({ color: new THREE.Color(p[`g${k + 1}`]), dashSize: 0.3, gapSize: 0.2 }));
            path.frustumCulled = false;
            for (const m of [solid, outline, ghost]) { m.matrixAutoUpdate = false; root.add(m); }
            root.add(path);
            ghost.visible = false; path.visible = false;
            return { solid, outline, ghost, path, half: g.half };
        });
        // error vectors
        const ev = new THREE.BufferGeometry();
        ev.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6 * strip.M), 3));
        this.errorLines = new THREE.LineSegments(ev, new THREE.LineBasicMaterial({ color: new THREE.Color(p.muted), transparent: true, opacity: 0.8 }));
        this.errorLines.frustumCulled = false; this.errorLines.visible = false;
        root.add(this.errorLines);
        // floor grid
        this.grid = new THREE.GridHelper(80, 32, 0x8a929c, 0x8a929c);
        this.grid.rotation.x = Math.PI / 2;
        this.grid.material.transparent = true; this.grid.material.opacity = p.dark ? 0.1 : 0.14;
        root.add(this.grid);
        this.frameView(world.scenario.view, [world.initial.P, world.target.P]);
        this.dirty = true;
    }

    material(key, color, opts = {}) {
        const m = new THREE.MeshLambertMaterial({ color: new THREE.Color(color), ...opts });
        this.mat[key] = m;
        return m;
    }

    // ---------------------------------------------------------------- state
    setPositions(mesh, P) {
        const a = mesh.geometry.attributes.position;
        a.array.set(P);
        a.needsUpdate = true;
        mesh.geometry.computeVertexNormals();
        this.dirty = true;
    }
    setObject(P) {
        if (this.failed) return;
        this.setPositions(this.object, P);
        let zMin = Infinity;
        for (let i = 2; i < P.length; i += 3) zMin = Math.min(zMin, P[i]);
        this.grid.position.z = Math.min(this.grid.position.z ?? 0, zMin - 2.5);
    }
    setColors(C) {
        if (this.failed) return;
        const a = this.object.geometry.attributes.color;
        a.array.set(C); a.needsUpdate = true;
        this.dirty = true;
    }
    setTarget(P) {
        if (this.failed) return;
        this.target.visible = !!P;
        if (P) this.setPositions(this.target, P);
        this.dirty = true;
    }
    setSheet(name, X) {
        if (this.failed) return;
        const mesh = this.sheets[name];
        mesh.visible = !!X;
        if (X) this.setPositions(mesh, X);
        this.dirty = true;
    }

    // segments: Float32Array (x0, y0, z0, x1, y1, z1, ...) or null
    setIsoline(k, segments, which = 'primary') {
        if (this.failed || !this.isolines[k]) return;
        const mesh = this.isolines[k][which], r = mesh.userData.radius;
        const m = new THREE.Matrix4(), q = new THREE.Quaternion(), a = new THREE.Vector3(), b = new THREE.Vector3(), d = new THREE.Vector3();
        let count = 0;
        if (segments) {
            for (let s = 0; s + 5 < segments.length && count < MAX_SEGMENTS; s += 6) {
                a.set(segments[s], segments[s + 1], segments[s + 2]);
                b.set(segments[s + 3], segments[s + 4], segments[s + 5]);
                d.subVectors(b, a);
                const len = d.length();
                if (len < 1e-6) continue;
                q.setFromUnitVectors(UP, d.multiplyScalar(1 / len));
                m.compose(a.add(b).multiplyScalar(0.5), q, new THREE.Vector3(r, len + r, r));
                mesh.setMatrixAt(count++, m);
            }
        }
        mesh.count = count;
        mesh.instanceMatrix.needsUpdate = true;
        this.dirty = true;
    }

    // poses: [{R, t}]; ghosts: [{R, t}] | null (Procrustes poses); paths: [[[x, y, z], ...]] | null
    setGrippers(poses, ghosts = null, paths = null) {
        if (this.failed) return;
        this.grippers.forEach((g, k) => {
            const set = (obj, T) => {
                const { R, t } = T, h = g.half;
                obj.matrix.set(R[0] * h[0], R[1] * h[1], R[2] * h[2], t[0], R[3] * h[0], R[4] * h[1], R[5] * h[2], t[1], R[6] * h[0], R[7] * h[1], R[8] * h[2], t[2], 0, 0, 0, 1);
            };
            set(g.solid, poses[k]); set(g.outline, poses[k]);
            g.ghost.visible = !!ghosts;
            if (ghosts) set(g.ghost, ghosts[k]);
            g.path.visible = !!paths;
            if (paths) {
                const attr = g.path.geometry.attributes.position, pts = paths[k], n = Math.min(64, pts.length);
                for (let i = 0; i < n; i++) attr.setXYZ(i, pts[i][0], pts[i][1], pts[i][2]);
                attr.needsUpdate = true;
                g.path.geometry.setDrawRange(0, n);
                g.path.computeLineDistances();
            }
        });
        this.dirty = true;
    }

    setErrorVectors(X, Y) {
        if (this.failed) return;
        this.errorLines.visible = !!(X && Y);
        if (X && Y) {
            const a = this.errorLines.geometry.attributes.position.array;
            for (let m = 0; m < X.length / 3; m++) {
                a[6 * m] = X[3 * m]; a[6 * m + 1] = X[3 * m + 1]; a[6 * m + 2] = X[3 * m + 2];
                a[6 * m + 3] = Y[3 * m]; a[6 * m + 4] = Y[3 * m + 1]; a[6 * m + 5] = Y[3 * m + 2];
            }
            this.errorLines.geometry.attributes.position.needsUpdate = true;
        }
        this.dirty = true;
    }

    // ---------------------------------------------------------------- labels and camera
    // items: [{p: [x, y, z], text, color, cls, dx, dy}]
    setLabels(items) { this.extraLabels = items; this.dirty = true; }

    placeLabels() {
        const items = this.extraLabels || [];
        while (this.labels.length < items.length) {
            const el = document.createElement('span');
            this.labelLayer.append(el);
            this.labels.push(el);
        }
        const w = this.container.clientWidth, h = this.container.clientHeight, v = new THREE.Vector3();
        this.labels.forEach((el, k) => {
            const item = items[k];
            if (!item) { el.style.display = 'none'; return; }
            v.set(item.p[0], item.p[1], item.p[2]).project(this.camera);
            if (v.z > 1) { el.style.display = 'none'; return; }
            el.style.display = '';
            el.className = `label3d ${item.cls || ''}`;
            el.textContent = item.text;
            if (item.color) el.style.color = item.color; else el.style.removeProperty('color');
            el.style.transform = `translate(${((v.x + 1) / 2 * w + (item.dx ?? 6)).toFixed(1)}px, ${((1 - v.y) / 2 * h + (item.dy ?? -8)).toFixed(1)}px)`;
        });
    }

    // Camera along the direction of view.from - view.at, at the distance that fits the
    // bounding sphere of the given point sets (flat arrays) in the view.
    frameView(view, pointSets = this.fitSets) {
        if (this.failed || !view) return;
        this.fitSets = pointSets;
        const dir = new THREE.Vector3(...view.from).sub(new THREE.Vector3(...view.at)).normalize();
        const box = new THREE.Box3();
        for (const P of pointSets || []) for (let i = 0; i < P.length; i += 3) box.expandByPoint(new THREE.Vector3(P[i], P[i + 1], P[i + 2]));
        if (box.isEmpty()) box.set(new THREE.Vector3(...view.at).subScalar(10), new THREE.Vector3(...view.at).addScalar(10));
        const center = box.getCenter(new THREE.Vector3()), radius = box.getSize(new THREE.Vector3()).length() / 2;
        const vfov = this.camera.fov * Math.PI / 180, hfov = 2 * Math.atan(Math.tan(vfov / 2) * Math.max(this.camera.aspect || 1, 0.5));
        const dist = 1.08 * radius / Math.sin(Math.min(vfov, hfov) / 2);
        this.controls.target.copy(center);
        this.camera.position.copy(center).addScaledVector(dir, dist);
        this.controls.update();
        this.dirty = true;
    }

    refreshTheme() {
        if (this.failed || !this.world) return;
        const p = palette();
        this.scene.background = new THREE.Color(p.scene);
        const set = (key, c) => this.mat[key]?.color.set(c);
        set('target', p.target); set('estimate', p.estimate); set('response', p.response); set('gripper', p.gripper);
        this.isolines.forEach((_, k) => { set(`iso${k}`, p[`g${k + 1}`]); set(`isoSoft${k}`, p.dark ? '#ffffff' : '#20242a'); });
        this.grippers.forEach((g, k) => { g.outline.material.color.set(p.gripper); g.ghost.material.color.set(p[`g${k + 1}`]); g.path.material.color.set(p[`g${k + 1}`]); });
        this.errorLines.material.color.set(p.muted);
        this.grid.material.opacity = p.dark ? 0.1 : 0.14;
        this.dirty = true;
    }

    snapshot() { return this.failed ? null : this.renderer.domElement; }
}

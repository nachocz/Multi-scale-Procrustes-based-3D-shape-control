// One closed-loop run: the simulated world, the perception (perceived points with
// measurement noise), and the multi-scale Procrustes controller. An iteration executes
// the pending gripper actions and then analyzes the new perceived shape, so that the
// state and its analysis (`out`) always correspond.
import { MultiScaleProcrustes } from './controller.js';
import { vertexTriangles } from './geodesic.js';
import { rng } from './linalg.js';
import { procrustes } from './procrustes.js';

export class Session {
    // options: controller options (js/sim/controller.js), noise [mm], seed, and target
    // (perceived target points; the target of the world by default)
    constructor(world, { noise = 0.5, seed = 1, target = null, ...controller } = {}) {
        this.world = world;
        this.noise = noise;
        this.seed = seed;
        this.target = target;
        this.options = { planar: world.planar, ...controller };
        this.surface = { tris: world.surface.tris, adjacency: vertexTriangles(world.surface.tris, world.M) };
        this.restart();
    }

    // Perceived points: the top layer of the simulated object plus Gaussian noise (in the
    // plane for a planar piece).
    measure() {
        const X = this.world.X(), s = this.noise / 10, r = this.random, step = this.world.planar ? 3 : 1;
        if (s > 0) {
            for (let i = 0; i < X.length; i++) {
                if (step === 3 && i % 3 === 2) continue;
                X[i] += s * Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r());
            }
        }
        return X;
    }

    restart() {
        const { world } = this;
        world.reset();
        this.random = rng(this.seed);
        this.ctrl = new MultiScaleProcrustes(this.surface, world.grippers.map((g) => g.source), world.r0, this.options);
        this.ctrl.start(this.measure(), this.target || world.Y);
        this.k = 0;
        this.t = 0;
        this.X = this.measure();
        this.out = this.ctrl.step(this.X);
    }

    step() {
        const dt = this.ctrl.options.dt;
        this.world.move(this.out.U);
        this.k += 1;
        this.t += dt;
        this.X = this.measure();
        this.out = this.ctrl.step(this.X);
    }

    // Shape error of the noise-free perceived points.
    trueError() { return procrustes(this.world.X(), this.ctrl.Y, null, this.ctrl.options.planar).d; }
}

"""Interactive demo of multi-scale Procrustes 3D shape control.

Run with:
    python -m python.demo
"""

from __future__ import annotations

import os
import sys
import numpy as np
import trimesh
import polyscope as ps

# Ensure the package is importable when run as a script
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from python.procrustes import procrustes
from python.surface_map import spectral_match, nearest_vertex_map
from python.multiscale_control import MultiScaleProcrustesController
from python.geodesic import geodesic_distances
from python.deformation import geodesic_blend_transforms

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
MESH_DIR = os.path.join(_ROOT, "cuiral-meshes")
SRC_MESH = os.path.join(MESH_DIR, "shape.stl")
TGT_MESH = os.path.join(MESH_DIR, "shape_target.stl")
MAP_FILE = os.path.join(MESH_DIR, "surfacemap.mat")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_mesh(path: str):
    """Load an STL mesh and return (V, F)."""
    mesh = trimesh.load_mesh(path)
    V = np.array(mesh.vertices, dtype=np.float64)
    F = np.array(mesh.faces, dtype=np.intp)
    return V, F


def simplify_mesh(V: np.ndarray, F: np.ndarray, target_faces: int):
    """Simplify a triangle mesh to approximately *target_faces* faces."""
    mesh = trimesh.Trimesh(vertices=V, faces=F, process=False)
    ratio = max(0.01, min(1.0, 1.0 - target_faces / len(F)))
    simplified = mesh.simplify_quadric_decimation(ratio)
    return np.array(simplified.vertices, dtype=np.float64), np.array(simplified.faces, dtype=np.intp)


def load_surface_map(path: str):
    """Load the pre-computed surface map and gripper indices from a .mat file.

    The .mat file is expected to contain:
    - T12     : (N,) point-to-point map
    - gIndex  : (G,) gripper vertex indices
    """
    from scipy.io import loadmat

    data = loadmat(path, squeeze_me=True)
    T12 = np.asarray(data["T12"], dtype=np.intp).ravel() - 1      # MATLAB 1-based → 0-based
    gIndex = np.asarray(data["gIndex"], dtype=np.intp).ravel() - 1
    return T12, gIndex


def pick_grippers_auto(V: np.ndarray, F: np.ndarray, n: int = 2) -> np.ndarray:
    """Select *n* vertices that are far apart as automatic gripper positions."""
    D = geodesic_distances(V, F)
    # Begin with the vertex farthest from vertex 0
    idx0 = int(np.argmax(D[0]))
    grippers = [idx0]
    for _ in range(n - 1):
        min_dists = D[grippers].min(axis=0)
        next_g = int(np.argmax(min_dists))
        grippers.append(next_g)
    return np.array(grippers, dtype=np.intp)


def colormap_by_correspondence(V_src, V_tgt, T12):
    """Build per-vertex colour array so matched vertices share the same colour."""
    # Map target vertex index to a hue
    n_tgt = len(V_tgt)
    hue = np.zeros(len(V_src))
    for i, t in enumerate(T12):
        hue[i] = t / max(n_tgt, 1)
    # Convert hue to RGB via HSV
    import colorsys
    colors = np.array([colorsys.hsv_to_rgb(h, 0.8, 0.9) for h in hue])
    return colors


# ---------------------------------------------------------------------------
# Main demo
# ---------------------------------------------------------------------------

TARGET_FACES = 600  # simplify meshes for manageable computation


def main():
    # ---- Load meshes ----
    if not os.path.isfile(SRC_MESH):
        print(f"Source mesh not found: {SRC_MESH}")
        sys.exit(1)

    V_src_full, F_src_full = load_mesh(SRC_MESH)
    V_tgt_full, F_tgt_full = load_mesh(TGT_MESH)

    # Simplify for tractable computation (matches MATLAB's use of surf2mesh)
    print(f"Simplifying meshes to ~{TARGET_FACES} faces …")
    V_src, F_src = simplify_mesh(V_src_full, F_src_full, TARGET_FACES)
    V_tgt, F_tgt = simplify_mesh(V_tgt_full, F_tgt_full, TARGET_FACES)

    # ---- Surface matching ----
    # The pre-computed map was made for the MATLAB-specific remesh, so we
    # always compute our own spectral match on the simplified meshes.
    print("Computing spectral surface map …")
    T12 = spectral_match(V_src, F_src, V_tgt, F_tgt, k=30)
    gripper_indices = pick_grippers_auto(V_src, F_src, n=2)

    print(f"Source: {len(V_src)} verts, {len(F_src)} faces")
    print(f"Target: {len(V_tgt)} verts, {len(F_tgt)} faces")
    print(f"Grippers: {gripper_indices}")

    # ---- Controller ----
    ctrl = MultiScaleProcrustesController(
        V0=V_src,
        F=F_src,
        Vt=V_tgt,
        Ft=F_tgt,
        T12=T12,
        gripper_indices=gripper_indices,
        dt=0.1,
        beta=1e4,
        n_scales=20,
    )

    # ---- Large gripper index sets (vertices near each gripper) ----
    D_src = geodesic_distances(V_src, F_src)
    large_grip = []
    for g in gripper_indices:
        near = np.where(D_src[g] <= ctrl.gripper_size)[0]
        large_grip.append(np.union1d(near, [g]))

    # ---- Polyscope visualisation setup ----
    ps.init()
    ps.set_up_dir("z_up")
    ps.set_ground_plane_mode("shadow_only")

    colors_src = colormap_by_correspondence(ctrl.V, ctrl.Vt, T12)

    ps_src = ps.register_surface_mesh("current", ctrl.V, F_src, smooth_shade=True)
    ps_src.add_color_quantity("correspondence", colors_src, enabled=True)

    colors_tgt = colormap_by_correspondence(ctrl.Vt[T12], ctrl.Vt, np.arange(len(T12)))
    ps_tgt = ps.register_surface_mesh("target", ctrl.Vt, F_tgt, smooth_shade=True, transparency=0.4)
    ps_tgt.add_color_quantity("correspondence", colormap_by_correspondence(
        ctrl.Vt, ctrl.Vt, np.arange(len(ctrl.Vt))
    ), enabled=True)

    # Gripper markers
    grip_pts = ctrl.V[gripper_indices]
    ps_grip = ps.register_point_cloud("grippers", grip_pts, radius=0.015)
    ps_grip.add_color_quantity("col", np.tile([0.1, 0.1, 0.1], (len(gripper_indices), 1)), enabled=True)

    # Pre-compute geodesic distances from each vertex to each gripper
    D_to_grippers = D_src[:, gripper_indices]  # (N, G)

    # ---- Simulation parameters ----
    max_iter = 200
    running = False
    step_count = [0]
    error_history: list[float] = []

    def callback():
        nonlocal running

        import polyscope.imgui as psim

        psim.PushItemWidth(200)

        if psim.Button("Run / Pause"):
            running = not running

        single_step = psim.Button("Single step")

        psim.Text(f"Iteration: {step_count[0]}")
        if error_history:
            psim.Text(f"Error: {error_history[-1]:.6f}")

        if psim.Button("Reset"):
            ctrl.V[:] = V_src
            ctrl.iteration = 0
            ctrl.action_scale_history.clear()
            ctrl.error_history.clear()
            ctrl._prev_estimated_V = None
            step_count[0] = 0
            error_history.clear()
            running = False
            _update_vis()

        psim.PopItemWidth()

        if (running or single_step) and step_count[0] < max_iter:
            _do_step()

    def _do_step():
        res = ctrl.step()

        # Simulate deformation via geodesic-weighted rigid transform blending
        ctrl.V[:] = geodesic_blend_transforms(
            ctrl.V, D_to_grippers, res.gripper_transforms
        )

        step_count[0] += 1
        error_history.append(res.error)
        _update_vis()
        if res.error < 1e-4:
            nonlocal running
            running = False
            print(f"Converged at iteration {step_count[0]}, error={res.error:.6f}")

    def _update_vis():
        ps_src.update_vertex_positions(ctrl.V)
        grip_pts = ctrl.V[gripper_indices]
        ps_grip.update_point_positions(grip_pts)

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    main()

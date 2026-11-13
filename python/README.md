# Multi-scale Procrustes-based 3D Shape Control — Python Implementation

Clean Python implementation of the method described in:

> **Multi-scale Procrustes-based 3D shape control**  
> I. Cuiral-Zueco and G. López-Nicolás  
> IEEE Transactions on Mechatronics, 2023

## Quick start

```bash
# Create & activate virtual environment
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux / macOS
source .venv/bin/activate

# Install dependencies
pip install -r python/requirements.txt

# Run the demo
python -m python.demo
```

## Project layout

```
python/
├── requirements.txt          # pip dependencies
├── __init__.py
├── procrustes.py             # Orthogonal Procrustes solver
├── geodesic.py               # Geodesic distances & iso-distance segments
├── rigid_transform.py        # SE(3) geodesic path interpolation
├── deformation.py            # Mesh deformation (geodesic-weighted + Laplacian)
├── multiscale_control.py     # Multi-scale Procrustes control loop (core)
├── surface_map.py            # Functional map utilities (spectral matching)
├── demo.py                   # Interactive Polyscope demo
└── __main__.py               # Entry-point for `python -m python.demo`
```

## Method overview

1. **Surface matching** — Functional maps (via the Laplace-Beltrami spectrum)
   provide point-to-point correspondences between the current mesh and the
   target mesh.
2. **Multi-scale Procrustes analysis** — For each gripper, geodesic balls of
   increasing radius define subsets of the mesh. An orthogonal Procrustes
   analysis at every scale quantifies how rigidly each subset moves.
3. **Optimal scale selection** — An error-reduction effectiveness surface
   *q(t, r)* is computed; a gradient update selects the locally optimal
   radius *r\** at which the control action is most effective.
4. **Control law** — The SE(3) rigid transform from the Procrustes analysis at
   scale *r\** is applied as a small incremental gripper action along the
   geodesic path in SE(3).

## Visualisation

The demo uses [Polyscope](https://polyscope.run/) for interactive 3D rendering.
Both the current and target meshes are displayed with a colour-mapped surface
correspondence, and grippers are shown as point-cloud markers.

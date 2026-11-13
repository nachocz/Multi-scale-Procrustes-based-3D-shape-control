"""Multi-scale Procrustes shape control — core algorithm.

Implements the control loop described in:
    *Multi-scale Procrustes-based 3D shape control*,
    I. Cuiral-Zueco & G. López-Nicolás, IEEE T-Mech 2023.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field

from .procrustes import procrustes
from .rigid_transform import rigid_transform_geodesic_path
from .geodesic import geodesic_distances, compute_iso_distance_segments


# ---------------------------------------------------------------------------
# Data container for one iteration's output
# ---------------------------------------------------------------------------
@dataclass
class IterationResult:
    """Stores quantities computed during one control iteration."""

    error: float = 0.0
    action_scale: float = 1.0
    gripper_transforms: list[np.ndarray] = field(default_factory=list)
    q_surface: np.ndarray | None = None


# ---------------------------------------------------------------------------
# Controller class
# ---------------------------------------------------------------------------
class MultiScaleProcrustesController:
    """Multi-scale Procrustes shape controller.

    Parameters
    ----------
    V0        : (N, 3)  initial vertex positions (source / current shape)
    F         : (Nf, 3) face array of the source mesh
    Vt        : (Nt, 3) target vertex positions
    Ft        : (Nft, 3) face array of the target mesh
    T12       : (N,) int  point-to-point map  source → target
    gripper_indices : (G,) int  vertex indices of the grippers in the source mesh
    dt        : float   time-step  (controls magnitude of incremental actions)
    beta      : float   LRB relaxation parameter  (default 1e4)
    n_scales  : int     number of discrete scale samples
    scale_window : int  half-width of the sliding scale window (after 1st iter)
    gripper_size : float  minimum geodesic radius around grippers
    """

    def __init__(
        self,
        V0: np.ndarray,
        F: np.ndarray,
        Vt: np.ndarray,
        Ft: np.ndarray,
        T12: np.ndarray,
        gripper_indices: np.ndarray,
        *,
        dt: float = 0.1,
        beta: float = 1e4,
        n_scales: int = 20,
        scale_window: int = 5,
        gripper_size: float = 1.0,
    ):
        self.F = np.asarray(F, dtype=np.intp)
        self.Ft = np.asarray(Ft, dtype=np.intp)
        self.T12 = np.asarray(T12, dtype=np.intp)
        self.gripper_indices = np.asarray(gripper_indices, dtype=np.intp)
        self.dt = dt
        self.beta = beta
        self.n_scales = n_scales
        self.scale_window = scale_window
        self.gripper_size = gripper_size

        self.V = np.array(V0, dtype=np.float64)
        self.Vt = np.array(Vt, dtype=np.float64)
        self.N = len(self.V)
        self.G = len(self.gripper_indices)

        # ---- initial global Procrustes alignment of target to current ----
        # procrustes(X, Y) returns Tg mapping X→Y, i.e. Vt[T12]→V.
        # Apply Tg to ALL of Vt to bring target into source frame.
        dp, Tg, _ = procrustes(self.Vt[self.T12], self.V)
        Vt_h = np.hstack([self.Vt, np.ones((len(self.Vt), 1))])
        self.Vt = (Tg @ Vt_h.T).T[:, :3]

        # State
        self.iteration = 0
        self.action_scale_history: list[float] = []
        self.error_history: list[float] = []
        self._prev_estimated_V: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def step(self) -> IterationResult:
        """Run one control iteration.  Returns an IterationResult."""
        V = self.V
        Vt = self.Vt
        T12 = self.T12
        F = self.F
        G = self.G
        dt = self.dt
        n_scales = self.n_scales

        # ----- geodesic distances on current shape ----- 
        D = geodesic_distances(V, F)

        # ----- scale vector -----
        grid_step = _mean_edge_length(V, F)
        max_dist = D.max()
        scale_vec = np.linspace(grid_step * 2, max_dist, n_scales)

        # ----- iso-distance masks -----
        masks = compute_iso_distance_segments(V, F, scale_vec, self.gripper_indices, D)

        # ----- global Procrustes error -----
        d_pro, _, _ = procrustes(V, Vt[T12])
        self.error_history.append(d_pro)

        # ----- Determine scale search range -----
        if self.iteration == 0:
            s_lo, s_hi = 0, n_scales
        else:
            prev_s = self.action_scale_history[-1]
            s_lo = max(int(round(prev_s)) - self.scale_window, 0)
            s_hi = min(int(round(prev_s)) + self.scale_window + 1, n_scales)

        # ----- Multi-scale Procrustes analysis -----
        Eproc = np.full((n_scales, G), np.nan)
        iTransform = np.zeros((4, 4, n_scales, G))

        for s in range(s_lo, s_hi):
            for g in range(G):
                idx = masks[:, s, g]
                if idx.sum() < 4:
                    continue
                dp_s, T_s, _ = procrustes(V[idx], Vt[T12[idx]])
                Eproc[s, g] = dp_s * (idx.sum() / self.N)
                iTransform[:, :, s, g] = T_s

        # ----- Estimate error-reduction surface  q(r, r') -----
        est_dpro = np.full((n_scales, n_scales), np.nan)
        estimatedV = np.zeros((self.N, 3, n_scales, n_scales))

        for s1 in range(s_lo, s_hi):  # action scale
            for s2 in range(s_lo, s_hi):  # influence scale
                est_v = V.copy()
                for g in range(G):
                    idx = masks[:, s2, g]
                    if idx.sum() < 4:
                        continue
                    T_act = iTransform[:, :, s1, g]
                    if np.allclose(T_act, 0):
                        continue
                    T_out = rigid_transform_geodesic_path(dt, np.eye(4), T_act)
                    Vh = np.hstack([V, np.ones((self.N, 1))])
                    new_v = (T_out @ Vh.T).T[:, :3]
                    increment = new_v - V
                    est_v[idx] += increment[idx]
                estimatedV[:, :, s1, s2] = est_v
                dp_est, _, _ = procrustes(est_v, Vt[T12])
                est_dpro[s1, s2] = dp_est

        est_dot_dpro = est_dpro - d_pro  # (n_scales, n_scales)

        # ----- LRB weighting  w(t, r') -----
        if self.iteration == 0 or self._prev_estimated_V is None:
            w = np.ones(n_scales)
        else:
            prev_s_idx = int(round(self.action_scale_history[-1]))
            prev_s_idx = np.clip(prev_s_idx, 0, n_scales - 1)
            d_est_quality = np.full(n_scales, 10.0)
            for s2 in range(s_lo, s_hi):
                if s2 >= self._prev_estimated_V.shape[3]:
                    continue
                prev_est = self._prev_estimated_V[:, :, prev_s_idx, s2]
                if np.allclose(prev_est, 0):
                    continue
                dq, _, _ = procrustes(prev_est, V)
                d_est_quality[s2] = dq / self.N
            w = np.exp(-self.beta * d_est_quality)  # w ∈ (0, 1]

        # ----- q(r) = Σ_{r'} ê(r, r') · w(r') -----
        # Weight the estimation surface
        weighted_est = np.where(np.isnan(est_dot_dpro), 0.0, est_dot_dpro) * w[np.newaxis, :]
        q = np.nansum(weighted_est, axis=1)  # (n_scales,)

        # Normalise
        max_abs = np.max(np.abs(q))
        if max_abs > 0:
            q /= max_abs

        # ----- Optimal scale r* -----
        if self.iteration == 0:
            action_scale = float(np.argmin(q))
        else:
            # Gradient descent on q(r)
            grad_q = np.gradient(q)
            opt_s = self.action_scale_history[-1]
            for _ in range(10):
                grad_at_s = np.interp(opt_s, np.arange(n_scales), grad_q)
                opt_s = opt_s - grad_at_s
            action_scale = float(np.clip(opt_s, 0, n_scales - 1))

        self.action_scale_history.append(action_scale)

        # ----- Compute gripper control actions -----
        s_idx = int(round(action_scale))
        s_idx = np.clip(s_idx, 0, n_scales - 1)
        gain = abs(q[s_idx]) * dt if self.iteration > 0 else dt

        gripper_transforms = []
        for g in range(G):
            pg = V[self.gripper_indices[g]]
            idx = masks[:, s_idx, g]
            if idx.sum() < 4:
                gripper_transforms.append(np.eye(4))
                continue
            dp_g, T_g, _ = procrustes(V[idx] - pg, Vt[T12[idx]] - pg)
            T_out = rigid_transform_geodesic_path(gain, np.eye(4), T_g)
            gripper_transforms.append(T_out)

        self._prev_estimated_V = estimatedV
        self.iteration += 1

        return IterationResult(
            error=d_pro,
            action_scale=action_scale,
            gripper_transforms=gripper_transforms,
            q_surface=q,
        )

    def apply_actions(
        self,
        gripper_transforms: list[np.ndarray],
        large_gripper_indices: list[np.ndarray] | None = None,
    ):
        """Apply the computed gripper transforms to update vertex positions.

        For a real system this sends commands to the robots; here we provide a
        simple *rigid* update of the gripper neighbourhood vertices (the demo
        couples this with an ARAP deformation solver when available).
        """
        for g, T_out in enumerate(gripper_transforms):
            if large_gripper_indices is not None:
                gidx = large_gripper_indices[g]
            else:
                gidx = np.array([self.gripper_indices[g]])
            Vh = np.hstack([self.V, np.ones((self.N, 1))])
            new_V = (T_out @ Vh.T).T[:, :3]
            self.V[gidx] = new_V[gidx]


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
def _mean_edge_length(V: np.ndarray, F: np.ndarray) -> float:
    edges = np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    edges = np.sort(edges, axis=1)
    edges = np.unique(edges, axis=0)
    lengths = np.linalg.norm(V[edges[:, 0]] - V[edges[:, 1]], axis=1)
    return float(lengths.mean())

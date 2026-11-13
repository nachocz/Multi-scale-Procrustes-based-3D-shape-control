"""Mesh deformation solvers for shape control simulation.

Provides two approaches:
1. Laplacian-based deformation (smooth but doesn't preserve local rigidity)
2. Geodesic-weighted rigid transform blending (better matches LRB assumption)
"""

import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import spsolve


def cotangent_laplacian(V: np.ndarray, F: np.ndarray) -> sp.csr_matrix:
    """Compute the cotangent Laplacian matrix for a triangle mesh."""
    n = len(V)
    I, J, W = [], [], []

    for face in F:
        for k in range(3):
            i, j, opp = face[k], face[(k + 1) % 3], face[(k + 2) % 3]
            e1 = V[i] - V[opp]
            e2 = V[j] - V[opp]
            cos_a = np.dot(e1, e2)
            sin_a = np.linalg.norm(np.cross(e1, e2))
            if sin_a < 1e-12:
                continue
            cot_a = cos_a / sin_a
            w = 0.5 * cot_a
            I.extend([i, j])
            J.extend([j, i])
            W.extend([w, w])

    L = sp.coo_matrix((W, (I, J)), shape=(n, n)).tocsr()
    diag = np.array(L.sum(axis=1)).flatten()
    L = L - sp.diags(diag)
    return L


def laplacian_deform(
    V: np.ndarray,
    F: np.ndarray,
    handle_indices: np.ndarray,
    handle_positions: np.ndarray,
    *,
    L: sp.csr_matrix | None = None,
) -> np.ndarray:
    """Deform a mesh given new handle positions using bi-Laplacian smoothing."""
    n = len(V)
    handle_indices = np.asarray(handle_indices, dtype=np.intp)
    handle_positions = np.asarray(handle_positions, dtype=np.float64)

    if L is None:
        L = cotangent_laplacian(V, F)

    LtL = L.T @ L
    lam = 1e6

    C = sp.csr_matrix(
        (np.ones(len(handle_indices)), (np.arange(len(handle_indices)), handle_indices)),
        shape=(len(handle_indices), n),
    )

    A = LtL + lam * C.T @ C
    rhs_base = LtL @ V

    V_new = np.zeros_like(V)
    for d in range(3):
        rhs = rhs_base[:, d].copy()
        rhs += lam * C.T @ handle_positions[:, d] - lam * C.T @ (C @ V[:, d])
        rhs[handle_indices] += lam * handle_positions[:, d]
        V_new[:, d] = spsolve(A.tocsc(), rhs)

    return V_new


def geodesic_blend_transforms(
    V: np.ndarray,
    D_to_grippers: np.ndarray,
    gripper_transforms: list[np.ndarray],
    sigma: float | None = None,
) -> np.ndarray:
    """Deform a mesh by blending per-gripper rigid transforms via geodesic distance.

    Each vertex gets a weighted combination of the gripper transforms, with
    weights decaying as a Gaussian of geodesic distance. This produces a smooth
    deformation that preserves local rigidity near the grippers — closely
    matching the paper's LRB assumption.

    Parameters
    ----------
    V                  : (N, 3)   vertex positions
    D_to_grippers      : (N, G)   geodesic distance from each vertex to each gripper
    gripper_transforms : list of G (4, 4) rigid transforms
    sigma              : Gaussian decay parameter (default: max_dist / 3)

    Returns
    -------
    V_new : (N, 3) deformed vertex positions
    """
    N = len(V)
    G = len(gripper_transforms)

    if sigma is None:
        sigma = D_to_grippers.max() / 3.0

    # Compute per-gripper, per-vertex weights  w_g(m) = exp(-d(m,g)^2 / (2σ^2))
    weights = np.exp(-D_to_grippers ** 2 / (2.0 * sigma ** 2))  # (N, G)
    w_sum = weights.sum(axis=1, keepdims=True)
    w_sum[w_sum < 1e-12] = 1.0
    weights /= w_sum  # normalise so they sum to 1 per vertex

    # Apply weighted transforms
    Vh = np.hstack([V, np.ones((N, 1))])  # (N, 4)
    V_new = np.zeros((N, 3))

    for g in range(G):
        T = gripper_transforms[g]
        transformed = (T @ Vh.T).T[:, :3]  # (N, 3)
        V_new += weights[:, g:g+1] * transformed

    return V_new

"""Functional-map surface correspondences via spectral analysis.

Implements the ZoomOut algorithm (Melzi et al. 2019) for computing dense
point-to-point surface correspondences from Laplace-Beltrami eigenbases.

References:
    S. Melzi et al., "ZoomOut: Spectral Upsampling for Efficient Shape
    Correspondence", ACM Trans. on Graphics (SIGGRAPH Asia), 2019.
"""

import numpy as np
import robust_laplacian
from scipy.spatial import cKDTree
from scipy.sparse.linalg import eigsh


# ---------------------------------------------------------------------------
# Laplace-Beltrami eigenbasis
# ---------------------------------------------------------------------------

def compute_laplacian_eigenbasis(V: np.ndarray, F: np.ndarray, k: int = 50):
    """Compute the first *k* eigenvectors of the mesh Laplacian.

    Returns
    -------
    evals : (k,)
    evecs : (N, k)   — mass-orthonormal  (Φ^T M Φ = I)
    M     : sparse (N, N) mass matrix
    """
    L, M = robust_laplacian.mesh_laplacian(V, F)
    evals, evecs = eigsh(L, k=k, M=M, sigma=0, which="LM")
    idx = np.argsort(evals)
    return evals[idx], evecs[:, idx], M


# ---------------------------------------------------------------------------
# Heat Kernel Signature (HKS) — shape descriptor
# ---------------------------------------------------------------------------

def _hks(evals: np.ndarray, evecs: np.ndarray, n_t: int = 100) -> np.ndarray:
    """Compute Heat Kernel Signature descriptors.

    Parameters
    ----------
    evals : (k,)   eigenvalues (λ_0 … λ_{k-1})
    evecs : (N, k) eigenvectors
    n_t   : number of time samples

    Returns
    -------
    desc : (N, n_t)  HKS descriptor per vertex
    """
    # Skip the zero eigenvalue for numerical stability
    start = 1 if evals[0] < 1e-8 else 0
    evals_pos = evals[start:]
    evecs_pos = evecs[:, start:]
    k = len(evals_pos)

    t_min = 4 * np.log(10) / evals_pos[-1]
    t_max = 4 * np.log(10) / max(evals_pos[0], 1e-12)
    ts = np.geomspace(t_min, t_max, n_t)  # (n_t,)

    # HKS(x, t) = Σ_i exp(-λ_i t) φ_i(x)^2
    exp_term = np.exp(-evals_pos[np.newaxis, :] * ts[:, np.newaxis])  # (n_t, k)
    phi_sq = evecs_pos ** 2  # (N, k)
    desc = phi_sq @ exp_term.T  # (N, n_t)
    return desc


# ---------------------------------------------------------------------------
# Functional map utilities
# ---------------------------------------------------------------------------

def _p2p_from_fmap(C: np.ndarray, evecs_src: np.ndarray, evecs_tgt: np.ndarray) -> np.ndarray:
    """Convert a functional map C to a point-to-point map via NN in spectral embedding."""
    k = C.shape[1]
    projected = evecs_src[:, :k] @ C.T  # (N_src, k_tgt)
    tree = cKDTree(evecs_tgt[:, :C.shape[0]])
    _, T12 = tree.query(projected)
    return T12.astype(np.intp)


def _p2p_to_fmap(T12: np.ndarray, evecs_src: np.ndarray, evecs_tgt: np.ndarray,
                 k_src: int, k_tgt: int) -> np.ndarray:
    """Convert a point-to-point map to a functional map via least-squares.

    C = argmin_C ||C Φ_src - Φ_tgt(T12)|| ^2
      = Φ_tgt(T12)^+ Φ_src   →  C such that C @ Φ_src.T ≈ Φ_tgt(T12).T

    Returns C : (k_tgt, k_src)
    """
    Phi_src = evecs_src[:, :k_src]   # (N, k_src)
    Phi_tgt = evecs_tgt[T12, :k_tgt]  # (N, k_tgt)
    # C Φ_src^T = Φ_tgt(T12)^T  →  C = Φ_tgt(T12)^T Φ_src (Φ_src^T Φ_src)^-1
    # With mass-orthonormal basis: Φ^T Φ ≈ I (on same mesh), so:
    C = Phi_tgt.T @ Phi_src  # (k_tgt, k_src)
    return C


# ---------------------------------------------------------------------------
# ZoomOut refinement
# ---------------------------------------------------------------------------

def _zoomout_refine(
    evecs_src: np.ndarray,
    evecs_tgt: np.ndarray,
    C_init: np.ndarray,
    k_start: int,
    k_final: int,
    n_iters: int = 10,
) -> tuple[np.ndarray, np.ndarray]:
    """ZoomOut iterative refinement of a functional map.

    Starting from C_init of size (k_start, k_start), progressively increase
    the basis size up to k_final by alternating between point-to-point
    conversion and functional map recovery at the larger basis.

    Returns
    -------
    C    : (k_final, k_final) refined functional map
    T12  : (N_src,) point-to-point map
    """
    C = C_init
    ks = np.linspace(k_start, k_final, n_iters + 1, dtype=int)
    ks = np.unique(ks)

    for k_new in ks[1:]:
        # Current p2p from current C
        T12 = _p2p_from_fmap(C, evecs_src, evecs_tgt)
        # Recompute C at larger basis
        C = _p2p_to_fmap(T12, evecs_src, evecs_tgt, k_new, k_new)

    T12 = _p2p_from_fmap(C, evecs_src, evecs_tgt)
    return C, T12


# ---------------------------------------------------------------------------
# Initial functional map from descriptors (HKS)
# ---------------------------------------------------------------------------

def _initial_fmap_from_descriptors(
    evals_src: np.ndarray, evecs_src: np.ndarray,
    evals_tgt: np.ndarray, evecs_tgt: np.ndarray,
    k_init: int,
    desc_weight: float = 1.0,
    lap_weight: float = 1e-3,
) -> np.ndarray:
    """Compute an initial functional map C (k_init × k_init) using HKS descriptors.

    Solves:  min_C  ||C A_src - A_tgt||^2  +  μ ||ΔΛ C||^2

    where A_src, A_tgt are descriptor coefficients in the eigenbases and ΔΛ
    is the eigenvalue-commutativity regulariser.
    """
    # Project HKS descriptors into spectral bases
    desc_src = _hks(evals_src, evecs_src)  # (N_src, n_t)
    desc_tgt = _hks(evals_tgt, evecs_tgt)  # (N_tgt, n_t)

    Phi_src = evecs_src[:, :k_init]  # (N_src, k)
    Phi_tgt = evecs_tgt[:, :k_init]  # (N_tgt, k)

    # Spectral coefficients: A = Φ^T D  (with mass-orthonormal basis)
    A_src = Phi_src.T @ desc_src  # (k, n_t)
    A_tgt = Phi_tgt.T @ desc_tgt  # (k, n_t)

    # Build linear system:  C A_src ≈ A_tgt
    # With Laplacian commutativity regulariser: (λ_i^tgt - λ_j^src) C_ij ≈ 0
    ev_src = evals_src[:k_init]
    ev_tgt = evals_tgt[:k_init]

    # Descriptor term:  ||C A_src - A_tgt||^2
    # This is equivalent to solving C row-by-row, but we vectorise.
    # vec(C) minimises || (A_src^T ⊗ I_k) vec(C) - vec(A_tgt) ||^2
    k = k_init
    n_desc = A_src.shape[1]
    # Solve column-wise: for each descriptor d, C @ a_src_d ≈ a_tgt_d
    # Stack:  [a_src_1^T ⊗ I; ... ; a_src_D^T ⊗ I] vec(C) = [a_tgt_1; ...; a_tgt_D]
    # Equivalent to: C @ A_src = A_tgt  →  A_src^T C^T = A_tgt^T
    # Solve for each row of C:  c_i @ A_src = a_tgt_i  →  c_i = a_tgt_i @ pinv(A_src)
    # But add regularisation, so use the full least-squares form.

    # Descriptor energy matrix
    E_desc = desc_weight * (A_src @ A_src.T)  # (k, k)
    B_desc = desc_weight * (A_tgt @ A_src.T)  # (k, k)

    # Commutativity regulariser: encourage C diag(λ_src) ≈ diag(λ_tgt) C
    # → C_ij (λ_j^src - λ_i^tgt) = 0
    # Penalty: Σ_ij  μ (λ_j^src - λ_i^tgt)^2 C_ij^2
    # This makes the system diagonal per-element, so add it to the diagonal of the
    # Gram matrix.
    delta_lam = (ev_src[np.newaxis, :] - ev_tgt[:, np.newaxis]) ** 2  # (k, k)
    E_reg = lap_weight * delta_lam

    # Solve (E_desc + diag(E_reg_row)) c_i = b_desc_i  for each row i of C
    C = np.zeros((k, k))
    for i in range(k):
        lhs = E_desc + np.diag(E_reg[i, :])
        rhs = B_desc[i, :]
        C[i, :] = np.linalg.solve(lhs, rhs)

    return C


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def spectral_match(
    V_src: np.ndarray,
    F_src: np.ndarray,
    V_tgt: np.ndarray,
    F_tgt: np.ndarray,
    k: int = 30,
    k_init: int = 10,
    zoomout_iters: int = 15,
) -> np.ndarray:
    """Compute a point-to-point map from source to target using functional maps + ZoomOut.

    Pipeline:
    1. Compute Laplace-Beltrami eigenbases on both meshes (k eigenvectors).
    2. Compute HKS descriptors and solve for an initial functional map C (k_init × k_init).
    3. Refine with ZoomOut, progressively increasing basis size up to k.
    4. Extract final dense point-to-point map via nearest-neighbour in spectral embedding.

    Parameters
    ----------
    V_src, F_src : source mesh vertices (M, 3) and faces (F1, 3)
    V_tgt, F_tgt : target mesh vertices (N, 3) and faces (F2, 3)
    k            : maximum number of eigenvectors
    k_init       : initial basis size for descriptor-based functional map
    zoomout_iters : number of ZoomOut refinement iterations

    Returns
    -------
    T12 : (M,) int array — T12[i] is the index of the target vertex matched
          to source vertex i.
    """
    print(f"  Computing eigenbases (k={k}) …")
    evals_src, evecs_src, _ = compute_laplacian_eigenbasis(V_src, F_src, k=k)
    evals_tgt, evecs_tgt, _ = compute_laplacian_eigenbasis(V_tgt, F_tgt, k=k)

    # Sign-align eigenvectors using the orientation-preserving heuristic
    # (align to the sign that makes the sum positive, consistent across meshes)
    for j in range(k):
        if evecs_src[:, j].sum() < 0:
            evecs_src[:, j] *= -1
        if evecs_tgt[:, j].sum() < 0:
            evecs_tgt[:, j] *= -1

    print(f"  Computing initial functional map (k_init={k_init}) …")
    C_init = _initial_fmap_from_descriptors(
        evals_src, evecs_src, evals_tgt, evecs_tgt, k_init
    )

    print(f"  Running ZoomOut refinement ({zoomout_iters} iters, {k_init}→{k}) …")
    C_final, T12 = _zoomout_refine(
        evecs_src, evecs_tgt, C_init, k_start=k_init, k_final=k, n_iters=zoomout_iters
    )

    print(f"  Functional map quality — C off-diagonal energy: "
          f"{np.sum(C_final**2) - np.sum(np.diag(C_final)**2):.4f}")

    return T12


def nearest_vertex_map(V_src: np.ndarray, V_tgt: np.ndarray) -> np.ndarray:
    """Simple nearest-neighbour map in Euclidean space (fallback)."""
    tree = cKDTree(V_tgt)
    _, T12 = tree.query(V_src)
    return T12.astype(np.intp)

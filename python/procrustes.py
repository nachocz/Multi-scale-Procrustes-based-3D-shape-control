"""Orthogonal Procrustes analysis.

Solves  min_R  ||R(X - x̄) - (Y - ȳ)||_F   s.t. R ∈ SO(3)
and returns the rigid transform T ∈ SE(3) together with the Procrustes distance.
"""

import numpy as np
from numpy.linalg import svd, norm


def procrustes(X: np.ndarray, Y: np.ndarray):
    """Orthogonal Procrustes analysis (no scaling, no reflection).

    Parameters
    ----------
    X : (M, 3)  source points
    Y : (M, 3)  target points (column-wise matched to X)

    Returns
    -------
    d  : float        Procrustes distance  ||R(X-x̄) - (Y-ȳ)||_F
    T  : (4, 4)       rigid transform from X to Y  (homogeneous)
    X2 : (M, 3)       transformed source points  T @ X^h
    """
    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)

    mean_x = X.mean(axis=0)  # (3,)
    mean_y = Y.mean(axis=0)

    Xc = X - mean_x
    Yc = Y - mean_y

    # SVD of cross-covariance
    H = Yc.T @ Xc  # (3, 3)
    U, _S, Vt = svd(H)

    # Ensure proper rotation (det = +1)
    d = np.linalg.det(U) * np.linalg.det(Vt)
    M = np.diag([1.0, 1.0, d])
    R = U @ M @ Vt  # (3, 3)

    t = mean_y - R @ mean_x  # (3,)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    X2 = (R @ X.T).T + t  # (M, 3)
    dp = norm(X2 - Y)

    return dp, T, X2

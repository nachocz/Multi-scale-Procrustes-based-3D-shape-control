"""SE(3) geodesic-path interpolation.

Given two rigid transforms H0 and H1, computes the interpolated transform at
parameter t ∈ [0, 1] along the geodesic in SE(3).
"""

import numpy as np
from scipy.linalg import logm


def rigid_transform_geodesic_path(
    t: float,
    H0: np.ndarray,
    H1: np.ndarray,
) -> np.ndarray:
    """Interpolate along the SE(3) geodesic from H0 towards H1.

    Parameters
    ----------
    t  : interpolation parameter (0 → H0, 1 → H1)
    H0 : (4, 4)  start rigid transform
    H1 : (4, 4)  end rigid transform

    Returns
    -------
    Tout : (4, 4)  interpolated rigid transform
    """
    # Relative transform  H = H1 · H0⁻¹
    R0 = H0[:3, :3]
    t0 = H0[:3, 3]
    H0_inv = np.eye(4)
    H0_inv[:3, :3] = R0.T
    H0_inv[:3, 3] = -R0.T @ t0

    H = H1 @ H0_inv

    R = H[:3, :3]
    p = H[:3, 3]

    S = logm(R).real  # skew-symmetric (3×3)

    s0, s1, s2 = S[2, 1], S[0, 2], S[1, 0]
    theta = np.sqrt(s0 * s0 + s1 * s1 + s2 * s2)
    theta_sq = theta * theta

    TOL = 1e-5

    # Compute inv(V) where V is from the exponential map
    if theta > TOL:
        c = (1.0 - (theta * np.sin(theta)) / (2.0 * (1.0 - np.cos(theta)))) / theta_sq
        invV = np.eye(3) - 0.5 * S + c * S @ S
    else:
        invV = np.eye(3)

    U = invV @ p

    angle = t * theta

    if theta > TOL:
        interpR = (
            np.eye(3)
            + (np.sin(angle) / theta) * S
            + ((1.0 - np.cos(angle)) / theta_sq) * S @ S
        )
    else:
        interpR = np.eye(3)

    if theta > TOL:
        theta_cub = theta * theta_sq
        c0 = (1.0 - np.cos(angle)) / theta_sq
        c1 = (angle - np.sin(angle)) / theta_cub
        interpT_timesV = t * np.eye(3) + c0 * S + c1 * S @ S
    else:
        interpT_timesV = t * np.eye(3)

    Tout = np.eye(4)
    Tout[:3, :3] = interpR @ R0
    Tout[:3, 3] = interpR @ t0 + interpT_timesV @ U

    return Tout

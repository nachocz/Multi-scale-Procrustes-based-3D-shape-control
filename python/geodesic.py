"""Geodesic distance computation and iso-distance segmentation on triangle meshes."""

import numpy as np
import scipy.sparse as sp
from scipy.sparse.csgraph import shortest_path


def mesh_graph_distances(V: np.ndarray, F: np.ndarray) -> sp.csr_matrix:
    """Build a sparse weighted adjacency matrix from edge lengths.

    Returns the dense shortest-path distance matrix (geodesic approximation).
    """
    edges = np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    edges = np.sort(edges, axis=1)
    edges = np.unique(edges, axis=0)

    weights = np.linalg.norm(V[edges[:, 0]] - V[edges[:, 1]], axis=1)

    n = len(V)
    W = sp.csr_matrix((weights, (edges[:, 0], edges[:, 1])), shape=(n, n))
    W = W + W.T  # symmetrise

    return W


def geodesic_distances(V: np.ndarray, F: np.ndarray) -> np.ndarray:
    """Approximate geodesic distance matrix via Dijkstra on the mesh graph.

    Returns
    -------
    D : (N, N)  dense distance matrix
    """
    W = mesh_graph_distances(V, F)
    D = shortest_path(W, directed=False)
    # Replace any remaining inf with Euclidean fallback
    inf_mask = np.isinf(D)
    if inf_mask.any():
        from scipy.spatial.distance import cdist
        D_euc = cdist(V, V)
        D[inf_mask] = D_euc[inf_mask]
    return D


def compute_iso_distance_segments(
    V: np.ndarray,
    F: np.ndarray,
    scale_vec: np.ndarray,
    gripper_indices: np.ndarray,
    D: np.ndarray,
):
    """Compute cumulative geodesic-ball membership masks for each gripper and scale.

    Parameters
    ----------
    V             : (N, 3)  vertex positions
    F             : (F, 3)  triangle faces
    scale_vec     : (S,)    array of scale (radius) values
    gripper_indices : (G,)  vertex indices of grippers
    D             : (N, N)  distance matrix

    Returns
    -------
    masks : (N, S, G)  boolean masks — masks[m, s, g] is True if vertex m lies
            within geodesic distance scale_vec[s] from gripper g.
    """
    N = len(V)
    S = len(scale_vec)
    G = len(gripper_indices)

    masks = np.zeros((N, S, G), dtype=bool)
    for g_idx, g in enumerate(gripper_indices):
        dists = D[:, g]  # (N,)
        for s_idx, r in enumerate(scale_vec):
            masks[:, s_idx, g_idx] = dists <= r
            masks[g, s_idx, g_idx] = True  # gripper always included

    return masks

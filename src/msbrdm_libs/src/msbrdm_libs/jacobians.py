import sympy as sym
from src.msbrdm_libs.src.msbrdm_libs.forward_kinematics import vec_to_skew, exp_matrix
from functools import lru_cache
import numpy as np
def adjoint_mapping(T: sym.Matrix) -> sym.Matrix:
    """Compute the adjoint mapping of a transformation matrix

    Args:
        T (sym.Matrix): Transformation matrix (4x4)

    Returns:
        sym.Matrix: Adjoint mapping of the transformation matrix
    """
    R = T[:3, :3]
    p = T[:3, 3]
    p_R = vec_to_skew(p) @ R
    adj_matrix = sym.eye(6)
    adj_matrix[:3, :3] = R
    adj_matrix[3:, 3:] = R
    adj_matrix[3:, :3] = p_R
    return cached_simplify(adj_matrix)


def T_inv(T: sym.Matrix) -> sym.Matrix:
    """Invervse of a transformation matrix

    Args:
        T (sym.Matrix): homogeneous transformation matrix (4x4)

    Returns:
        sym.Matrix: inverse homogeneous transformation matrix (4x4)
    """
    T_inv = sym.eye(4)
    rotation_inverse = T[:3, :3].transpose()
    T_inv[:3, :3] = rotation_inverse
    T_inv[:3, 3] = -rotation_inverse @ T[:3, 3]
    return T_inv

def space_jacobian(SList: sym.Matrix, q: sym.Matrix) -> sym.Matrix:
    """Compute the space jacobian matrix

    Args:
        SList (sym.Matrix): List of screw axes (6 x n)
        q (sym.Matrix): Joint positions (n x 1)

    Returns:
        sym.Matrix: Space jacobian matrix (6 x n)
    """
    n = len(q)
    Js = sym.zeros(6, n)
    T = sym.eye(4)
    Js[:, 0] = SList[:, 0]
    for i in range(1, len(SList[0, :])):
        # Compute transform for joints 1 ... i-1
        T = cached_simplify(T * cached_simplify(exp_matrix(SList[:, i-1], q[i-1])))
        Js[:, i] = cached_simplify(adjoint_mapping(T) * SList[:, i])
    return cached_simplify(Js)

@lru_cache(maxsize=2000)
def cached_simplify_immutable(exp, trig=False):
    try:
        #return sym.trigsimp(exp) if trig else sym.simplify(exp)
        expr = sym.expand(exp)
        expr = sym.trigsimp(expr)
        return sym.factor(expr)
    except RecursionError as e:
        print("Simplification blew up due to deep recursion...")
        return exp

def cached_simplify(M):
    if not isinstance(M, sym.ImmutableMatrix):
        M = sym.ImmutableMatrix(M)
    return cached_simplify_immutable(M)

def body_jacobian(SList: sym.Matrix, q: sym.Matrix, T_b_s: sym.Matrix) -> sym.Matrix:
    """Compute body jacobian from space jacobian

    Args:
        Js (sym.Matrix): space jacobian matrix (6 x n)
        T_b_s (sym.Matrix): Transformation matrix of body w.r.t. space frame

    Returns:
        sym.Matrix: body jacobian matrix (6 x n)
    """
    Jb = adjoint_mapping(T_inv(T_b_s)) @ space_jacobian(SList, q)
    return cached_simplify(Jb)


def body_jacobian_space_aligned(SList: sym.Matrix, q: sym.Matrix, T_b_s: sym.Matrix) -> sym.Matrix:
    """Body jacobian aligned with the space frame

    Args:
        Jb (sym.Matrix): Body jacobian matrix
        T_b_s (sym.Matrix): Transformation matrix of body w.r.t. space frame

    Returns:
        sym.Matrix: body jacobian matrix aligned with the space frame
    """
    R_b_s = T_b_s[:3, :3]       
    rotation = sym.zeros(6, 6)
    rotation[:3, :3] = R_b_s
    rotation[3:, 3:] = R_b_s
    Jb_s = rotation @ body_jacobian(SList, q, T_b_s)
    return Jb_s

def body_jacobian_space_aligned_cross(SList: sym.Matrix, q: sym.Matrix, T_b_s: sym.Matrix) -> sym.Matrix:
    """Body jacobian aligned with the space frame

    Args:
        Jb (sym.Matrix): Body jacobian matrix
        T_b_s (sym.Matrix): Transformation matrix of body w.r.t. space frame

    Returns:
        sym.Matrix: body jacobian matrix aligned with the space frame
    """
    t = T_b_s[:3, 3]
    skew = vec_to_skew(-t)
    matrix = sym.eye(6)
    matrix[3:, :3] = skew
    Jb_s = cached_simplify(matrix) @ space_jacobian(SList, q)
    return cached_simplify(Jb_s)
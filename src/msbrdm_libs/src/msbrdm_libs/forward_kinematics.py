import sympy as sym
from functools import lru_cache

def vec_to_skew(v: sym.Matrix) -> sym.Matrix:
    """Compute the skew-symmetric matrix of a vector

    Args:
        v (sym.Matrix): 3x1 vector

    Returns:
        sym.Matrix: skew-symmetric matrix form of the vector [v]
    """

    # TODO: Implement the skew-symmetric matrix of a vector
    
    mat = sym.Matrix([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])
    return mat

def rodrigues_formula(omega: sym.Matrix, theta: sym.Matrix) -> sym.Matrix:
    """Compute the rotation matrix using Rodrigues' formula

    Args:
        omega (sym.Matrix): rotation axis
        theta (sym.Matrix): rotation angle

    Returns:
        sym.Matrix: rotation matrix
    """
    # TODO: Implement the rotation matrix using Rodrigues' formula
    skew = vec_to_skew(omega)
    R = sym.eye(3) + sym.sin(theta) * skew + (1 - sym.cos(theta)) * skew @ skew
    return R


def exp_matrix(Si: sym.Matrix, qi: sym.Matrix) -> sym.Matrix:
    """Transformation of joint i using the exponential formula: T = exp([Si]*qi)

    Args:
        Si (sym.Matrix): screw axis of joint i
        qi (sym.Matrix): displacement of joint i

    Returns:
        sym.Matrix: transformation matrix
    """
   
    # TODO: Implement the transformation matrix using the exponential formula
    w = Si[0:3, 0]
    v = Si[3:6, 0]
    w_skew = vec_to_skew(w)
    

    R = rodrigues_formula(w, qi)
    p = qi * v + (1 - sym.cos(qi)) * w_skew * v + (qi - sym.sin(qi)) * w_skew * w_skew * v
    
    T = sym.Matrix.vstack(
        sym.Matrix.hstack(R, p),
        sym.Matrix([[0, 0, 0, 1]])
    )
    return T


def forward_kinematics(SList: sym.Matrix, M: sym.Matrix, q: sym.Matrix) -> sym.Matrix:
    """Product of exponentials for forward kinematics
    T = exp([S1]*q1) * exp([S2]*q2) * ... * exp([Sn]*qn) * M

    Args:
        SList (sym.Matrix): List of screw axes
        M (sym.Matrix): Initial configuration
        q (sym.Matrix): joint displacements (nq x 1)

    Returns:
        sym.Matrix: transformation matrix
    """

    # TODO: Implement the product of exponentials for forward kinematics
    acc = exp_matrix(SList[:, 0], q[0])
    for i in range(1, len(SList[0, :])):
        acc = cached_simplify(acc @ exp_matrix(SList[:, i], q[i]))
    
    return cached_simplify(acc @ M)

@lru_cache(maxsize=2024)
def cached_symplify_immutable(expr):
    return sym.simplify(expr)

def cached_simplify(M):
    if not isinstance(M, sym.ImmutableMatrix):
        M = sym.ImmutableMatrix(M)
    return cached_symplify_immutable(M)
import sympy as sym
import numpy as np
from typing import List, Tuple
from functools import lru_cache
import pickle

SIMPLIFY = True
def dynamic_model(T_cm: List[sym.Matrix], J_cm: List[sym.Matrix]) -> Tuple[sym.Matrix, sym.Matrix, sym.Matrix]:
    """Calculate the M, C, G matrices for the robot dynamics.

    Args:
        T_cm (List[sym.Matrix]): List of the transformation matrices of all centers of mass
        J_cm (List[sym.Matrix]): List of the Jacobian matrices of all centers of mass

    Returns:
        Tuple[sym.Matrix, sym.Matrix, sym.Matrix]: M, C, G matrices
    """
    M = M_matrix(T_cm, J_cm)
    print("M")
    with open("M_matrix_pure_cross.pkl", "wb") as f:
        pickle.dump(M, f)

    C = C_matrix(M)
    print("C")
    with open("C_matrix_pure_cross.pkl", "wb") as f:
        pickle.dump(C, f)
    # with open("C_matrix.pkl", "rb") as f:
    #     C = pickle.load(f)

    G = G_vector(T_cm)
    print("G")
    with open("G_matrix_pure_cross.pkl", "wb") as f:
        pickle.dump(G, f)
    print(verify_dynamics(M, C, G), "verify")
    return M, C, G


def generate_I(n: int) -> List[sym.Matrix]:
    """Generate a list of inertia matrices for each link

    Args:
        n (int): number of links

    Returns:
        sym.Matrix: I[n][3x3] inertia matrix for each link
        For each I[k] = [[Ik00, Ik01, Ik02],
                         [Ik01, Ik11, Ik12],
                         [Ik02, Ik12, Ik22]]
    """
    I = []
    for k in range(0, n):
        I_k = sym.zeros(3, 3)
        for i in range(0, 3):
            for j in range(i, 3):
                I_k[i, j] = sym.symbols('I'+str(k)+str(i)+str(j), real=True, positive=True)
                I_k[j, i] = I_k[i, j]
        I.append(I_k)
    return I

def generate_inertias():
    I1 = np.array([
        [0.00610633, 0.        , 0.        ],
       [0.        , 0.00610633, 0.        ],
       [0.        , 0.        , 0.01125   ]
       ])

    I2 = np.array([
        [0.03147431, 0.        , 0.        ],
       [0.        , 0.03147431, 0.        ],
       [0.        , 0.        , 0.02187562]
       ])

    I3 = np.array([
        [0.4217538 , 0.        , 0.        ],
       [0.        , 0.4217538 , 0.        ],
       [0.        , 0.        , 0.03636562]
       ])

    I4 = np.array([
        [0.11106969, 0.        , 0.        ],
       [0.        , 0.11106969, 0.        ],
       [0.        , 0.        , 0.01088437]
       ])

    I5 = np.array([
        [0.00510825, 0.        , 0.        ],
       [0.        , 0.00510825, 0.        ],
       [0.        , 0.        , 0.0055125 ]
       ])

    I6 = np.array([
        [0.00510825, 0.        , 0.        ],
       [0.        , 0.00510825, 0.        ],
       [0.        , 0.        , 0.0055125 ]
       ])

    return [I1, I2, I3, I4, I5, I6]

def generate_symbol_maps(n):
    sym_map = {}
    I_sym = generate_I(n=n)
    I_num = generate_inertias()
    m_num = np.array([7.778, 12.93, 3.87, 1.96, 1.96, 0.202])
    m_sym = sym.symbols('m0:' + str(n), real=True, positive=True)
    for k in range(0, n):
        for i in range(0, 3):
            for j in range(i, 3):
                sym_map[I_sym[k][i, j]] = I_num[k][i, j]

    for k in range(0, n):
        sym_map[m_sym[k]] = m_num[k]
    return sym_map
def M_matrix(T_cm: List[sym.Matrix], J_cm: List[sym.Matrix]) -> sym.Matrix:
    """Inertia matrix 
    Args:
        T_cm (List[sym.Matrix]): List of the transformation matrices of all centers of mass
        J_cm (List[sym.Matrix]): List of the Jacobian matrices of all centers of mass

    Returns:
        sym.Matrix: M Matrix [n x n]
    """

    # numer of joints
    n = len(T_cm)

    # symbolic variables
    #m = np.array([7.778, 12.93, 3.87, 1.96, 1.96, 0.202])
    m = sym.symbols('m0:' + str(n), real=True, positive=True)
    I = generate_I(n)
    

    # TODO: calculate M Matrix
    M = sym.zeros(n, n)
    for i in range(n):
        J_cm_i = J_cm[i]
        T_cm_i = T_cm[i]
        print(i, "M")
        M += symplify_matrix(m[i] * get_translation_jacobian(J_cm_i).T @ get_translation_jacobian(J_cm_i) + get_rotation_jacobian(J_cm_i).T @ get_rotation_matrix(T_cm_i) @ I[i] @ get_rotation_matrix(T_cm_i).T @ get_rotation_jacobian(J_cm_i))
    return symplify_matrix(M)

@lru_cache(maxsize=2000)
def cached_symplify(exp, trig=False):
    if not SIMPLIFY:
        return exp
    try:
        #return sym.trigsimp(exp) if trig else sym.simplify(exp)
        expr = sym.expand(exp)
        expr = sym.trigsimp(expr)
        return sym.factor(expr)
    except RecursionError as e:
        print("Simplification blew up due to deep recursion...")
        return exp

def symplify_matrix(matrix):
    rows, cols = matrix.shape
    for i in range(rows):
        for j in range(cols):
            matrix[i, j] = cached_symplify(matrix[i, j])
            print(f"{i * rows + j} / {rows * cols}")
            print(cached_symplify.cache_info())
    return matrix

def get_rotation_matrix(T):
    return sym.Matrix(T[:3, :3])

def get_translation_jacobian(J):
    return sym.Matrix(J[3:, :])

def get_rotation_jacobian(J):
    return sym.Matrix(J[:3, :])

def C_matrix(M: sym.Matrix) -> sym.Matrix:
    """Coriolis and Centripetal matrix using Christoffel symbols

    Args:
        M (sym.Matrix): Inertia matrix

    Returns:
        sym.Matrix: C Matrix [n x n]
    """
    n = M.shape[0]

    # symbolic variables
    q = sym.symbols('q(0:' + str(n) + ')', real=True)
    qdot = sym.symbols('q(0:' + str(n) + ')dot', real=True)

    # TODO: calculate C Matrix
    C = sym.zeros(n, n)
    for k in range(n):
        for j in range(n):
            for i in range(n):
                print(f"{k * n * n + j * n + i} / {n * n * n}", "C")
                C[k, j] += 1/2 * (
                    cached_symplify(sym.diff(M[k, j], q[i]), trig=True) +
                    cached_symplify(sym.diff(M[k, i], q[j]), trig=True) -
                    cached_symplify(sym.diff(M[i, j], q[k]), trig=True)
                        ) * qdot[i]
                print(cached_symplify.cache_info())
    return symplify_matrix(C)


def G_vector(T_cm: List[sym.Matrix]) -> sym.Matrix:
    """Gravity vector

    Args:
        T_cm (List[sym.Matrix]): List of the transformation matrices of all centers of mass

    Returns:
        sym.Matrix: G Vector [n x 1]
    """
    n = len(T_cm)

    # symbolic variables
    q = sym.symbols('q(0:' + str(n) + ')', real=True)
    #m = np.array([7.778, 12.93, 3.87, 1.96, 1.96, 0.202])
    m = sym.symbols('m0:' + str(n), real=True, positive=True)
    g = sym.Matrix([0, 0, 9])

    # TODO: calculate G Vector
    G = sym.zeros(n, 1)

    p = 0
    for i in range(n):
        p += (m[i] * g.T @ T_cm[i][:3, 3])[0]
    for i in range(n):
        G[i] = sym.diff(p, q[i])
    #return G
    return sym.simplify(G)


def M_dot(M: sym.Matrix) -> sym.Matrix:
    """Time derivative of the inertia matrix

    Args:
        M (sym.Matrix): Inertia matrix

    Returns:
        sym.Matrix: Mdot Matrix [n x n]
    """
    n = M.shape[0]

    q = sym.symbols('q(0:'+ str(n) +')', real=True)
    qdot = sym.symbols('q(0:'+str(n)+')dot', real=True)

    # TODO: calculate Mdot Matrix
    Mdot = sym.zeros(n, n)
    for i in range(n):
        Mdot += M.diff(q[i]) * qdot[i]
    return sym.simplify(sym.expand(Mdot))


def verify_dynamics(M: sym.Matrix, C: sym.Matrix, G: sym.Matrix) -> bool:
    """Verify the dynamics matrices M, C, G by substituting random joint values
    and compare the numerical np.array results.

    Args:
        M (sym.Matrix): Inertia matrix
        C (sym.Matrix): Coriolis and Centripetal matrix
        G (sym.Matrix): Gravity vector

    Note: the input matrices should only contain symbolic joint variables, 
    other parameters (m, I, L, g) should be substituted before calling this function.
    The reason why we don't directly varify the symbolic matrices is because the
    simplify() function becomes very slow for large matrices.

    Returns:
        bool: True if the matrices are correct, False otherwise
    """
    n = M.shape[0]
    q = sym.symbols('q(0:'+str(n)+')', real=True)
    qdot = sym.symbols('q(0:'+str(n)+')dot', real=True)
    qddot = sym.symbols('q(0:'+str(n)+')ddot', real=True)

    # verify with random q, qdot values
    q_val = np.random.rand(n)
    qdot_val = np.random.rand(n)

    q_map = {q[i]: q_val[i] for i in range(n)}
    qdot_map = {qdot[i]: qdot_val[i] for i in range(n)}
    joint_maps = {**q_map, **qdot_map}
    M_val = sym.simplify(M.subs(q_map))
    if not M_val.is_symmetric():
        print(M_val)
        return False
    C_val = C.subs(joint_maps)
    N = M_dot(M).subs(joint_maps) - 2 * C_val
    print("starting simp N")
    if not (N == -N.T):
        return False
    return True

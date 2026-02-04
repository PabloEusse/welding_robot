import sympy as sym
import numpy as np
import src.msbrdm_libs.src.msbrdm_libs.forward_kinematics as FK
import src.msbrdm_libs.src.msbrdm_libs.jacobians as jc
import src.msbrdm_libs.src.msbrdm_libs.dynamics as dn
import src.msbrdm_libs.src.msbrdm_libs.regressor as rg
import pickle
class UR10Model3DOF:
    def __init__(self, pure_symbolic: bool = False, load: bool = False) -> None:
        self.pure_symbolic = pure_symbolic  
        self.load = load
        self.config_joints(nq=6)
        self.config_params()
        #self.kinematics()
        #self.jacobians()
        self.dynamics()
    def config_joints(self, nq: int) -> None:
        """Define symbolic variables for robot joints

        Args:
            nq (int): number of joints
        """
        # number of joints
        self.nq = nq
        # define joint symbols
        self.q = sym.symbols('q(0:'+str(self.nq)+')', real=True)
        self.qdot = sym.symbols('q(0:'+str(self.nq)+')dot', real=True)
        self.qddot = sym.symbols('q(0:'+str(self.nq)+')ddot', real=True)

    def config_params(self) -> None:
        """ Initialize robot parameters
        """
        # save robot parameters as a dictionary
        self.params_dict = {'L1': 0.128, 'L2': 0.6127, 'L3': 0.5716, 
                            'L4': 0.1157, 'L5': 0.1639, 'L6': 0.0922,
                            'L7': 0.1, 'L8': 0.15, 'L9': 0.085, 'L10': 0.6127/2, 'L11': (0.1157 + 0.5716) / 2}
        # convert dict to list of tuples for later substitution
        self.params_list = [(k, v) for k, v in self.params_dict.items()]
        self.D = np.diag([5, 5, 5, 1, 1, 1])    

    def kinematics(self) -> None:
        """Symbolic analysis of forward kinematics
        """
        ########################################################################
        # Robot parameters
        ########################################################################
        if self.pure_symbolic:
            L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11 = sym.symbols('L1:12', positive=True, real=True)
        else:
            L1, L2, L3, L4, L5, L6, L7,  L8, L9 = [self.params_dict[k] for k in [
                'L1', 'L2', 'L3', 'L4', 'L5', 'L6', 'L7', 'L8', 'L9']]
            L10 = L2/2
            L11 = (L3 + L4) / 2     
            

        ########################################################################
        # Base transformation
        ########################################################################
        # TODO: Define the transformation matrix of the base frame (0) w.r.t. world frame (w)
        self.T_0_w = sym.Matrix([
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        ########################################################################
        # Screw axes
        ########################################################################
        # TODO: Treat the {0} frame as spatial frame. Define the screw axes of 
        # the robot in a sym.Matrix type
        # S: [w, v].T (6 x n)
        w1 = sym.Matrix([0, 0, 1])
        v1 = -FK.vec_to_skew(w1) @ sym.Matrix([0, 0, L1])
        w2 = sym.Matrix([0, 1, 0])
        v2 = -FK.vec_to_skew(w2) @ sym.Matrix([0, 0, L1])
        w3 = sym.Matrix([0, 1, 0])
        v3 = -FK.vec_to_skew(w3) @ sym.Matrix([0, 0, L1 + L2])
        w4 = sym.Matrix([0, 1, 0])
        v4 = -FK.vec_to_skew(w4) @ sym.Matrix([0, 0, L1 + L2 + L3])
        w5 = sym.Matrix([0, 0, 1])
        v5 = -FK.vec_to_skew(w5) @ sym.Matrix([0 , L5, L1 + L2 + L3])
        w6 = sym.Matrix([0, 1, 0])
        v6 = -FK.vec_to_skew(w6) @ sym.Matrix([0, L5 + L6, L1 + L2 + L3 + L4])

        omega_list = sym.Matrix([
            w1.T, w2.T, w3.T, w4.T, w5.T, w6.T
        ]).T
        v_list = sym.Matrix([
            v1.T, v2.T, v3.T, v4.T, v5.T, v6.T
        ]).T
        self.SList = sym.Matrix.vstack(omega_list, v_list)
        ########################################################################
        # Home Transformation
        ########################################################################
        t_ee_0 = jc.T_inv(self.T_0_w) @ sym.Matrix([0, L5 + L6, L1 + L2 + L3 + L4, 1])
        self.M_ee_0 = sym.eye(4)
        self.M_ee_0[:, 3] = t_ee_0

        ########################################################################
        # Forward kinematics
        ########################################################################
        # TODO: Call the fuction implemented in msbrdm_libs/forward_kinematics.py
        # to compute the end-effector frame in {0}
        self.sym_q = self.q
        M_ee_w = self.T_0_w @ self.M_ee_0
        self.T_ee_0 = jc.T_inv(self.T_0_w) @ FK.forward_kinematics(self.SList, M_ee_w, self.sym_q)
        
        M_cm6_w = sym.eye(4)
        M_cm6_w[:, 3] = sym.Matrix([0, L5 + L6, L1 + L2 + L3 + L4, 1])
        self.T_cm_6_w = FK.forward_kinematics(self.SList[:, :6], M_cm6_w, self.sym_q)

        M_6_w = sym.eye(4)
        M_6_w[:, 3] = sym.Matrix([0, L5 + L6, L1 + L2 + L3 + L4, 1])
        self.T_6_w = FK.forward_kinematics(self.SList[:, :6], M_6_w, self.sym_q)
        
        M_cm5_w = sym.eye(4)
        M_cm5_w[:, 3] = sym.Matrix([0, L5 , L1 + L2 + L3 + L4, 1])
        self.T_cm_5_w = FK.forward_kinematics(self.SList[:, :5], M_cm5_w, self.sym_q)

        M_5_w = sym.eye(4)
        M_5_w[:, 3] = sym.Matrix([0 , L5, L1 + L2 + L3, 1])
        self.T_5_w = FK.forward_kinematics(self.SList[:, :5], M_5_w, self.sym_q)

        M_cm4_w = sym.eye(4)
        M_cm4_w[:, 3] = sym.Matrix([0, L5, L1 + L2 + L3 + L4 / 2, 1])
        self.T_cm_4_w = FK.forward_kinematics(self.SList[:, :4], M_cm4_w, self.sym_q)

        M_4_w = sym.eye(4)
        M_4_w[:, 3] = sym.Matrix([0, 0, L1 + L2 + L3, 1])
        self.T_4_w = FK.forward_kinematics(self.SList[:, :4], M_4_w, self.sym_q)

        M_cm3_w = sym.eye(4)
        M_cm3_w[:, 3] = sym.Matrix([0, L8 - L9, L1 + L2 + L11, 1])
        self.T_cm_3_w = FK.forward_kinematics(self.SList[:, :3], M_cm3_w, self.sym_q)

        M_3_w = sym.eye(4)
        M_3_w[:, 3] = sym.Matrix([0, L8 - L9, L1 + L2, 1])
        self.T_3_w = FK.forward_kinematics(self.SList[:, :3], M_3_w, self.sym_q)

        M_cm2_w = sym.eye(4)
        M_cm2_w[:, 3] = sym.Matrix([0, L8, L1 + L10, 1])
        self.T_cm_2_w = FK.forward_kinematics(self.SList[:, :2], M_cm2_w, self.sym_q)

        M_2_w = sym.eye(4)
        M_2_w[:, 3] = sym.Matrix([0, L8, L1, 1])
        self.T_2_w = FK.forward_kinematics(self.SList[:, :2], M_2_w, self.sym_q)

        M_cm1_w = sym.eye(4)
        M_cm1_w[:, 3] = sym.Matrix([0, 0, L7, 1])
        self.T_cm_1_w = FK.forward_kinematics(self.SList[:, :1], M_cm1_w, self.sym_q)

        self.T_1_w = self.T_cm_1_w
        # TODO: Calculate the end-effector transforamations in the world frame
        self.T_ee_w = FK.forward_kinematics(self.SList, M_ee_w, self.sym_q)

        if self.pure_symbolic:
            self.Tsym_ee_0 = self.T_ee_0.copy()
            self.Tsym_ee_w = self.T_ee_w.copy()
            self.T_ee_0 = self.T_ee_w.subs(self.params_list)
            self.T_ee_w = self.T_ee_w.subs(self.params_list)
        
        ########################################################################
        # Generate functions for numerical evaluation
        ########################################################################
        # TODO: Generate transformation functions of joint poisition T(q) for
        # numerical updates, use sym.lambdify() function
        self.T_cm6_w_fn = sym.lambdify(self.sym_q, self.T_cm_6_w)
        self.T_cm5_w_fn = sym.lambdify(self.sym_q, self.T_cm_5_w)
        self.T_cm4_w_fn = sym.lambdify(self.sym_q, self.T_cm_4_w)
        self.T_cm3_w_fn = sym.lambdify(self.sym_q, self.T_cm_3_w)
        self.T_cm2_w_fn = sym.lambdify(self.sym_q, self.T_cm_2_w)
        self.T_cm1_w_fn = sym.lambdify(self.sym_q, self.T_cm_1_w)
        self.T_6_w_fn = sym.lambdify(self.sym_q, self.T_6_w)
        self.T_5_w_fn = sym.lambdify(self.sym_q, self.T_5_w)
        self.T_4_w_fn = sym.lambdify(self.sym_q, self.T_4_w)
        self.T_3_w_fn = sym.lambdify(self.sym_q, self.T_3_w)
        self.T_2_w_fn = sym.lambdify(self.sym_q, self.T_2_w)
        self.T_1_w_fn = sym.lambdify(self.sym_q, self.T_1_w)
        self.T_ee_0_fn = sym.lambdify(self.sym_q, self.T_ee_0)
        self.T_ee_w_fn = sym.lambdify(self.sym_q, self.T_ee_w)
        print("kinematics done")
    def jacobians(self) -> None:
        """Symbolic analysis of Jacobians
        """


        self.J_cm1_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :1], self.sym_q, self.T_cm_1_w)
        self.J_cm2_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :2], self.sym_q, self.T_cm_2_w)
        self.J_cm3_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :3], self.sym_q, self.T_cm_3_w)
        self.J_cm4_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :4], self.sym_q, self.T_cm_4_w)
        self.J_cm5_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :5], self.sym_q, self.T_cm_5_w)
        self.J_cm6_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :6], self.sym_q, self.T_cm_6_w)
        self.J_ee_w = jc.body_jacobian_space_aligned_cross(self.SList, self.sym_q, self.T_ee_w)

        self.J_1_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :1], self.sym_q, self.T_1_w)
        self.J_2_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :2], self.sym_q, self.T_2_w)
        self.J_3_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :3], self.sym_q, self.T_3_w)
        self.J_4_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :4], self.sym_q, self.T_4_w)
        self.J_5_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :4], self.sym_q, self.T_5_w)
        self.J_6_w = jc.body_jacobian_space_aligned_cross(self.SList[:, :6], self.sym_q, self.T_6_w)

        self.J_1_w_fn = sym.lambdify(self.sym_q, self.J_1_w)
        self.J_2_w_fn = sym.lambdify(self.sym_q, self.J_2_w)
        self.J_3_w_fn = sym.lambdify(self.sym_q, self.J_3_w)
        self.J_4_w_fn = sym.lambdify(self.sym_q, self.J_4_w)
        self.J_5_w_fn = sym.lambdify(self.sym_q, self.J_5_w)
        self.J_6_w_fn = sym.lambdify(self.sym_q, self.J_6_w)
        self.J_ee_w_fn =  sym.lambdify(self.sym_q, self.J_ee_w)

    def dynamics(self):
        params_sym = {sym.symbols(k, positive=True, real=True): v for k, v in self.params_dict.items()}
        params_sym.update(dn.generate_symbol_maps(self.nq))
        if self.load:
            with open("/Users/pablo.eusse/PycharmProjects/pablo-martinez-ws/src/robot_models/ur_model/src/ur10_model/M_matrix_pure_cross.pkl", "rb") as f:
                M_sym = pickle.load(f)
            with open("/Users/pablo.eusse/PycharmProjects/pablo-martinez-ws/src/robot_models/ur_model/src/ur10_model/C_matrix_pure_cross.pkl", "rb") as f:
                C_sym = pickle.load(f)
            with open("/Users/pablo.eusse/PycharmProjects/pablo-martinez-ws/src/robot_models/ur_model/src/ur10_model/G_matrix_pure_cross.pkl", "rb") as f:
                G_sym = pickle.load(f)
            with open("/Users/pablo.eusse/PycharmProjects/pablo-martinez-ws/src/robot_models/ur_model/src/ur10_model/Y.pkl", "rb") as f:
                Y = pickle.load(f)
            # with open("Jdot_matrix.pkl", "rb") as f:
            #     self.jd_ee_w = pickle.load(f)
        else:
            T_cm = [self.T_cm_1_w, self.T_cm_2_w, self.T_cm_3_w, self.T_cm_4_w, self.T_cm_5_w, self.T_cm_6_w, ]
            J_cm = [self.J_cm1_w, self.J_cm2_w, self.J_cm3_w, self.J_cm4_w, self.J_cm5_w, self.J_cm6_w, ]   
            M_sym, C_sym, G_sym = dn.dynamic_model(T_cm, J_cm)
            Jdot = sym.zeros(*self.J_ee_w.shape)
            Y, theta, Y_fn, theta_fn, parameters = rg.regressor(M_sym, C_sym, G_sym, params_sym.keys(), reference=False)
            for i in range(len(self.sym_q)):
                Jdot += self.J_ee_w.diff(self.sym_q[i]) * self.qdot[i]
                Jdot = sym.simplify(Jdot)
                print(i)
            self.jd_ee_w = sym.simplify(Jdot)
            print("jacobian")
            with open("Jdot_matrix.pkl", "wb") as f:
                pickle.dump(self.jd_ee_w, f)

        qddot = sym.symbols('q(0:' + str(self.nq) + ')ddot', real=True)
        qrdot = sym.symbols('qr(0:' + str(self.nq) + ')dot', real=True)
        qrddot = sym.symbols('qr(0:' + str(self.nq) + ')ddot', real=True)
        self.Y_fn = sym.lambdify([self.q, self.qdot, qddot, qrdot, qrddot], Y)
        self.theta_fn = theta_fn
        self.theta = theta.subs(self.params_dict)
        M_sym = M_sym.subs(params_sym)
        C_sym = C_sym.subs(params_sym)
        G_sym = G_sym.subs(params_sym)
        with open("M.pkl", "wb") as f:
            pickle.dump(M_sym, f)
        with open("C.pkl", "wb") as f:
            pickle.dump(C_sym, f)
        with open("G.pkl", "wb") as f:
            pickle.dump(G_sym, f)
        print("finished subs")
        #self.jd_ee_w = self.jd_ee_w.subs(params_sym)
        #self.Y = Y.subs(params_sym)
        self.M_fn = sym.lambdify([*self.q], M_sym)
        self.C_fn = sym.lambdify([*self.q, *self.qdot], C_sym)
        self.G_fn = sym.lambdify([*self.q], G_sym)

        #self.Jd_ee_w_fn = sym.lambdify([self.q[0], self.q[1], self.q[2], self.qdot[0], self.qdot[1], self.qdot[2]], self.jd_ee_w)
        #print(dn.verify_dynamics(M_sym, C_sym, G_sym))
if __name__ == '__main__':
    model = UR10Model3DOF(pure_symbolic=False, load=True)

    print("Tsym_ee_0:\n", model.Tsym_ee_0, "\n")
    print("Tsym_ee_w:\n", model.Tsym_ee_w, "\n")
    print("Jsym_ee_0:\n", model.Jsym_ee_0, "\n")
    print("Jsym_ee_w:\n", model.Jsym_ee_w, "\n")
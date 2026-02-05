import rospy
import numpy as np

from ur10_model.ur10_model_3dof import UR10Model3DOF
from ur10_robot.ur10_robot_3dof import UR10Robot3DOF
from ur10_controllers.ur10_js_controller import UR10Controller
from ur10_controllers.ur10_joint_error_space_controller import JointErrorSpaceController
def main():
    q0 = np.array([0, 0, 0])
    model = UR10Model3DOF(load=True)
    robot = UR10Robot3DOF(model, q0)
    controller = JointErrorSpaceController(model)

    rate = rospy.Rate(100)
    t0 = rospy.Time.now().to_sec()
    t_prev = t0
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()   
        # TODO: Get current joint states from robot
        q, qdot, qddot = robot.q, robot.qdot, robot.qddot
        # TODO: Update controller with joint states feedback
        dt = t - t_prev
        print(dt)
        joint_states = np.row_stack((q, qdot, qddot))
        tau = controller.update(joint_states, t, dt)
        robot.update_dynamics(tau, dt)
        # TODO: publich joint states and TF frames from robot class
        robot.publish_joint_states()
        t_prev = t
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ur10_js_control_node')
    main()
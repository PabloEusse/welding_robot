import rospy
import numpy as np

from ur10_model.ur10_model_3dof import UR10Model3DOF
from ur10_robot.ur10_robot_3dof import UR10Robot3DOF
from ur10_controllers.ur10_operational_space_controller_6dof import SpaceController
#the main drawback I can think is the assumption that we have a perfect model of the robot and we know all the parameters beforehand.
#A different drawback I found is that the controller is not as intuitive to understand as a normal PD/PID controller which can make tuning harder
def main():
    q0 = np.array([0, 0, 0, 0, 0, 0])
    model = UR10Model3DOF(load=True)
    robot = UR10Robot3DOF(model, q0)
    controller = SpaceController(model, q0)

    rate = rospy.Rate(100)
    t0 = rospy.Time.now().to_sec()
    t_prev = t0
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()   
        
        q, qdot, qddot = robot.q, robot.qdot, robot.qddot
        x, xdot, xddot = robot.get_ee_w(), robot.get_ee_w_dot(), np.zeros(3) 
        
        dt = (t - t_prev) / 10
        joint_states = np.row_stack((q, qdot, qddot))
        cartesian_states = np.row_stack((x, xdot, xddot))
        tau = controller.update(cartesian_states, joint_states, t, dt)
        robot.update_dynamics(tau, dt)
        
        robot.publish_joint_states()
        t_prev = t
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ur10_js_control_node')
    main()
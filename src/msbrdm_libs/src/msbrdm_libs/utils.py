import numpy as np

def heun_integration(out_old, input_old, input_new, dt):
  """Integration from xdot to x"""
  return out_old + (input_old + input_new)*(dt/2.)

def joint_states_integration(qddot: np.ndarray, joint_states: np.ndarray, dt:float) -> np.ndarray:
  """Integrate joint velocities and positions from accelerations using Heun's method

  Args:
      qddot (np.ndarray): input accelerations
      joint_states (np.ndarray): current joint states [q, qdot, qddot].T (3 x n)
      dt (float): time interval

  Returns:
      np.ndarray: updated joint states [q, qdot, qddot].T (3 x n)
  """
  n = len(qddot)
  for i in range(0, n):
    old_vel = joint_states[1, i]
    # velocities
    joint_states[1, i] = heun_integration(joint_states[1, i], joint_states[2, i], qddot[i], dt)
    # positions
    joint_states[0, i] = heun_integration(joint_states[0, i], old_vel, joint_states[1, i], dt)
    joint_states[2, i] = qddot[i]

  return joint_states
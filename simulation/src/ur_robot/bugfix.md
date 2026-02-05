These are bug fixes for Tutorial 4. In order to avoid merge conflicts, these changes are not uploaded to the templates. Please make the changes manually yourselves. 

In the file ur10_robot_3dof.py, make the following changes
1. Replace the lines
```python
# joint states: [q, qdot, qddot] (n x 3)
self.joint_states = np.zeros((self.model.nq, 3))
if q_init is not None:
    self.joint_states[:, 0] = q_init
# Note: q, qdot, qddot hold the same reference of joint_states
# they can be accessed both as a whole and separately
self.q = self.joint_states[:, 0].reshape(self.model.nq, 1)
self.qdot = self.joint_states[:, 1].reshape(self.model.nq, 1)
self.qddot = self.joint_states[:, 2].reshape(self.model.nq, 1)
```
with the following:
```python
# joint states: [q, qdot, qddot].T (3 x n)
self.q = np.zeros(self.model.nq)
if q_init is not None:
    self.q = q_init
self.qdot = np.zeros(self.model.nq)
self.qddot = np.zeros(self.model.nq)
```

2. Add the following function in the class
```python
def get_joint_states(self) -> np.ndarray:
""" Get the joint states

Returns:
    np.ndarray: joint states (3 x n)
    first row: joint positions
    second row: joint velocities
    third row: joint accelerations
"""
joint_states = np.row_stack((self.q, self.qdot, self.qddot))
return joint_states
```

Explanation: the idea is to simplify the variables for joint states. Inside the class, we only use the variables `self.q, self.qdot, self.qddot`, without having a synthesized member variable `self.joint_states` anymore. The getter function `get_joint_states()` provides the possibility to access the whole joint states from the outside.
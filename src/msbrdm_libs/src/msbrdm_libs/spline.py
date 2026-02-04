import numpy as np


def polynomial_5th_order(xi: float, xf: float, ti: float, tf: float) -> np.ndarray:
    """Calculate the coefficients of a 5th-order polynomial trajectory of a 1D spline

    Args:
        xi (float): start position
        xf (float): end position
        ti (float): start time
        tf (float): end time

    Returns:
        np.ndarray: vector of coefficients of the 5th-order polynomial
    """
    T = np.array([
        [1, ti, ti**2, ti**3, ti**4, ti**5],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*ti, 3*ti**2, 4*ti**3, 5*ti**4],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*ti, 12*ti**2, 20*ti**3],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
    ])
    x = np.array(
        [xi, xf, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)]
    )
    a = np.zeros(6)
    return np.linalg.pinv(T) @ x


class LinearSplineTrajectory:
    def __init__(self, x_init: np.ndarray, x_final: np.ndarray, duration: float, t0 = 0) -> None:
        """Initialize a linear spline trajectory by calculating the coefficients 
        of a 5th-order polynomial for all three linear axes

        Args:
            x_init (np.ndarray): start position [x, y, z]
            x_final (np.ndarray): end position [x, y, z]<
            duration (float): duration of the trajectory
        """
        # TODO: calculate the coefficients of the 5th-order polynomial for all three axes
        # self.a0 = x_init.copy()
        self.duration = duration
        self.goal = x_final
        # self.a1 = (x_final - x_init) / duration
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5 = polynomial_5th_order(x_init, x_final, t0, duration)   
    def update(self, t: float) -> np.ndarray:
        """ Update the desired trajectory at time t

        Args:
            t (float): timestamp

        Returns:
            np.ndarray: x_des, v_des, a_des
        """
        x_des = self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3  + self.a4 * t**4 + self.a5 * t**5 if t < self.duration else self.goal
        v_des = self.a1 if t < self.duration else np.zeros(3)
        a_des = np.zeros(3)

        # TODO: calculate the desired position, velocity, and acceleration at time t
        # with the coefficients

        return x_des, v_des, a_des

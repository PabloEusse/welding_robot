class PathController:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.current_idx = 0
        self.completed = False

    def get_next_waypoint(self, current_robot_pose):
        """
        Called by the robot control loop (e.g., at 100Hz)
        """
        if self.completed:
            return None

        target = self.trajectory[self.current_idx]
        
        # Check distance to target
        dist = np.linalg.norm(target['pos'] - current_robot_pose[:3])
        
        # If we are close enough, switch to next point
        if dist < 0.005: # 5mm tolerance
            self.current_idx += 1
            if self.current_idx >= len(self.trajectory):
                self.completed = True
                return None
            target = self.trajectory[self.current_idx]

        return target # Returns the 6D pose for the robot to servo towards
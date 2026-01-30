import numpy as np
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation as R

class TrajectoryPlanner:
    def __init__(self, weld_speed=0.01, corner_angle_thresh=30):
        self.weld_speed = weld_speed # m/s
        self.corner_thresh = np.radians(corner_angle_thresh)

    def generate_plan(self, raw_points, surface_normal):
        """
        Main Interface Function
        Input: Nx3 numpy array (XYZ), 3x1 normal vector
        Output: List of dictionaries [{'pos': [x,y,z], 'quat': [x,y,z,w], 'cmd': 'MOVE/WELD'}]
        """
        # 1. Analyze Geometry
        corners_indices = self._detect_corners(raw_points)
        
        # 2. Segment the path
        segments = self._split_at_indices(raw_points, corners_indices)
        
        full_trajectory = []
        
        for i, segment in enumerate(segments):
            # 3. Fit Spline/Line to this segment
            dense_path = self._fit_path(segment)
            
            # 4. Compute Orientations (Tangents)
            segment_traj = self._compute_orientations(dense_path, surface_normal)
            
            # 5. Link Segments (Corner Logic)
            if i > 0:
                # Calculate pivot rotation at the corner
                prev_pose = full_trajectory[-1]
                curr_pose = segment_traj[0]
                
                # Insert a Corner Pivot Action (Robot stays at X,Y,Z but rotates)
                pivot_step = self._generate_corner_pivot(prev_pose, curr_pose)
                full_trajectory.extend(pivot_step)
            
            # Add the actual movement
            full_trajectory.extend(segment_traj)
            
        return full_trajectory


    # NOTE: this is probably helpful
    def _detect_corners(self, points):
        """ Returns indices where direction changes sharply """
        corners = [0] # Start is always a 'break'
        vecs = np.diff(points, axis=0)
        # Normalize vectors
        vecs = vecs / np.linalg.norm(vecs, axis=1)[:, np.newaxis]
        
        for i in range(len(vecs)-1):
            # Dot product to find angle
            dot = np.dot(vecs[i], vecs[i+1])
            angle = np.arccos(np.clip(dot, -1.0, 1.0))
            if angle > self.corner_thresh:
                corners.append(i+1)
        
        corners.append(len(points)-1)
        return corners

    # NOTE: probably not good enough
    def _fit_path(self, points, resolution=0.005):
        """ Fits B-Spline to smooth 'hand jitter' """
        if len(points) < 3: return points # Too short for spline
        
        # 's' parameter controls smoothness. Higher = smoother (less jitter)
        tck, u = splprep(points.T, s=0.005) 
        
        # Calculate how many points we need for resolution
        dist = np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))
        num_points = int(dist / resolution)
        
        u_new = np.linspace(0, 1, num_points)
        new_points = np.column_stack(splev(u_new, tck))
        return new_points

    def _compute_orientations(self, points, normal):
        """ Generates 6D poses based on Tangent & Normal """
        traj = []
        z_axis = normal / np.linalg.norm(normal)
        
        for i in range(len(points)-1):
            # X-axis follows the movement
            tangent = points[i+1] - points[i]
            x_axis = tangent / np.linalg.norm(tangent)
            
            # Y-axis completes the frame
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            # Re-orthogonalize X to ensure 90 degrees
            x_axis = np.cross(y_axis, z_axis)
            
            # Create Rotation Matrix -> Quaternion
            rot_mat = np.column_stack((x_axis, y_axis, z_axis))
            quat = R.from_matrix(rot_mat).as_quat()
            
            traj.append({
                'pos': points[i],
                'quat': quat,
                'cmd': 'WELD'
            })
        return traj

    def _generate_corner_pivot(self, pose_a, pose_b):
        """ Interpolates orientation while keeping position fixed """
        # In a real welding scenario, this pauses the weld, rotates, then resumes
        return [{
            'pos': pose_a['pos'], # Position stays same
            'quat': pose_b['quat'], # Snap to new rotation (or interpolate properly)
            'cmd': 'PIVOT' # Marker for the robot controller to stop linear motion
        }]
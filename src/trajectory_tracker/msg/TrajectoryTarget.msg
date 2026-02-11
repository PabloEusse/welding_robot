# TrajectoryTarget.msg
# Detected target position for trajectory tracking

Header header

# 3D position in robot base frame [meters]
geometry_msgs/Point position

# 2D pixel position in image [pixels]
float64 pixel_x
float64 pixel_y

# Detection confidence [0.0 - 1.0]
float64 confidence

# Marker ID (if using ArUco markers, -1 if pen tip detection)
int32 marker_id

# Is this target valid for trajectory following?
bool valid

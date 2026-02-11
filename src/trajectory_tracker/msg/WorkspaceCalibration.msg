# WorkspaceCalibration.msg
# Workspace defined by two corner markers

Header header

# Is workspace calibrated?
bool is_calibrated

# Marker IDs used for calibration
int32 marker_id_0
int32 marker_id_1

# Workspace corner positions in robot frame [meters]
geometry_msgs/Point corner_0
geometry_msgs/Point corner_1

# Workspace dimensions
float64 width
float64 height

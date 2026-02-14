# WorkspaceCalibration.msg
# Workspace defined by four corner markers

Header header

# Is workspace calibrated?
bool is_calibrated

# Marker IDs used for calibration (4 corners)
# Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
int32[] marker_ids

# Workspace corner positions in robot frame [meters]
# Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
geometry_msgs/Point[] corners

# Workspace dimensions
float64 width   # X-axis span (left to right)
float64 height  # Y-axis span (top to bottom)

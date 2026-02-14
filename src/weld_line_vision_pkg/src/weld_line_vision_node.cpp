#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/aruco.hpp>      
#include <algorithm>
#include <cmath>
#include <sstream>

class TrajectoryGenerator
{
public:
  explicit TrajectoryGenerator(ros::NodeHandle& nh)
  : nh_(nh), it_(nh_)
  {
    // Frames
    nh_.param<std::string>("image_topic", image_topic_, "/droidcam/image_raw");
    // nh_.param<std::string>("frame_id", frame_id_, "paper");
    nh_.param<std::string>("frame_id", frame_id_, "workspace_frame");

    // Size of the paper representing the workspace (We used paper for test.)
    nh_.param<double>("paper_width_m",  paper_w_m_, 0.210); // A4 width in m
    nh_.param<double>("paper_height_m", paper_h_m_, 0.297); // A4 height in m

    // Settings for the ArUco codes.
    nh_.param<int>("aruco_dict_id", aruco_dict_id_, (int)cv::aruco::DICT_4X4_50);
    nh_.param<double>("aruco_marker_size", aruco_marker_size_, 0.06);
    nh_.param<double>("ws_w1_margin_offset", paper_w1_margin_offset_, 0.015);
    nh_.param<double>("ws_h1_margin_offset", paper_h1_margin_offset_, 0.018);
    nh_.param<double>("ws_w2_margin_offset", paper_w2_margin_offset_, 0.015);
    nh_.param<double>("ws_h2_margin_offset", paper_h2_margin_offset_, 0.018);
    nh_.param<double>("ws_w3_margin_offset", paper_w3_margin_offset_, 0.015);
    nh_.param<double>("ws_h3_margin_offset", paper_h3_margin_offset_, 0.018);
    nh_.param<double>("ws_w4_margin_offset", paper_w4_margin_offset_, 0.015);
    nh_.param<double>("ws_h4_margin_offset", paper_h4_margin_offset_, 0.018);
    nh_.param<int>("id_tl", id_tl_, 0);   // ID Top Left
    nh_.param<int>("id_tr", id_tr_, 1);   // ID Top Right
    nh_.param<int>("id_bl", id_bl_, 2);   // ID Bottom Left
    nh_.param<int>("id_br", id_br_, 3);   // ID Bottom Right

    // Robot base markers (10cm markers)
    nh_.param<int>("id_robot_top", id_robot_top_, 7);       // ID 7 (top marker)
    nh_.param<int>("id_robot_bottom", id_robot_bottom_, 17); // ID 17 (bottom marker)
    nh_.param<double>("robot_marker_size", robot_marker_size_, 0.10); // 10cm
    nh_.param<double>("robot_marker_distance", robot_marker_distance_, 0.20); // Distance between robot markers
    nh_.param<double>("robot_base_y_offset_m", robot_base_y_offset_m_, 0.0); // Lateral offset in workspace frame
    nh_.param<std::string>("robot_base_frame", robot_base_frame_, "robot_base_link");

    // HSV values for the chosen color (cyan blue).
    nh_.param<int>("h_low",  h_low_,  75);
    nh_.param<int>("h_high", h_high_, 130);
    nh_.param<int>("s_low",  s_low_,  20);
    nh_.param<int>("v_low",  v_low_,  40);

    // ROI is optional for the moment
    nh_.param<int>("roi_x", roi_x_, 0);
    nh_.param<int>("roi_y", roi_y_, 0);
    nh_.param<int>("roi_w", roi_w_, 0);
    nh_.param<int>("roi_h", roi_h_, 0);

    // Morphology
    nh_.param<int>("morph_ksize", morph_ksize_, 3);

    // Sampling along skeleton
    nh_.param<int>("sample_step_px", sample_step_px_, 6);
    nh_.param<bool>("rotate_image_90_cw", rotate_image_90_cw_, false);
    nh_.param<bool>("use_camera_separate_calibration", use_camera_separate_calibration_, true);
    nh_.param<bool>("allow_homography_fallback_for_robot_pose", allow_homography_fallback_for_robot_pose_, false);
    nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/droidcam/camera_info");
    // Backward compatibility with older launch files.
    nh_.param<std::string>("info_topic", camera_info_topic_, camera_info_topic_);

    // Subscribers 
    img_sub_  = it_.subscribe(image_topic_, 1, &TrajectoryGenerator::imgCallback, this);
    cam_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &TrajectoryGenerator::cameraInfoCallback, this);

    // Publishers
    path_pub_   = nh_.advertise<nav_msgs::Path>("weld_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("weld_points_marker", 1);
    workspace_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("workspace_marker", 1);

    dbg_pub_  = it_.advertise("debug_image", 1);    // Bounding box for the detected object (for debugging)
    mask_pub_ = it_.advertise("debug_mask", 1);     // Mask or the detected object (for debugging)
    skel_pub_ = it_.advertise("debug_skeleton", 1); // Piwel-wise path

    dict_ = cv::aruco::getPredefinedDictionary((cv::aruco::PREDEFINED_DICTIONARY_NAME)aruco_dict_id_);
    params_ = cv::aruco::DetectorParameters::create();
    params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params_->adaptiveThreshWinSizeMin = 3;
    params_->adaptiveThreshWinSizeMax = 53;
    params_->adaptiveThreshWinSizeStep = 4;
    params_->minMarkerPerimeterRate = 0.01;
    params_->minDistanceToBorder = 3;

    ROS_INFO("Trajectory Generator initialized. Subscribing to: %s", image_topic_.c_str());
    ROS_INFO("Camera info topic: %s", camera_info_topic_.c_str());
    ROS_INFO("Robot pose mode: %s", use_camera_separate_calibration_ ? "camera-separate (PnP)" : "workspace-homography");
    ROS_INFO("Robot pose fallback to homography: %s", allow_homography_fallback_for_robot_pose_ ? "enabled" : "disabled");
    ROS_INFO("Robot base Y offset: %.4f m", robot_base_y_offset_m_);
    //ROS_INFO("Paper size: %.3f x %.3f m, frame_id=%s", paper_w_m_, paper_h_m_, frame_id_.c_str());
    ROS_INFO("ArUco ids: TL=%d TR=%d BL=%d BR=%d (dict=%d)", id_tl_, id_tr_, id_bl_, id_br_, aruco_dict_id_);
  }

private:
  bool isInsideWorkspace(double x, double y, double margin = 0.0) const
  {
    return (x >= -margin && x <= (paper_w_m_ + margin) &&
            y >= -margin && y <= (paper_h_m_ + margin));
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    if (msg->K[0] <= 0.0 || msg->K[4] <= 0.0) {
      ROS_WARN_THROTTLE(2.0, "Invalid camera intrinsics on %s (fx/fy <= 0).",
                        camera_info_topic_.c_str());
      return;
    }

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        camera_matrix_.at<double>(r, c) = msg->K[r * 3 + c];
      }
    }

    const int dsize = std::max<size_t>(msg->D.size(), 5);
    dist_coeffs_ = cv::Mat::zeros(dsize, 1, CV_64F);
    for (size_t i = 0; i < msg->D.size(); ++i) {
      dist_coeffs_.at<double>((int)i, 0) = msg->D[i];
    }

    camera_frame_id_ = msg->header.frame_id.empty() ? "camera_optical_frame" : msg->header.frame_id;

    if (!has_camera_info_) {
      ROS_INFO("Received camera intrinsics from %s (frame=%s).",
               camera_info_topic_.c_str(), camera_frame_id_.c_str());
    }
    has_camera_info_ = true;
  }

  static bool normalizeVec(cv::Vec3d& v)
  {
    const double n = cv::norm(v);
    if (!std::isfinite(n) || n < 1e-9) return false;
    v /= n;
    return true;
  }

  static tf2::Matrix3x3 cvMatToTf3x3(const cv::Mat& R)
  {
    return tf2::Matrix3x3(
      R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)
    );
  }

  static std::vector<cv::Point3f> markerObjectPoints(double marker_size_m)
  {
    const float s = static_cast<float>(marker_size_m * 0.5);
    return {
      cv::Point3f(-s,  s, 0.f), // top-left
      cv::Point3f( s,  s, 0.f), // top-right
      cv::Point3f( s, -s, 0.f), // bottom-right
      cv::Point3f(-s, -s, 0.f)  // bottom-left
    };
  }

  bool estimateSingleMarkerPoseInCamera(
      const std::vector<int>& ids,
      const std::vector<std::vector<cv::Point2f>>& corners,
      int target_id,
      double marker_size_m,
      cv::Vec3d& rvec,
      cv::Vec3d& tvec) const
  {
    if (!has_camera_info_) return false;

    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != target_id) continue;
      const std::vector<cv::Point3f> obj_pts = markerObjectPoints(marker_size_m);
      const bool ok = cv::solvePnP(
        obj_pts, corners[i], camera_matrix_, dist_coeffs_,
        rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
      return ok;
    }
    return false;
  }

  bool computeWorkspacePoseInCamera(
      const std::vector<int>& ids,
      const std::vector<std::vector<cv::Point2f>>& corners,
      cv::Mat& R_cam_ws,
      cv::Vec3d& t_cam_ws) const
  {
    if (!has_camera_info_) return false;

    auto findCenterById = [&](int target_id, cv::Point2f& out_center) -> bool {
      for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == target_id) {
          out_center = markerCenter(corners[i]);
          return true;
        }
      }
      return false;
    };

    cv::Point2f c_tl, c_tr, c_bl, c_br;
    const bool ok =
      findCenterById(id_tl_, c_tl) &&
      findCenterById(id_tr_, c_tr) &&
      findCenterById(id_bl_, c_bl) &&
      findCenterById(id_br_, c_br);
    if (!ok) return false;

    const std::vector<cv::Point2f> paper_pts = computeAuroCenters();
    std::vector<cv::Point3f> obj_pts;
    obj_pts.reserve(4);
    for (const auto& p : paper_pts) {
      obj_pts.emplace_back(p.x, p.y, 0.f);
    }

    const std::vector<cv::Point2f> img_pts = {c_tl, c_tr, c_bl, c_br};
    cv::Vec3d rvec_ws;
    if (!cv::solvePnP(obj_pts, img_pts, camera_matrix_, dist_coeffs_, rvec_ws, t_cam_ws, false, cv::SOLVEPNP_ITERATIVE)) {
      return false;
    }

    cv::Rodrigues(rvec_ws, R_cam_ws);
    return true;
  }

  bool computeRobotBasePoseInCamera(
      const std::vector<int>& ids,
      const std::vector<std::vector<cv::Point2f>>& corners,
      cv::Mat& R_cam_base,
      cv::Vec3d& t_cam_base) const
  {
    cv::Vec3d rvec7, tvec7, rvec17, tvec17;
    const bool ok7 = estimateSingleMarkerPoseInCamera(ids, corners, id_robot_top_, robot_marker_size_, rvec7, tvec7);
    const bool ok17 = estimateSingleMarkerPoseInCamera(ids, corners, id_robot_bottom_, robot_marker_size_, rvec17, tvec17);
    if (!ok7 || !ok17) return false;

    cv::Mat R_cam_m7;
    cv::Rodrigues(rvec7, R_cam_m7);

    // X axis: normal to marker 7 plane.
    cv::Vec3d x_axis(
      R_cam_m7.at<double>(0, 2),
      R_cam_m7.at<double>(1, 2),
      R_cam_m7.at<double>(2, 2));
    if (!normalizeVec(x_axis)) return false;

    // Z axis: direction from marker 17 -> marker 7, orthogonalized against X.
    cv::Vec3d z_axis = tvec7 - tvec17;
    z_axis -= x_axis * z_axis.dot(x_axis);
    if (!normalizeVec(z_axis)) {
      ROS_WARN_THROTTLE(2.0, "Robot Z axis from IDs %d->%d is degenerate.",
                        id_robot_bottom_, id_robot_top_);
      return false;
    }

    // Y axis: left direction on the marker 7 plane.
    cv::Vec3d y_axis = z_axis.cross(x_axis);
    if (!normalizeVec(y_axis)) return false;

    // Re-orthogonalize Z for numerical stability.
    z_axis = x_axis.cross(y_axis);
    if (!normalizeVec(z_axis)) return false;

    R_cam_base = cv::Mat::zeros(3, 3, CV_64F);
    R_cam_base.at<double>(0, 0) = x_axis[0];
    R_cam_base.at<double>(1, 0) = x_axis[1];
    R_cam_base.at<double>(2, 0) = x_axis[2];
    R_cam_base.at<double>(0, 1) = y_axis[0];
    R_cam_base.at<double>(1, 1) = y_axis[1];
    R_cam_base.at<double>(2, 1) = y_axis[2];
    R_cam_base.at<double>(0, 2) = z_axis[0];
    R_cam_base.at<double>(1, 2) = z_axis[1];
    R_cam_base.at<double>(2, 2) = z_axis[2];

    // Origin at marker 7 center.
    t_cam_base = tvec7;
    return true;
  }

  static inline bool inBounds(int x, int y, int w, int h)
  {
    return (x >= 0 && y >= 0 && x < w && y < h);
  }

  // This function returns the number of active neighbours for a pixel (x,y).
  int neighborCount8(const cv::Mat& skel, int x, int y) const
  {
    // remember that the coordinate system in OpenCV is defined like:
    // (0,0) --- x
    //   |
    //   y
    static const int dx[8] = {-1, 0, 1,-1, 1,-1, 0, 1};
    static const int dy[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    int c = 0;
    for (int k = 0; k < 8; ++k) {
      int nx = x + dx[k], ny = y + dy[k];
      if (inBounds(nx, ny, skel.cols, skel.rows) && skel.at<uint8_t>(ny, nx) > 0) c++;
    //std::cout<<"Iteration: "<<k<<", nx value: "<<nx<<"ny value:"<<ny<<"skel columns"<<skel.cols<<"skel rows: "<<skel.rows<<std::endl;
    }
    return c;
  }

  std::vector<cv::Point> traceSkeleton(const cv::Mat& skel) const
  {
    std::vector<cv::Point> points;
    std::vector<cv::Point> endpoints;

    std::vector<cv::Point> on_pixels;
    on_pixels.reserve(5000);

    // Get the pixels that are part of the trayectory.
    for (int y = 0; y < skel.rows; ++y) {
      const uint8_t* row = skel.ptr<uint8_t>(y);
      for (int x = 0; x < skel.cols; ++x) {
        if (row[x] > 0) on_pixels.emplace_back(x, y);
      }
    }

    if (on_pixels.size() < 2)
    {
      ROS_WARN_THROTTLE(1.0, "Only one pixel was detected. Cannot for a path.");
      return points;
    }

    for (const auto& p : on_pixels) {
      int n = neighborCount8(skel, p.x, p.y);
      if (n == 1) endpoints.push_back(p);
    }

    cv::Point start = endpoints.empty() ? on_pixels.front() : endpoints.front();
    cv::Mat visited = cv::Mat::zeros(skel.size(), CV_8U);

    static const int dx[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
    static const int dy[8] = {-1, -1, -1,  0, 0,  1, 1, 1};

    cv::Point cur = start;
    cv::Point prev(-999, -999);

    while (true) {
      points.push_back(cur);
      visited.at<uint8_t>(cur.y, cur.x) = 255;

      std::vector<cv::Point> neigh;
      neigh.reserve(8);
      for (int k = 0; k < 8; ++k) {
        cv::Point np(cur.x + dx[k], cur.y + dy[k]);
        if (!inBounds(np.x, np.y, skel.cols, skel.rows)) continue;
        if (skel.at<uint8_t>(np.y, np.x) == 0) continue;
        if (np == prev) continue;
        neigh.push_back(np);
      }
      if (neigh.empty()) break;

      cv::Point next = neigh.front();
      bool found_unvisited = false;
      for (const auto& np : neigh) {
        if (visited.at<uint8_t>(np.y, np.x) == 0) {
          next = np;
          found_unvisited = true;
          break;
        }
      }
      if (!found_unvisited) break;

      prev = cur;
      cur = next;

      if ((int)points.size() > 200000) break;
    }

    return points;
  }

  static cv::Point2f markerCenter(const std::vector<cv::Point2f>& corners4)
  {
    cv::Point2f c(0.f, 0.f);
    for (const auto& p : corners4) c += p;
    c *= 0.25f;
    return c;
  }

  static cv::Point2f pointFromRotated90CWToOriginal(const cv::Point2f& p_rot, int original_rows)
  {
    // Rotation used in the pipeline: original -> rotated is 90deg CW.
    // Here we invert it so camera PnP uses coordinates compatible with camera_info.
    return cv::Point2f(p_rot.y, static_cast<float>(original_rows - 1) - p_rot.x);
  }

  static std::vector<std::vector<cv::Point2f>> unrotateCorners90CWToOriginal(
      const std::vector<std::vector<cv::Point2f>>& corners_rot,
      int original_rows)
  {
    std::vector<std::vector<cv::Point2f>> corners_orig;
    corners_orig.reserve(corners_rot.size());
    for (const auto& one_marker : corners_rot) {
      std::vector<cv::Point2f> marker_orig;
      marker_orig.reserve(one_marker.size());
      for (const auto& p : one_marker) {
        marker_orig.emplace_back(pointFromRotated90CWToOriginal(p, original_rows));
      }
      corners_orig.emplace_back(marker_orig);
    }
    return corners_orig;
  }

  static std::string idsToString(const std::vector<int>& ids)
  {
    if (ids.empty()) return "none";
    std::ostringstream oss;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (i > 0) oss << ",";
      oss << ids[i];
    }
    return oss.str();
  }

  std::vector<cv::Point2f> computeAuroCenters() const
  {
    std::vector<cv::Point2f> paper_pts;
    paper_pts.reserve(4);
    const float marker_center = static_cast<float>(aruco_marker_size_ * 0.5);

    cv::Point2f top_left_center(marker_center + paper_w1_margin_offset_, 
                                    marker_center + paper_h1_margin_offset_); 
    cv::Point2f top_right_center(paper_w_m_ - marker_center - paper_w2_margin_offset_, 
                                    marker_center + paper_h2_margin_offset_); 
    cv::Point2f bottom_left_center(marker_center + paper_w3_margin_offset_, 
                                    paper_h_m_ - marker_center - paper_h3_margin_offset_); 
    cv::Point2f bottom_right_center(paper_w_m_ - marker_center - paper_w4_margin_offset_, 
                                    paper_h_m_ - marker_center - paper_h4_margin_offset_);   
    paper_pts.emplace_back(top_left_center);  
    paper_pts.emplace_back(top_right_center);
    paper_pts.emplace_back(bottom_left_center);
    paper_pts.emplace_back(bottom_right_center);          
    return paper_pts;                                                     
  }

  bool computeHomographyFromArucoCenters(
      const cv::Mat& bgr_full,
      cv::Mat& H_img_to_paper,
      cv::Mat& dbg_draw)
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    // Get Aruco markers, their ID and their corners (in pixels).
    cv::aruco::detectMarkers(bgr_full, dict_, corners, ids, params_);

    dbg_draw = bgr_full.clone();
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(dbg_draw, corners, ids);
    }

    auto findCenterById = [&](int target_id, cv::Point2f& out_center) -> bool {
      for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == target_id) {
          out_center = markerCenter(corners[i]);
          return true;
        }
      }
      return false;
    };

    cv::Point2f c_tl, c_tr, c_bl, c_br;
    bool ok =
      findCenterById(id_tl_, c_tl) &&
      findCenterById(id_tr_, c_tr) &&
      findCenterById(id_bl_, c_bl) &&
      findCenterById(id_br_, c_br);

    if (!ok) {
      ROS_WARN_THROTTLE(1.0, "Missing ArUco corners. Need ids TL=%d TR=%d BL=%d BR=%d, detected=[%s], dict=%d",
                        id_tl_, id_tr_, id_bl_, id_br_, idsToString(ids).c_str(), aruco_dict_id_);
      return false;
    }

    // Image points (pixels)
    std::vector<cv::Point2f> img_pts = {c_tl, c_tr, c_bl, c_br};

    std::vector<cv::Point2f> paper_pts = computeAuroCenters();

    H_img_to_paper = cv::findHomography(img_pts, paper_pts, cv::RANSAC);
    if (H_img_to_paper.empty()) {
      ROS_WARN_THROTTLE(1.0, "Homography could not be calculated.");
      return false;
    }

    // draw centers for visual debug
    auto drawC = [&](const cv::Point2f& p, const std::string& txt) {
      cv::circle(dbg_draw, p, 5, cv::Scalar(0, 255, 255), -1);
      cv::putText(dbg_draw, txt, p + cv::Point2f(6, -6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    };
    drawC(c_tl, "TL");
    drawC(c_tr, "TR");
    drawC(c_bl, "BL");
    drawC(c_br, "BR");

    return true;
  }

  bool buildWorkspaceMaskInImage(
      const cv::Mat& H_img_to_paper,
      const cv::Size& image_size,
      cv::Mat& workspace_mask,
      std::vector<cv::Point>& workspace_poly) const
  {
    workspace_mask = cv::Mat::zeros(image_size, CV_8U);
    workspace_poly.clear();

    if (H_img_to_paper.empty()) return false;

    cv::Mat H_paper_to_img = H_img_to_paper.inv();
    if (H_paper_to_img.empty()) return false;

    std::vector<cv::Point2f> paper_corners = {
      cv::Point2f(0.f, 0.f),
      cv::Point2f(static_cast<float>(paper_w_m_), 0.f),
      cv::Point2f(static_cast<float>(paper_w_m_), static_cast<float>(paper_h_m_)),
      cv::Point2f(0.f, static_cast<float>(paper_h_m_))
    };

    std::vector<cv::Point2f> image_corners;
    cv::perspectiveTransform(paper_corners, image_corners, H_paper_to_img);

    workspace_poly.reserve(image_corners.size());
    for (const auto& p : image_corners) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        return false;
      }
      workspace_poly.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));
    }

    if (workspace_poly.size() != 4) return false;

    cv::fillConvexPoly(workspace_mask, workspace_poly, cv::Scalar(255));
    return true;
  }

  void publishWorkspaceMarker(const ros::Time& stamp)
  {
    visualization_msgs::Marker ws_marker;
    ws_marker.header.frame_id = frame_id_;
    ws_marker.header.stamp = stamp;
    ws_marker.ns = "workspace";
    ws_marker.id = 0;
    ws_marker.type = visualization_msgs::Marker::LINE_STRIP;
    ws_marker.action = visualization_msgs::Marker::ADD;
    ws_marker.pose.orientation.w = 1.0;
    ws_marker.scale.x = 0.003; // Line width
    ws_marker.color.r = 0.0;
    ws_marker.color.g = 1.0;
    ws_marker.color.b = 0.0;
    ws_marker.color.a = 0.8;

    // Workspace corners
    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = 0.0; p1.y = 0.0; p1.z = 0.0;
    p2.x = paper_w_m_; p2.y = 0.0; p2.z = 0.0;
    p3.x = paper_w_m_; p3.y = paper_h_m_; p3.z = 0.0;
    p4.x = 0.0; p4.y = paper_h_m_; p4.z = 0.0;

    ws_marker.points.push_back(p1);
    ws_marker.points.push_back(p2);
    ws_marker.points.push_back(p3);
    ws_marker.points.push_back(p4);
    ws_marker.points.push_back(p1); // Close the loop

    workspace_marker_pub_.publish(ws_marker);
  }

  bool computeRobotBaseTransformFromHomography(
      const std::vector<int>& ids,
      const std::vector<std::vector<cv::Point2f>>& corners,
      const cv::Mat& H_img_to_paper,
      const ros::Time& stamp)
  {
    // Find robot markers (7=top, 17=bottom)
    auto findCenterById = [&](int target_id, cv::Point2f& out_center) -> bool {
      for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == target_id) {
          out_center = markerCenter(corners[i]);
          return true;
        }
      }
      return false;
    };

    cv::Point2f c_top, c_bottom;
    bool ok = findCenterById(id_robot_top_, c_top) &&
              findCenterById(id_robot_bottom_, c_bottom);

    if (!ok) {
      ROS_WARN_THROTTLE(2.0, "Robot markers not detected. Need IDs %d (top) and %d (bottom)",
                        id_robot_top_, id_robot_bottom_);
      return false;
    }

    // Transform marker centers from pixels to workspace coordinates
    std::vector<cv::Point2f> robot_centers_px = {c_top, c_bottom};
    std::vector<cv::Point2f> robot_centers_ws;
    cv::perspectiveTransform(robot_centers_px, robot_centers_ws, H_img_to_paper);

    cv::Point2f marker7_ws = robot_centers_ws[0];   // ID 7 (origin)
    cv::Point2f marker17_ws = robot_centers_ws[1];  // ID 17 (heading reference)

    // Robot base origin is at marker 7 position
    double robot_x = marker7_ws.x;
    double robot_y = marker7_ws.y;
    double robot_z = 0.0;  // On the workspace plane

    // Compute robot base frame orientation:
    // Use marker 17 -> 7 as heading direction in the workspace plane.
    // Keep robot Z axis aligned with workspace +Z so the robot stays upright in RViz.
    double vec_x = marker7_ws.x - marker17_ws.x;
    double vec_y = marker7_ws.y - marker17_ws.y;
    double vec_norm = std::sqrt(vec_x*vec_x + vec_y*vec_y);

    if (vec_norm < 1e-6) {
      ROS_WARN_THROTTLE(2.0, "Markers 7 and 17 are too close!");
      return false;
    }

    // Normalized heading vector (17→7) on workspace plane.
    const double dir_x = vec_x / vec_norm;
    const double dir_y = vec_y / vec_norm;

    // Robot axes in workspace coordinates:
    // X (forward): heading (17->7)
    // Y (left): 90deg CCW from heading
    // Z (up): workspace +Z
    const double x_x = dir_x;
    const double x_y = dir_y;
    const double y_x = -dir_y;
    const double y_y = dir_x;

    // Rotation matrix with columns [X_robot | Y_robot | Z_robot]
    tf2::Matrix3x3 rot_matrix(
      x_x, y_x, 0.0,
      x_y, y_y, 0.0,
      0.0, 0.0, 1.0
    );

    // Convert rotation matrix to quaternion
    tf2::Quaternion q;
    rot_matrix.getRotation(q);

    // Create and publish TF transform
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = frame_id_;  // workspace_frame
    transform_stamped.child_frame_id = robot_base_frame_;

    transform_stamped.transform.translation.x = robot_x;
    transform_stamped.transform.translation.y = robot_y;
    transform_stamped.transform.translation.z = robot_z;
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(transform_stamped);

    // Log robot base info
    double roll, pitch, yaw_out;
    rot_matrix.getRPY(roll, pitch, yaw_out);
    ROS_INFO_THROTTLE(5.0, "Robot base at marker 7: pos=(%.3f, %.3f, %.3f)m, heading(17->7)=%.2f deg, rpy=(%.2f, %.2f, %.2f) deg",
                      robot_x, robot_y, robot_z,
                      std::atan2(dir_y, dir_x) * 180.0 / M_PI,
                      roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw_out * 180.0 / M_PI);

    return true;
  }

  bool computeRobotBaseTransformFromCamera(
      const std::vector<int>& ids,
      const std::vector<std::vector<cv::Point2f>>& corners,
      const ros::Time& stamp)
  {
    cv::Mat R_cam_ws, R_cam_base;
    cv::Vec3d t_cam_ws, t_cam_base;

    if (!computeWorkspacePoseInCamera(ids, corners, R_cam_ws, t_cam_ws)) {
      ROS_WARN_THROTTLE(2.0, "Workspace pose in camera frame unavailable (need IDs %d,%d,%d,%d).",
                        id_tl_, id_tr_, id_bl_, id_br_);
      return false;
    }

    if (!computeRobotBasePoseInCamera(ids, corners, R_cam_base, t_cam_base)) {
      ROS_WARN_THROTTLE(2.0, "Robot base pose in camera frame unavailable (need IDs %d and %d).",
                        id_robot_top_, id_robot_bottom_);
      return false;
    }

    // Compose relative transform via camera frame:
    // T_ws_base = inv(T_cam_ws) * T_cam_base
    const tf2::Transform T_cam_ws(
      cvMatToTf3x3(R_cam_ws),
      tf2::Vector3(t_cam_ws[0], t_cam_ws[1], t_cam_ws[2]));
    const tf2::Transform T_cam_base(
      cvMatToTf3x3(R_cam_base),
      tf2::Vector3(t_cam_base[0], t_cam_base[1], t_cam_base[2]));

    const tf2::Transform T_ws_base = T_cam_ws.inverse() * T_cam_base;
    tf2::Vector3 p = T_ws_base.getOrigin();
    tf2::Matrix3x3 R = T_ws_base.getBasis();

    // Desired convention correction:
    // rotate the composed robot pose by 180 deg around workspace +Y axis.
    // p' = Ry(pi) * p, R' = Ry(pi) * R
    const tf2::Matrix3x3 Ry_pi(
      -1.0, 0.0,  0.0,
       0.0, 1.0,  0.0,
       0.0, 0.0, -1.0);
    p = Ry_pi * p;
    R = Ry_pi * R;

    // Additional correction: rotate orientation by 180 deg around robot local +Z.
    const tf2::Matrix3x3 Rz_pi(
      -1.0,  0.0, 0.0,
       0.0, -1.0, 0.0,
       0.0,  0.0, 1.0);
    R = R * Rz_pi;

    // Apply translation offset only (no axis/orientation change).
    p.setY(p.y() + robot_base_y_offset_m_);

    tf2::Quaternion q;
    R.getRotation(q);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = frame_id_;
    transform_stamped.child_frame_id = robot_base_frame_;
    transform_stamped.transform.translation.x = p.x();
    transform_stamped.transform.translation.y = p.y();
    transform_stamped.transform.translation.z = p.z();
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(transform_stamped);

    double roll, pitch, yaw_out;
    R.getRPY(roll, pitch, yaw_out);
    ROS_INFO_THROTTLE(5.0, "Robot base (camera-separated): ws pos=(%.3f, %.3f, %.3f)m, rpy=(%.2f, %.2f, %.2f) deg",
                      p.x(), p.y(), p.z(),
                      roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw_out * 180.0 / M_PI);

    return true;
  }

  bool computeRobotBaseTransform(
      const std::vector<int>& ids,
      const std::vector<std::vector<cv::Point2f>>& corners_for_camera,
      const std::vector<std::vector<cv::Point2f>>& corners_for_homography,
      const cv::Mat& H_img_to_paper,
      const ros::Time& stamp)
  {
    if (use_camera_separate_calibration_) {
      if (!has_camera_info_) {
        ROS_WARN_THROTTLE(2.0, "No camera_info yet on %s for camera-separated robot pose.",
                          camera_info_topic_.c_str());
        if (!allow_homography_fallback_for_robot_pose_) return false;
      } else if (computeRobotBaseTransformFromCamera(ids, corners_for_camera, stamp)) {
        return true;
      } else {
        ROS_WARN_THROTTLE(2.0, "Camera-separated robot pose failed.");
        if (!allow_homography_fallback_for_robot_pose_) return false;
      }
      ROS_WARN_THROTTLE(2.0, "Falling back to homography robot pose.");
    }

    return computeRobotBaseTransformFromHomography(ids, corners_for_homography, H_img_to_paper, stamp);
  }

  void imgCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat bgr_raw;
    try {
      bgr_raw = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const std::exception& e) {
      ROS_WARN("cv_bridge: %s", e.what());
      return;
    }

    cv::Mat bgr_full = bgr_raw;
    if (rotate_image_90_cw_) {
      cv::rotate(bgr_raw, bgr_full, cv::ROTATE_90_CLOCKWISE);
    }

    // Detect all ArUco markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(bgr_full, dict_, corners, ids, params_);
    std::vector<std::vector<cv::Point2f>> corners_for_camera = corners;
    if (rotate_image_90_cw_) {
      corners_for_camera = unrotateCorners90CWToOriginal(corners, bgr_raw.rows);
    }

    // Keep workspace boundary visible in RViz even when ArUco is not detected yet.
    publishWorkspaceMarker(msg->header.stamp);

    // Compute homography (pixel -> paper meters)
    cv::Mat H;
    cv::Mat dbg_aruco;
    if (!computeHomographyFromArucoCenters(bgr_full, H, dbg_aruco)) {
      // Only publish marker detection
      dbg_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", dbg_aruco).toImageMsg());
      return;
    }

    // Build a strict image mask for the calibrated workspace area.
    cv::Mat workspace_mask_full;
    std::vector<cv::Point> workspace_poly_img;
    if (!buildWorkspaceMaskInImage(H, bgr_full.size(), workspace_mask_full, workspace_poly_img)) {
      ROS_WARN_THROTTLE(1.0, "Failed to build workspace mask from ArUco homography.");
      dbg_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", dbg_aruco).toImageMsg());
      return;
    }

    cv::polylines(dbg_aruco, workspace_poly_img, true, cv::Scalar(255, 255, 0), 2);

    // Compute and publish robot base transform from markers 7 and 17
    computeRobotBaseTransform(ids, corners_for_camera, corners, H, msg->header.stamp);

    // ROI for color processing (optional)
    cv::Rect roi(0, 0, bgr_full.cols, bgr_full.rows);
    if (roi_w_ > 0 && roi_h_ > 0) {
      roi = cv::Rect(roi_x_, roi_y_, roi_w_, roi_h_) & cv::Rect(0, 0, bgr_full.cols, bgr_full.rows);
    }
    cv::Mat bgr = bgr_full(roi);
    cv::Mat workspace_mask_roi = workspace_mask_full(roi);

    // BGR -> HSV
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Threshold cyan blue.
    cv::Mat mask;
    cv::inRange(hsv,
                cv::Scalar(h_low_, s_low_, v_low_),
                cv::Scalar(h_high_, 255, 255),
                mask);

    // Hard gate: only keep blue pixels that lie inside workspace (ArUco 0,1,2,3).
    cv::bitwise_and(mask, workspace_mask_roi, mask);

    // Morphology
    int k = std::max(1, morph_ksize_);  // Make sure that the kernel size is positive
    if (k % 2 == 0) k += 1;   // Make sure that the kernel is odd
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));    // Kernel with an ellipsoid shape fits better
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    mask_pub_.publish(cv_bridge::CvImage(msg->header, "mono8", mask).toImageMsg());

    // Skeleton
    cv::Mat skel;
    cv::ximgproc::thinning(mask, skel, cv::ximgproc::THINNING_ZHANGSUEN);
    skel_pub_.publish(cv_bridge::CvImage(msg->header, "mono8", skel).toImageMsg());   // OpenCV image -> ROS image

    // Trace skeleton
    std::vector<cv::Point> ordered = traceSkeleton(skel);
    if ((int)ordered.size() < 10) {
      ROS_WARN_THROTTLE(1.0, "Skeleton too small to be traceable.");
      dbg_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", dbg_aruco).toImageMsg());
      return;
    }

    // Sample points (back to full image coords by adding ROI offset)
    std::vector<cv::Point2f> sampled_px;
    sampled_px.reserve(ordered.size() / std::max(1, sample_step_px_) + 1);

    for (int i = 0; i < (int)ordered.size(); i += std::max(1, sample_step_px_)) {
      sampled_px.emplace_back((float)(ordered[i].x + roi.x), (float)(ordered[i].y + roi.y));
    }
    if (sampled_px.size() < 2) return;

    // Pixel to paper meters using homography
    std::vector<cv::Point2f> sampled_xy;
    cv::perspectiveTransform(sampled_px, sampled_xy, H);

    // Publish Path in the workspace frame (paper with Z=0)
    nav_msgs::Path path;
    path.header.stamp = msg->header.stamp;
    path.header.frame_id = frame_id_;

    visualization_msgs::Marker mk;
    mk.header = path.header;
    mk.ns = "weld_points";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.01;
    mk.color.a = 1.0;
    mk.color.b = 1.0;

    int outside_workspace = 0;
    for (size_t i = 0; i < sampled_xy.size(); ++i) {
      const double X = sampled_xy[i].x;
      const double Y = sampled_xy[i].y;

      if (!isInsideWorkspace(X, Y, 0.005)) {
        outside_workspace++;
        continue;
      }

      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = X;
      ps.pose.position.y = Y;
      ps.pose.position.z = 0.0;   // on paper plane
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);

      geometry_msgs::Point p;
      p.x = X; p.y = Y; p.z = 0.0;
      mk.points.push_back(p);
    }

    if (outside_workspace > 0) {
      ROS_WARN_THROTTLE(2.0, "Filtered %d trajectory points outside workspace bounds", outside_workspace);
    }

    if (path.poses.size() < 2) {
      ROS_WARN_THROTTLE(1.0, "No valid trajectory points inside workspace.");
      path_pub_.publish(path);
      marker_pub_.publish(mk);
      dbg_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", dbg_aruco).toImageMsg());
      return;
    }

    path_pub_.publish(path);
    marker_pub_.publish(mk);

    // Show skeleton (green) + aruco detection
    cv::Mat dbg = dbg_aruco.clone();
    for (int y = 0; y < skel.rows; ++y) {
      const uint8_t* row = skel.ptr<uint8_t>(y);
      for (int x = 0; x < skel.cols; ++x) {
        if (row[x] > 0) {
          int xx = x + roi.x;
          int yy = y + roi.y;
          if (inBounds(xx, yy, dbg.cols, dbg.rows)) {
            dbg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 255, 0);
          }
        }
      }
    }

    // Draw sampled points (red)
    for (const auto& p : sampled_px) {
      cv::circle(dbg, p, 2, cv::Scalar(0, 0, 255), -1);
    }

    dbg_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg());
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Subscriber cam_info_sub_;

  ros::Publisher path_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher workspace_marker_pub_;

  image_transport::Publisher dbg_pub_;
  image_transport::Publisher mask_pub_;
  image_transport::Publisher skel_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string image_topic_, frame_id_;
  std::string robot_base_frame_;
  std::string camera_info_topic_;
  std::string camera_frame_id_ = "camera_optical_frame";

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool has_camera_info_ = false;
  bool use_camera_separate_calibration_ = true;
  bool allow_homography_fallback_for_robot_pose_ = false;


  // paper magnitudes
  double paper_w_m_ = 0.210;
  double paper_h_m_ = 0.297;

  // Aruco parameters
  double aruco_marker_size_ = 0.06;
  double paper_w1_margin_offset_ = 0.015;
  double paper_h1_margin_offset_ = 0.018;
  double paper_w2_margin_offset_ = 0.015;
  double paper_h2_margin_offset_ = 0.018;
  double paper_w3_margin_offset_ = 0.015;
  double paper_h3_margin_offset_ = 0.018;
  double paper_w4_margin_offset_ = 0.015;
  double paper_h4_margin_offset_ = 0.018;
  

  // aruco parameters
  int aruco_dict_id_ = (int)cv::aruco::DICT_4X4_50;
  int id_tl_ = 0, id_tr_ = 1, id_bl_ = 2, id_br_ = 3;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;

  // Robot base markers
  int id_robot_top_ = 7;
  int id_robot_bottom_ = 17;
  double robot_marker_size_ = 0.10;
  double robot_marker_distance_ = 0.20;
  double robot_base_y_offset_m_ = 0.0;

  // hsv values
  int h_low_ = 75, h_high_ = 130;
  int s_low_ = 20, v_low_ = 40;

  // roi (optional)
  int roi_x_ = 0, roi_y_ = 0, roi_w_ = 0, roi_h_ = 0;

  // morph
  int morph_ksize_ = 3;

  // sampling step
  int sample_step_px_ = 6;

  // image orientation
  bool rotate_image_90_cw_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "weld_line_vision_node");
  //ros::init(argc, argv, "weld_line_vision_aruco");
  ros::NodeHandle nh("~");
  TrajectoryGenerator node(nh);
  ros::spin();
  return 0;
}

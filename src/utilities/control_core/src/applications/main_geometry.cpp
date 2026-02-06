#include <control_core/geometry/shapes.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <control_core/geometry/geometry_3d.h>

int main(int argc, char **argv)
{
  cc::AngularPosition Q = cc::AngularPosition::Zero();
  cc::Vector3 vec = cc::logMapS3(Q);

  std::cout << "vec=" << vec << std::endl;

  cc::Vector3 w, p, cp;
  cc::Scalar b;

  w << 0, 1, 0;
  b = -1;
  p << 3, 3, 0;

  cc::Vector4 w_comp;
  w_comp << b, w;

  cp = cc::closest_point_on_line(p, w_comp);

  ROS_WARN_STREAM("cp=" << cp.toString());

  ros::init(argc, argv, "field_generator");
  ROS_INFO_STREAM("starting field_generator...");

  ros::NodeHandle nh("~");
  ros::Publisher shape_pub = nh.advertise<visualization_msgs::MarkerArray>("shapes", 1);
  visualization_msgs::MarkerArray shapes;
  std::vector<cc::Vector2> points;

  ros::Time time, prev_time;
  ros::Duration dt;
  ros::Rate rate(30);
  while (ros::ok())
  {
    // time update
    time = ros::Time::now();
    dt = time - prev_time;
    prev_time = time;

    // clear
    shapes.markers.clear();

    // generate points on a line
    points.clear();
    for (size_t i = 0; i < 10; ++i)
    {
      points.push_back((cc::Vector2() << 0.2 * i - 1, -1).finished());
      cc::PointShape point(points.back());
      shapes.markers.push_back(point.toMarkerMsg(0, 1, 0));
    }

    // fit a line
    cc::LineShape line_fit;
    line_fit.fit(points);
    shapes.markers.push_back(line_fit.toMarkerMsg(1, 1, 0));

    // generate points on a circle
    points.clear();
    points.push_back((cc::Vector2() << 0.1, 0.7).finished());
    points.push_back((cc::Vector2() << 0.2, 0.6).finished());
    points.push_back((cc::Vector2() << 0.5, 0.8).finished());
    points.push_back((cc::Vector2() << 0.7, 0.7).finished());
    points.push_back((cc::Vector2() << 0.9, 0.5).finished());
    points.push_back((cc::Vector2() << 0.3, 0.7).finished());
    for(size_t i = 0; i < points.size(); ++i)
    {
      cc::PointShape point(points[i]);
      shapes.markers.push_back(point.toMarkerMsg(0, 1, 0));
    }

    // fit a circle
    cc::CircleShape circle_fit;
    circle_fit.fit(points);
    shapes.markers.push_back(circle_fit.toMarkerMsg(1, 1, 0));

    // generate polygon
    std::vector<cc::Vector2> vertices;
    vertices.push_back((cc::Vector2() << 1.0, 1.0).finished());
    vertices.push_back((cc::Vector2() << 2.5, 1.5).finished());
    vertices.push_back((cc::Vector2() << 2.5, 2.0).finished());
    vertices.push_back((cc::Vector2() << 1.0, 2.0).finished());
    cc::PolygonShape poly(vertices);
    shapes.markers.push_back(poly.toMarkerMsg(0, 0, 1));

    // publish
    for(size_t i = 0; i < shapes.markers.size(); ++i)
    {
      shapes.markers[i].id = i;
      shapes.markers[i].ns = "shapes";
      shapes.markers[i].header.frame_id = "map";
    }
    shape_pub.publish(shapes);

    ros::spinOnce();
    rate.sleep();
  }

  //----------------------------------------------------------------------------

  return 0;
}
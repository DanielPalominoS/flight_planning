//ROS standard libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include  "std_msgs/Float64.h"
#include  "geometry_msgs/TwistStamped.h"
#include  "geometry_msgs/PoseStamped.h"
#include  "geometry_msgs/Pose.h"
#include  "geometry_msgs/Twist.h"
#include  "geometry_msgs/Vector3.h"
#include  "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include  "actionlib/client/terminal_state.h"
#include  "actionlib/client/simple_action_client.h"
#include  "sensor_msgs/Imu.h"
#include  "sensor_msgs/NavSatFix.h"
//#include  <hector_quadrotor_interface/limiters.h>
#include  "hector_uav_msgs/EnableMotors.h"
//openCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point.h>
//own generated packages libraries
#include  "hector_image_processing/find_crop_features.h"
#include  "hector_image_processing/take_save_picture.h"
#include  "hector_image_processing/image_point.h"
#include  "hector_image_processing/crop.h"
#include  "hector_waypoint_control/calculate_flight_height.h"
#include  "hector_waypoint_control/calculate_gsd.h"
#include  "hector_waypoint_control/TakeoffAction.h"
#include  "sensor_msgs/CameraInfo.h"
#include  "flight_planning/generate_plan.h"
#include  "flight_planning/georef_points.h"
#include  <GeographicLib/Geocentric.hpp>
#include  <GeographicLib/Constants.hpp>
#include  <GeographicLib/LocalCartesian.hpp>
#include  <GeographicLib/Geodesic.hpp>
#include  <boost/geometry.hpp>
namespace gl=GeographicLib;
namespace bg=boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> bg_point;
typedef bg::model::polygon<bg_point> bg_polygon;
typedef bg::model::linestring<bg_point> bg_line;

#ifndef LINE_CLASS_H
#define LINE_CLASS_H
class line_class
{
public:
  cv::Mat line_;
  cv::Point2f intercept_point;
  bool  intercpt_found;
  line_class();
  line_class(double  x1,double y1,double x2,double y2);
  line_class(double  x1,double y1,cv::Mat* vector);
  line_class(double  x1,double y1,geometry_msgs::Vector3* vector);
  /// finds the interception between the line to intercept and the line
  bool find_interception(cv::Mat* line_to_int);
  void  line_move(double x,double y);
};

#endif // LINE_CLASS_H

#ifndef FLIGHT_PLANNER_H
#define FLIGHT_PLANNER_H
const double a=gl::Constants::WGS84_a();//6378137;//Semimajor axis
const double f=gl::Constants::WGS84_f();//1/298.257223563;
const gl::Geodesic& geod=gl::Geodesic::WGS84 ();
const gl::Geocentric& ecef=gl::Geocentric::WGS84 ();
const double b=a*(1-f);//Semiminor axis
const double e2=(a*a-b*b)/(a*a);//first excentricity
const double E2=e2/(1-e2);//second excentricity
class flight_planner
{
  ros::NodeHandle nh;
  ros::ServiceServer  planner;
  ros::ServiceServer  georeferencer;
  double  node_frec;
  //ros::Subscriber gps_sub;//= nh.subscribe("/ground_truth_to_tf/pose", 100, chatterCallback);
  //ros::Subscriber imu_sub;
  geometry_msgs::Vector3  imu_uav;//Orienntation
  /*sensor_msgs::NavSatFix  uav_gps_pose;
  sensor_msgs::NavSatFix  ref_gps_pose;*/
  geometry_msgs::Vector3  uav_gps_pose;
  geometry_msgs::Vector3  ref_gps_pose;
  double uav_alt;
  //Image and camera variables
  double i_height;//image height
  double i_width;//image width
  double s_height;//sensor height
  double s_width;//sensor width
  double fl;//focal lenght=0.0088;//m
  double focal_plane_res;//=i_width/s_width;//pixel_per_m
  double fl_pixel;//=focal_plane_res*focal_lenght;
  double scale_number;//Scale S=uav_alt/fl_pixel;
  double image_scale;//image scale s=1/scale_number;
  double cam_ori;
  double  flight_height;
  cv::Mat k;
  cv::Mat dist_coeffs;
  cv::Mat pix_coor;
  //Photogrammetric parameters
  double  sidelap;
  double  endlap;
  double  gsd;
  double  base;
  double  line_space;
  bool  are_degrees;
  bool  debug_;
  gl::LocalCartesian  navf_;
  //crop
  hector_image_processing::crop crop_f;
public:
  flight_planner();
  ///Object destructor
  ~flight_planner();
  bool  georeferencerCB(flight_planning::georef_points::Request &req,
                        flight_planning::georef_points::Response &res);
  ///call back that handles the flight planning
  bool plannerCB(flight_planning::generate_plan::Request &req,
                 flight_planning::generate_plan::Response &res);
  void  generate_points(std::vector<geometry_msgs::Vector3>* navf_points,std::vector<line_class>* lines,
                        geometry_msgs::Vector3* row_vector_,double x_max,double y_max,double x_min,double y_min,
                        geometry_msgs::Vector3* side_vector_,double base, double line_space,double lines_number,
                        uint first_point_index, bool debug,std::vector<std::vector<geometry_msgs::Vector3> >* waypoints);
  void find_max_min_x_y(std::vector<geometry_msgs::Vector3> points, cv::Mat* max_vector,
                        cv::Mat* min_vector);
  ///call back that handles the process to georeference a point

  uint  find_initial_point_and_side_vector(std::vector<geometry_msgs::Vector3>* navf_points,std::vector<line_class>* lines,
                                           geometry_msgs::Vector3* row_vector,double x_max,double y_max,double x_min,double y_min,
                                           geometry_msgs::Vector3* side_vector, double line_space,uint* line_number, bool debug);
  void  rotate_vetor(geometry_msgs::Vector3* orig ,geometry_msgs::Vector3* rotated,
                     double magnitude, double  rotation,bool are_degrees);
  void  normalize_2d(geometry_msgs::Vector3 *vector);
  double calculate_height(double fl,double gsd,double sw,double iw);
  double  calculate_gsd(double fl,double fh,double sw,double iw);
  void  polar2cart(geometry_msgs::Vector3 *vector,double r,double theta,bool is_deg);
  /// caclulates cartesian distance between two points p1, p2
  double  calculate_distance(double x1,double y1,double x2, double  y2);
  /// converts gps_coord(sensor_msgs/Vector3) to cv_Mat column vector
  void  vector3_2cvMat(geometry_msgs::Vector3  gps,cv::Mat* gps_mat);
  void  cvMat2vector3(geometry_msgs::Vector3  *gps,cv::Mat gps_mat);
  /// converts cv_Mat column vector to gps_coord(sensor_msgs/Vector3)
  void update_k(cv::Mat *k,double fx,double fy,double cx,double cy,double skew);
  ///updates rotation matrix from camera frame to body frame(uav)
  /// cam ori is the angle between flight direction and camera width
  void update_CF2BF(double cam_ori,cv::Mat* CF2BF,bool is_degrees);
  ///updates rotation matrix from body frame to navigation frame
  void update_BF2NF(geometry_msgs::Vector3 IMU_ori,cv::Mat* BF2NF,bool is_degrees);
  ///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void  enu2ned(geometry_msgs::Vector3* coord);
  ///converts geodetic coors(lat,lon,alt) to navigation frame
  void geod2nf(geometry_msgs::Vector3 uav_gps_posi,geometry_msgs::Vector3 ref_gps_posi,
                          geometry_msgs::Vector3* navf_point, bool is_degres,bool debug);
  void if2navf(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                          cv::Mat k,geometry_msgs::Vector3 image_coord, geometry_msgs::Vector3 ref_gps_posi,
                          geometry_msgs::Vector3* navf_point, bool is_degrees, double scale,double fl,bool debug);
  void navf2if(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
               cv::Mat k, cv::Mat navf_coord, geometry_msgs::Vector3 ref_gps_posi,
               geometry_msgs::Vector3* image_coord, bool is_degrees, double scale, double fl, bool debug);
  void georef_navf_point(geometry_msgs::Vector3 navf_coord, geometry_msgs::Vector3 ref_gps_posi,
                          geometry_msgs::Vector3* gps_coord, bool is_degrees, bool debug);
  ///given the image coordinates, uav pose and camera info, returns the geodetic coordinates
  void georef_image_point(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                          cv::Mat k,geometry_msgs::Vector3 image_coord, geometry_msgs::Vector3 ref_gps_posi,
                          geometry_msgs::Vector3* georeferenced_point, bool is_degrees, double scale, double fl, bool debug);
};

#endif // FLIGHT_PLANNER_H

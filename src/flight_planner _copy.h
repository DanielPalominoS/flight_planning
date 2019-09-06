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
//hector uav librarires
#include  "hector_uav_msgs/YawrateCommand.h"
#include  "hector_uav_msgs/ThrustCommand.h"
#include  "hector_uav_msgs/AttitudeCommand.h"
#include  "hector_uav_msgs/TakeoffAction.h"
#include  "hector_uav_msgs/EnableMotors.h"
#include  "hector_uav_msgs/LandingAction.h"
#include  "hector_uav_msgs/PoseAction.h"
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
  /// finds the interception between the line to intercept and the line
  bool find_interception(cv::Mat* line_to_int);
  void  line_move(double x,double y);
};

#endif // LINE_CLASS_H

#ifndef FLIGHT_PLANNER_H
#define FLIGHT_PLANNER_H

const double a=6378137;//Semimajor axis
const double f=1/298.257223563;
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
  //crop
  hector_image_processing::crop crop_f;
public:
  flight_planner();
  ///Object destructor
  ~flight_planner();
  ///call back that handles the flight planning
  bool plannerCB(flight_planning::generate_plan::Request &req,
                 flight_planning::generate_plan::Response &res);
  void  generate_points(std::vector<cv::Mat>* navf_points,std::vector<line_class>* lines,
                        cv::Mat* row_vector,double x_max,double y_max,double x_min,double y_min,
                        cv::Mat* side_vector,double base, double line_space,double lines_number,
                        uint first_point_index, bool debug,std::vector<std::vector<cv::Mat> >* waypoints);
  void find_max_min_x_y(std::vector<cv::Mat>  points,cv::Mat* max_vector,
                        cv::Mat* min_vector);
  ///call back that handles the process to georeference a point
  bool  georeferencerCB(flight_planning::georef_points::Request &req,
                        flight_planning::georef_points::Response &res);
  uint  find_initial_point_and_side_vector(std::vector<cv::Mat>* navf_points,std::vector<line_class>* lines,
                                           cv::Mat* row_vector,double x_max,double y_max,double x_min,double y_min,
                                           cv::Mat* side_vector, double line_space,uint* line_number, bool debug);
  void  rotate_vetor(cv::Mat* orig ,cv::Mat* rotated,double magnitude, double  rotation,bool are_degrees);
  double calculate_height(double fl,double gsd,double sw,double iw);
  double  calculate_gsd(double fl,double fh,double sw,double iw);
  void  polar2cart(double *x,double* y,double r,double theta,bool is_deg);
  void  polar2cart(cv::Mat *vector,double r,double theta,bool is_deg);
  /// caclulates cartesian distance between two points p1, p2
  double  calculate_distance(double x1,double y1,double x2, double  y2);
  ///converts imu coord(sensor_msgs/Vector3) to cv_Mat column vector;
  void  imu2cvMat(geometry_msgs::Vector3 imu,cv::Mat* imu_mat);
  /// converts gps_coord(sensor_msgs/Vector3) to cv_Mat column vector
  void  gps2cvMat(geometry_msgs::Vector3  gps,cv::Mat* gps_mat);
  void  cvMat2gps(sensor_msgs::NavSatFix  *gps,cv::Mat* gps_mat);
  /// converts cv_Mat column vector to gps_coord(sensor_msgs/Vector3)
  void  cvMat2gps(geometry_msgs::Vector3  *gps,cv::Mat* gps_mat);  ///updates intrinsic parameters matrix
  void update_k(cv::Mat *k,double fx,double fy,double cx,double cy,double skew);
  ///updates rotation matrix from camera frame to body frame(uav)
  /// cam ori is the angle between flight direction and camera width
  void update_CF2BF(double cam_ori,cv::Mat* CF2BF,bool is_degrees);
  ///updates rotation matrix from body frame to navigation frame(uav)
  void update_BF2NF2(cv::Mat IMU_ori,cv::Mat* BF2NF,bool is_degrees);
  ///updates rotation matrix from body frame to navigation frame
  void update_BF2NF(cv::Mat IMU_ori,cv::Mat* BF2NF,bool is_degrees);
  ///updates rotation matrix from navigation frame to ECEF frame
  void update_NF2ECEF(cv::Mat gps_position,cv::Mat* NF2ECEF,bool is_degrees);
  ///updates rotation matrix from ECEF frame to navigation frame
  void update_ECEF2NF(cv::Mat gps_position,cv::Mat* NF2ECEF,bool is_degrees);
  ///updates rotation matrix from ECEF frame to navigaction frame
  void update_ECEF2NF_2(cv::Mat gps_position,cv::Mat* NF2ECEF,bool is_degrees);
  ///converts geodetic coors(lat,lon,alt) to ecef coordinates
  void geod2ecef(cv::Mat gps_position,cv::Mat *uav_ecef_posi,bool is_degrees);
  ///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_4(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi);
  ///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_1(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi);
///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_2(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi);
  ///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_3(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi);
  void  normalize_2d(cv::Mat vector);
  void  normalize_3d(cv::Mat vector);
  ///converts geodetic coors(lat,lon,alt) to navigation frame
  void geod2nf(cv::Mat uav_gps_posi, cv::Mat ref_gps_posi,
                          cv::Mat* navf_point, bool is_degrees,bool debug);
  void if2navf(double cam_ori, cv::Mat imu_ori, cv::Mat uav_gps_posi,
                          cv::Mat k,cv::Mat image_coord, cv::Mat ref_gps_posi,
                          cv::Mat* navf_point, bool is_degrees, double scale, double fl,bool debug);
  void navf2if(double cam_ori, cv::Mat imu_ori,
                          cv::Mat k,cv::Mat navf_coord,
                          cv::Mat* image_point, bool is_degrees, double scale, double fl,bool debug);
  void georef_nav_point( cv::Mat navf_coord, cv::Mat ref_gps_posi,
                          cv::Mat* georeferenced_point, bool is_degrees, bool debug);
  ///given the image coordinates, uav pose and camera info, returns the geodetic coordinates
  void georef_image_point(double cam_ori, cv::Mat imu_ori, cv::Mat uav_gps_posi,
                          cv::Mat k,cv::Mat image_coord, cv::Mat ref_gps_posi,
                          cv::Mat* georeferenced_point, bool is_degrees, double scale, double fl, bool debug);
};

#endif // FLIGHT_PLANNER_H

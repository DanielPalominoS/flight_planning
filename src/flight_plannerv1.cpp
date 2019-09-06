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
const double a=6378137;//Semimajor axis
const double f=1/298.257223563;
const double b=a*(1-f);//Semiminor axis
const double e2=(a*a-b*b)/(a*a);//first excentricity
const double E2=e2/(1-e2);//second excentricity

class FlightPlanner
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
  //crop
  hector_image_processing::crop crop_f;


public:
  FlightPlanner()
    :i_width(1280)
  {
    //i_width=1280;
    i_height=960;
    s_width=0.0088;
    s_height=0.0132;
    fl=0.0088;
    focal_plane_res=i_width/s_width;
    fl_pixel=focal_plane_res*fl;
    sidelap=0.3;
    endlap=0.6;
    are_degrees=true;
    k=cv::Mat(3,3,CV_64F);
    //k=cv::Mat::zeros(3,CV_64F);

    planner=nh.advertiseService("flight_planning/plan_flight", &FlightPlanner::plannerCB,this);
    georeferencer=nh.advertiseService("flight_planning/georef_points", &FlightPlanner::georeferencerCB,this);
  }
  ///Object destructor
  ~FlightPlanner()
  {

  }


  ///call back that handles the flight planning
  bool plannerCB(flight_planning::generate_plan::Request &req,
                 flight_planning::generate_plan::Response &res){
    i_width=req.i_width;
    i_height=req.i_height;
    s_width=req.s_width;
    gsd=req.gsd;
    fl=req.fl;
    update_k(&k,req.fx,req.fy,req.cx,req.cy,req.skew);
    uav_gps_pose=req.uav_gps;
    ref_gps_pose=req.ref_gps;
    imu_uav=req.uav_Ori;
    cam_ori=req.cam_ori;
    are_degrees=req.are_degrees;
    cv::Mat imu_ori=cv::Mat(3,1,CV_64F);
    cv::Mat uav_gps=cv::Mat(3,1,CV_64F);
    cv::Mat ref_gps=cv::Mat(3,1,CV_64F);
    std::vector<cv::Mat> crop_pixel_coord;
    std::vector<cv::Mat> plan_pixel_coord;
    cv::Mat  georef_point=cv::Mat(3,1,CV_64F);
    cv::Mat  first_point_navf=cv::Mat(3,1,CV_64F);

    cv::Mat crop_row_vector;


    //cv::Mat  pix_coor=cv::Mat(3,1,CV_64F);
    //inicializar variables
    uav_alt=uav_gps_pose.z-ref_gps_pose.z;
    focal_plane_res=i_width/s_width;
    fl_pixel=focal_plane_res*fl;
    scale_number=uav_alt/fl_pixel;
    image_scale=1/scale_number;
    imu2cvMat(imu_uav,&imu_ori);
    gps2cvMat(uav_gps_pose,&uav_gps);
    gps2cvMat(ref_gps_pose,&ref_gps);
    uint  numCropPoints=req.crop.contour.size();
    crop_pixel_coord.resize(numCropPoints);
    double  crop_height;
    double  crop_width;
    double  crop_area;

    double  gr_height_cover;
    double  gr_width_cover;
    
    uint nfl;//Number of flight lines
    uint ipfl;//number images per flight line
    double  fld;//first line direction

    endlap=req.endlap;
    sidelap=req.sidelap;

    for(int i=0;i<numCropPoints;i++){
      crop_pixel_coord.at(i)=cv::Mat(3,1,CV_64F);
      crop_pixel_coord.at(i).row(0).col(0)=req.crop.contour.at(i).x;
      crop_pixel_coord.at(i).row(1).col(0)=req.crop.contour.at(i).y;
      crop_pixel_coord.at(i).row(2).col(0)=fl_pixel;
    }

    std::vector<cv::Mat>  crop_gps_coors;
    crop_gps_coors.resize(numCropPoints);
    res.crop_gps_coors.resize(numCropPoints);
    for(int i=0;i<numCropPoints;i++){
      //pixel_coord.at(i).copyTo(pix_coor);
      //georef_image_point(cam_ori,imu_ori,uav_gps,k,pix_coor,ref_gps,&georef_point,are_degrees,scale_number,fl);
      georef_image_point(cam_ori,imu_ori,uav_gps,k,crop_pixel_coord.at(i),ref_gps,
                         &georef_point,are_degrees,scale_number,fl);
      georef_point.copyTo(crop_gps_coors.at(i));
      cvMat2gps(&res.crop_gps_coors[i],&georef_point);
    }
    uint  initial_point_index;
    initial_point_index= find_initial_point(&crop_gps_coors,ref_gps,are_degrees);
    std::cout<<"inicial point"<<crop_gps_coors.at(initial_point_index)<<std::endl;
    /*calculate_crop_h_w(&crop_height,&crop_width,scale_number,
                       req.crop.contour[0].x,req.crop.contour[0].y,
                       req.crop.contour[1].x,req.crop.contour[1].y,
                       req.crop.contour[2].x,req.crop.contour[2].y,
                       req.crop.row_orientation);*/
    calculate_crop_h_w_fld(&crop_height,&crop_width,&fld,scale_number,initial_point_index,req.crop);
    crop_area=crop_height*crop_width;

    std::cout<<"image height\n"<<i_height<<std::endl;
    std::cout<<"image width\n"<<i_width<<std::endl;
    std::cout<<"gsd\n"<<gsd<<std::endl;

    std::cout<<"crop_height\n"<<crop_height<<std::endl;
    std::cout<<"crop_width\n"<<crop_width<<std::endl;
    res.crop_area=crop_area;
    std::cout<<"crop_area\n"<<crop_area<<std::endl;
    /*********************/
    gr_height_cover=i_height*gsd;
    std::cout<<"ground height coverrage\n"<<gr_height_cover<<std::endl;
    gr_width_cover=i_width*gsd;
    std::cout<<"ground width coverrage\n"<<gr_width_cover<<std::endl;
    base=gr_height_cover*(1-endlap/100);
    std::cout<<"base\n"<<base<<std::endl;
    line_space=gr_width_cover*(1-sidelap/100);
    std::cout<<"line space\n"<<line_space<<std::endl;
    nfl=(uint)ceil(crop_width/line_space)+1;
    std::cout<<"number of flight lines\n"<<nfl<<std::endl;
    ipfl=(uint)ceil(crop_height/base)+1;
    std::cout<<"images per line\n"<<ipfl<<std::endl;


    std::vector<std::vector<geometry_msgs::Vector3> > waypoints_navf;
    std::vector<std::vector<geometry_msgs::Vector3> > waypoints_geod;
    std::vector<std::vector<geometry_msgs::Vector3> > waypoints_image;
    double forw_step_x,forw_step_y;
    double back_step_x,back_step_y;
    double side_step_x,side_step_y;

    polar2cart(&forw_step_x,&forw_step_y,base,fld,true);
    polar2cart(&back_step_x,&back_step_y,base,fld+180,true);
    polar2cart(&side_step_x,&side_step_y,line_space,fld+90,true);//Hay que tener en cuenta si se van a generar los puntos en sentido horario o antihorario;

    waypoints_navf.resize(nfl);
    for(int i=0;i<nfl;i++){
      waypoints_navf.at(i).resize(ipfl+4);//se recomienda colocar 4 waypoints adicionales para que el uav se alinie adecuadamente cuando cambia de linea
    }
    //georef_point;
    geod2nf(crop_gps_coors.at(initial_point_index),ref_gps,&first_point_navf,are_degrees);

    std::cout<<"first point navf"<<first_point_navf<<std::endl;

    //Inicialización primera linea
    waypoints_navf.at(0).at(0).x=2*back_step_x +first_point_navf.at<double>(0,0);//falta sumar la posición del primer punto en el frame de navegación
    waypoints_navf.at(0).at(0).y=2*back_step_y+first_point_navf.at<double>(1,0);
    waypoints_navf.at(0).at(0).z=10;
    waypoints_navf.at(0).at(1).x=back_step_x+first_point_navf.at<double>(0,0);
    waypoints_navf.at(0).at(1).y=back_step_y+first_point_navf.at<double>(1,0);
    waypoints_navf.at(0).at(1).z=10;
    for(int j=2;j<ipfl+4;j++){
        waypoints_navf.at(0).at(j).x=forw_step_x+waypoints_navf.at(0).at(j-1).x;
        waypoints_navf.at(0).at(j).y=forw_step_y+waypoints_navf.at(0).at(j-1).y;
        waypoints_navf.at(0).at(j).z=10;
    }

    std::cout<<"plan: "<<std::endl;
    for(int i=1;i<nfl;i++){
      if(i%2==0){
        waypoints_navf.at(i).at(0).x=waypoints_navf.at(i-1).at(ipfl+4-1).x+side_step_x;//al ultimo punto de la linea anterior se le suma el desplazamiento vertical
        waypoints_navf.at(i).at(0).y=waypoints_navf.at(i-1).at(ipfl+4-1).y+side_step_y;
        waypoints_navf.at(i).at(0).z=10;
        for(int j=1;j<ipfl+4;j++){
          waypoints_navf.at(i).at(j).x=forw_step_x+waypoints_navf.at(i).at(j-1).x;
          waypoints_navf.at(i).at(j).y=forw_step_y+waypoints_navf.at(i).at(j-1).y;
          waypoints_navf.at(i).at(j).z=10;
        }

      }
      else{
        waypoints_navf.at(i).at(0).x=waypoints_navf.at(i-1).at(ipfl+4-1).x+side_step_x;//al ultimo punto de la linea anterior se le suma el desplazamiento vertical
        waypoints_navf.at(i).at(0).y=waypoints_navf.at(i-1).at(ipfl+4-1).x+side_step_y;
        waypoints_navf.at(i).at(0).z=10;
        for(int j=1;j<ipfl+4;j++){
          waypoints_navf.at(i).at(j).x=back_step_x+waypoints_navf.at(i).at(j-1).x;
          waypoints_navf.at(i).at(j).y=back_step_y+waypoints_navf.at(i).at(j-1).y;
          waypoints_navf.at(i).at(j).z=10;
        }
      }

    }
    for(int i=0;i<nfl;i++){
      for(int j=0;j<ipfl+4;j++){
        std::cout<<waypoints_navf[i].at(j)<<std::endl;
      }

    }
    //std::cout<<"plan: "<<waypoints_navf.data()->size()<<std::endl;
    //req.crop_area=(req.crop.contour.at(0).x)*scale_number*()*scale_number;
    return true;
  }

  ///call back that handles the process to georeference a point
  bool  georeferencerCB(flight_planning::georef_points::Request &req,
                        flight_planning::georef_points::Response &res){
    uint  numPoints=req.im_coors.size();    

    i_width=req.i_width;
    i_height=req.i_height;
    s_width=req.s_width;
    fl=req.fl;
    update_k(&k,req.fx,req.fy,req.cx,req.cy,req.skew);
    uav_gps_pose=req.uav_gps;
    ref_gps_pose=req.ref_gps;
    imu_uav=req.uav_Ori;
    cam_ori=req.cam_ori;
    are_degrees=req.are_degrees;
    cv::Mat imu_ori=cv::Mat(3,1,CV_64F);
    cv::Mat uav_gps=cv::Mat(3,1,CV_64F);
    cv::Mat ref_gps=cv::Mat(3,1,CV_64F);
    std::vector<cv::Mat> pixel_coord;
    cv::Mat  georef_point=cv::Mat(3,1,CV_64F);
    //cv::Mat  pix_coor=cv::Mat(3,1,CV_64F);
    //inicializar variables
    uav_alt=uav_gps_pose.z-ref_gps_pose.z;
    focal_plane_res=i_width/s_width;
    fl_pixel=focal_plane_res*fl;
    scale_number=uav_alt/fl_pixel;
    image_scale=1/scale_number;
    imu2cvMat(imu_uav,&imu_ori);
    gps2cvMat(uav_gps_pose,&uav_gps);
    gps2cvMat(ref_gps_pose,&ref_gps);
    pixel_coord.resize(numPoints);
    for(int i=0;i<numPoints;i++){
      pixel_coord.at(i)=cv::Mat(3,1,CV_64F);
      pixel_coord.at(i).col(0).row(0)=req.im_coors[i].x;
      pixel_coord.at(i).row(1).col(0)=req.im_coors[i].y;
      pixel_coord.at(i).row(2).col(0)=fl_pixel;
    }
    res.gps_coors.resize(numPoints);
    for(int i=0;i<numPoints;i++){
      //pixel_coord.at(i).copyTo(pix_coor);
      //georef_image_point(cam_ori,imu_ori,uav_gps,k,pix_coor,ref_gps,&georef_point,are_degrees,scale_number,fl);
      georef_image_point(cam_ori,imu_ori,uav_gps,k,pixel_coord.at(i),ref_gps,
                         &georef_point,are_degrees,scale_number,fl);
      cvMat2gps(&res.gps_coors[i],&georef_point);
    }



    return  true;
  }
  ///function that calculates the height and width oc identified crop according to crop orientation
  void  calculate_crop_h_w(double* crop_h,double*  crop_w,double scale,double x1,double y1,
                           double x2,double y2,double x3,double y3,double crop_ori){

    double  a;
    double  b;
    double  theta1=atan2(-abs(y2-y1),abs(x2-x1))*180/CV_PI+90;
    double  theta2=atan2(-abs(y3-y2),abs(x3-x2))*180/CV_PI+90;
    double  thres=5;

    if(abs(theta1-crop_ori)<thres){
      *crop_h=calculate_distance(x1,y1,x2,y2);
      *crop_w=calculate_distance(x2,y2,x3,y3);
    }
    else{
      *crop_w=calculate_distance(x1,y1,x2,y2);
      *crop_h=calculate_distance(x2,y2,x3,y3);
    }
    std::cout<<"scale:\n"<<scale<<std::endl;
    std::cout<<"crop h(pixels):\n"<<*crop_h<<std::endl;
    std::cout<<"crop w(pixels):\n"<<*crop_w<<std::endl;
    *crop_h*=scale;
    *crop_w*=scale;
    return;
  }
  ///calculates crop's height and width, and the first line direction (degrees)
  /// index reffers to the first point identified index
  void  calculate_crop_h_w_fld(double* crop_h,double*  crop_w,double  *first_line_dir,double scale,uint index,hector_image_processing::crop  crop){
    double  x1,x2,x3,y1,y2,y3;
    double  a;
    double  b;
    double  crop_ori=crop.row_orientation;

    double  thres=5;

    if(index==0){
      x2=crop.contour.at(index).x;
      y2=-crop.contour.at(index).y;
      x1=crop.contour.at(crop.contour.size()-1).x;
      y1=-crop.contour.at(crop.contour.size()-1).y;
      x3=crop.contour.at(index+1).x;
      y3=-crop.contour.at(index+1).y;
    }
    else if(index==crop.contour.size()-1){
      x2=crop.contour.at(index).x;
      y2=-crop.contour.at(index).y;
      x1=crop.contour.at(index-1).x;
      y1=-crop.contour.at(index-1).y;
      x3=crop.contour.at(0).x;
      y3=-crop.contour.at(0).y;
    }
    else{
      x2=crop.contour.at(index).x;
      y2=-crop.contour.at(index).y;
      x1=crop.contour.at(index-1).x;
      y1=-crop.contour.at(index-1).y;
      x3=crop.contour.at(index+1).x;
      y3=-crop.contour.at(index+1).y;

    }
    std::cout<<"crop gps: "<<crop<<std::endl;
    std::cout<<"x1: "<<x1<<" x2: "<<x2<<" x3: "<<x3<<std::endl;
    std::cout<<"y1: "<<y1<<" y2: "<<y2<<" y3: "<<y3<<std::endl;
    double  theta1=-atan2(-abs(y1-y2),abs(x1-x2))*180/CV_PI;//Se opera de esta forma debido  a las coordenadas de ima imagen, donde el eje y es positivo al lado de abajo
    double  theta2=-atan2(-abs(y3-y2),abs(x3-x2))*180/CV_PI;
    if(theta1<0){theta1+=360;}
    if(theta2<0){theta2+=360;}
    std::cout<<"theta 1:\n"<<theta1<<std::endl;
    std::cout<<"theta 2:\n"<<theta2<<std::endl;
    //double  first_line_ori;

    if(abs(theta1-crop_ori)<thres){
      std::cout<<"1 "<<std::endl;
      *crop_h=calculate_distance(x1,y1,x2,y2);
      *crop_w=calculate_distance(x2,y2,x3,y3);
      *first_line_dir=atan2(y1-y2,x1-x2)*180/CV_PI;
    }
    else{
      std::cout<<"2 "<<std::endl;
      *crop_w=calculate_distance(x1,y1,x2,y2);
      *crop_h=calculate_distance(x2,y2,x3,y3);
      *first_line_dir=atan2(y3-y2,x3-x2)*180/CV_PI;
      //first_line_ori=atan2(-abs(y3-y2),abs(x3-x2))*180/CV_PI;
    }

    std::cout<<"first line ori "<<first_line_dir<<std::endl;
    std::cout<<"scale:\n"<<scale<<std::endl;
    std::cout<<"crop h(pixels):\n"<<*crop_h<<std::endl;
    std::cout<<"crop w(pixels):\n"<<*crop_w<<std::endl;
    *crop_h*=scale;
    *crop_w*=scale;
    return;
  }

  ///function that finds the initial point of the flight plan
  /// the initial point is chosen as the fartest point frome the re_gps given
  ///
  uint  find_initial_point(std::vector<cv::Mat>* crop_gps_coords,cv::Mat  ref_gps,bool are_degrees){
    uint  index;
    std::vector<cv::Mat>  navf_coords;
    cv::Mat ref_navf_coor;
    geod2nf(ref_gps,ref_gps,&ref_navf_coor,are_degrees);
    std::vector<double> dist;
    navf_coords.resize(crop_gps_coords->size());
    dist.resize(crop_gps_coords->size());
    std::cout<<"size "<<crop_gps_coords->size()<<std::endl;

    for(int i=0;i<crop_gps_coords->size();i++){
      geod2nf(crop_gps_coords->at(i),ref_gps,&navf_coords.at(i),are_degrees);
      dist.at(i)=calculate_distance(ref_navf_coor.at<double>(0,0),ref_navf_coor.at<double>(1,0),
                                    crop_gps_coords->at(i).at<double>(0,0),crop_gps_coords->at(i).at<double>(1,0));
    }

    index=0;

    for(int i=1;i<crop_gps_coords->size();i++){
      if(dist.at(i)>dist.at(index)){
        index=i;
      }
    }
    for(int i=0;i<crop_gps_coords->size();i++){
      std::cout<<"dist "<<i<<": "<<dist.at(i)<<std::endl;
    }
    std::cout<<"index "<<index<<std::endl;

    //index=
    return  index;
  }

  void  polar2cart(double *x,double* y,double r,double theta,bool is_deg){
    if(is_deg){
      theta*=CV_PI/180;
    }
    *x=r*cos(theta);
    *y=r*sin(theta);
    return;
  }

  /// caclulates cartesian distance between two points p1, p2
  double  calculate_distance(double x1,double y1,double x2, double  y2){
    double  dist=sqrt(pow(x2-x1,2)+pow(y2-y1,2));
    return  dist;
  }

  ///converts imu coord(sensor_msgs/Vector3) to cv_Mat column vector;
  void  imu2cvMat(geometry_msgs::Vector3 imu,cv::Mat* imu_mat){
    imu_mat->at<double>(0,0)=imu.x;
    imu_mat->at<double>(1,0)=imu.y;
    imu_mat->at<double>(2,0)=imu.z;
    return;
  }
  /*void  gps2cvMat(sensor_msgs::NavSatFix  gps,cv::Mat* gps_mat){
    gps_mat->col(0).row(0)=gps.latitude;
    gps_mat->col(1).row(0)=gps.longitude;
    gps_mat->col(2).row(0)=gps.altitude;
    return;
  }*/

  /// converts gps_coord(sensor_msgs/Vector3) to cv_Mat column vector
  void  gps2cvMat(geometry_msgs::Vector3  gps,cv::Mat* gps_mat){
      gps_mat->row(0).col(0)=gps.x;
      gps_mat->row(1).col(0)=gps.y;
      gps_mat->row(2).col(0)=gps.z;
      return;
    }

  void  cvMat2gps(sensor_msgs::NavSatFix  *gps,cv::Mat* gps_mat){
    gps->latitude=gps_mat->at<double>(0,0);
    gps->longitude=gps_mat->at<double>(1,0);
    gps->altitude=gps_mat->at<double>(2,0);
    return;
  }

  /// converts cv_Mat column vector to gps_coord(sensor_msgs/Vector3)
  void  cvMat2gps(geometry_msgs::Vector3  *gps,cv::Mat* gps_mat){
    gps->x=gps_mat->at<double>(0,0);
    gps->y=gps_mat->at<double>(1,0);
    gps->z=gps_mat->at<double>(2,0);
    return;
  }

  ///updates intrinsic parameters matrix
  void update_k(cv::Mat *k,double fx,double fy,double cx,double cy,double skew){
      k->row(0).col(0)=fx;
      k->row(0).col(1)=skew;
      k->row(0).col(2)=cx;
      k->row(1).col(0)=0;
      k->row(1).col(1)=fy;
      k->row(1).col(2)=cy;
      k->row(2).col(0)=0;
      k->row(2).col(1)=0;
      k->row(2).col(2)=1;
      return;
  }

  void help()
  {
      std::cout << "\nThis program undistort an image using intrinsic parameters\n"
           "Usage:\n"
           "./undistort <NIR_image_name> <RED_image_name>   , Default is pic1.jpg\n" << std::endl;
  }

  ///updates rotation matrix from camera frame to body frame(uav)
  /// cam ori is the angle between flight direction and camera width
  void update_CF2BF(double cam_ori,cv::Mat* CF2BF,bool is_degrees){
      if(is_degrees){
          cam_ori*=M_PI/180;
      }
      //std::cout<<"UpdatingCF2BF\ncam ori:"<< cam_ori<<std::endl;
      cv::Mat Rz=cv::Mat::zeros(3,3,CV_64F);
      Rz.row(2).col(2)=1;
      Rz.row(0).col(0)=cos(cam_ori);
      //std::cout<<"cos(cam_ori):"<< cos(cam_ori)<<std::endl;
      Rz.row(0).col(1)=-sin(cam_ori);
      Rz.row(1).col(0)=sin(cam_ori);
      Rz.row(1).col(1)=cos(cam_ori);
      //std::cout<<"rz:"<< Rz<<std::endl;
      *CF2BF=Rz;
      //std::cout<<"cf2bf:"<< CF2BF<<std::endl;
      return;
  }

  ///updates rotation matrix from body frame to navigation frame(uav)
  void update_BF2NF2(cv::Mat IMU_ori,cv::Mat* BF2NF,bool is_degrees){
      /***********
       * yaw: orientation.x
       * pitch: orientation.y
       * roll: orientation.z
      ***********/
      double y,p,r;
      if(is_degrees){
          y=IMU_ori.at<double>(0,0)*M_PI/180;
          p=IMU_ori.at<double>(1,0)*M_PI/180;
          r=IMU_ori.at<double>(2,0)*M_PI/180;
      }

      else{
          y=IMU_ori.at<double>(0,0);
          p=IMU_ori.at<double>(1,0);
          r=IMU_ori.at<double>(2,0);
      }
      cv::Mat tf=cv::Mat::zeros(3,3,CV_64F);
      tf.row(0).col(0)=cos(y)*cos(r)+sin(y)*sin(p)*sin(r);
      tf.row(0).col(1)=sin(y)*cos(p);
      tf.row(0).col(2)=cos(y)*sin(r)-sin(y)*sin(p)*cos(r);
      tf.row(1).col(0)=-sin(y)*cos(r)+cos(y)*sin(p)*sin(r);
      tf.row(1).col(1)=cos(y)*sin(p);
      tf.row(1).col(2)=-sin(y)*sin(r)-cos(y)*sin(p)*cos(r);
      tf.row(2).col(0)=-cos(p)*sin(r);
      tf.row(2).col(1)=sin(p);
      tf.row(2).col(2)=cos(p)*cos(r);
      *BF2NF=tf;
      return;
  }

  ///updates rotation matrix from body frame to navigation frame
  void update_BF2NF(cv::Mat IMU_ori,cv::Mat* BF2NF,bool is_degrees){
      /***********
       * yaw: orientation.x
       * pitch: orientation.y
       * roll: orientation.z
      ***********/
      double y,p,r;
      if(is_degrees){
          y=IMU_ori.at<double>(0,0)*M_PI/180;
          p=IMU_ori.at<double>(1,0)*M_PI/180;
          r=IMU_ori.at<double>(2,0)*M_PI/180;
      }

      else{
          y=IMU_ori.at<double>(0,0);
          p=IMU_ori.at<double>(1,0);
          r=IMU_ori.at<double>(2,0);
      }
      cv::Mat tf=cv::Mat::zeros(3,3,CV_64F);
      tf.row(0).col(0)=cos(p)*cos(y);
      tf.row(1).col(0)=cos(p)*sin(y);
      tf.row(2).col(0)=-sin(p);
      tf.row(0).col(1)=-cos(r)*sin(y)+sin(r)*sin(p)*cos(y);
      tf.row(1).col(1)=cos(r)*cos(y)+cos(r)*sin(p)*sin(y);
      tf.row(2).col(1)=sin(r)*cos(p);
      tf.row(0).col(2)=sin(r)*sin(y)+cos(r)*sin(p)*cos(y);
      tf.row(1).col(2)=-sin(r)*cos(y)+cos(r)*sin(p)*sin(y);
      tf.row(2).col(2)=cos(r)*cos(p);

      *BF2NF=tf;
      return;
  }

  ///updates rotation matrix from navigation frame to ECEF frame
  void update_NF2ECEF(cv::Mat gps_position,cv::Mat* NF2ECEF,bool is_degrees){
      double lat,lon;
      if(is_degrees){
          lat=gps_position.at<double>(0,0)*M_PI/180;
          lon=gps_position.at<double>(1,0)*M_PI/180;
      }
      else{
          lat=gps_position.at<double>(0,0);
          lon=gps_position.at<double>(1,0);
      }
      cv::Mat tf=cv::Mat::zeros(3,3,CV_64F);
      tf.row(0).col(0)=-sin(lat);
      tf.row(0).col(1)=cos(lat);
      tf.row(1).col(0)=-sin(lon)*cos(lat);
      tf.row(1).col(1)=-sin(lon)*sin(lat);
      tf.row(1).col(2)=cos(lon);
      tf.row(2).col(0)=cos(lon)*cos(lat);
      tf.row(2).col(1)=cos(lon)*sin(lat);
      tf.row(2).col(2)=sin(lon);
      *NF2ECEF=tf;
      return;
  }

  ///updates rotation matrix from ECEF frame to navigation frame
  void update_ECEF2NF(cv::Mat gps_position,cv::Mat* NF2ECEF,bool is_degrees){
      double lat,lon;
      /*if(is_degrees){
          lat=ecef_position.at<double>(0,0)*M_PI/180;
          lon=ecef_position.at<double>(0,0)*M_PI/180;
      }
      else{
          lat=ecef_position.at<double>(0,0);
          lon=ecef_position.at<double>(0,0);
      }*/
      if(is_degrees){
          lat=gps_position.at<double>(0,0)*M_PI/180;
          lon=gps_position.at<double>(1,0)*M_PI/180;
      }
      else{
          lat=gps_position.at<double>(0,0);
          lon=gps_position.at<double>(1,0);
      }
      cv::Mat tf=cv::Mat::zeros(3,3,CV_64F);

      tf.row(0).col(0)=-sin(lat)*cos(lon);
      tf.row(0).col(1)=-sin(lat)*sin(lon);
      tf.row(0).col(2)=cos(lat);
      tf.row(1).col(0)=-sin(lon);
      tf.row(1).col(1)=cos(lon);
      tf.row(2).col(0)=-cos(lat)*cos(lon);
      tf.row(2).col(1)=-cos(lat)*sin(lon);
      tf.row(2).col(2)=sin(lat);
      *NF2ECEF=tf;
      return;
  }
  ///updates rotation matrix from ECEF frame to navigaction frame
  void update_ECEF2NF_2(cv::Mat gps_position,cv::Mat* NF2ECEF,bool is_degrees){
      double lat,lon;
      /*if(is_degrees){
          lat=ecef_position.at<double>(0,0)*M_PI/180;
          lon=ecef_position.at<double>(0,0)*M_PI/180;
      }
      else{
          lat=ecef_position.at<double>(0,0);
          lon=ecef_position.at<double>(0,0);
      }*/
      if(is_degrees){
          lat=gps_position.at<double>(0,0)*M_PI/180;
          lon=gps_position.at<double>(1,0)*M_PI/180;
      }
      else{
          lat=gps_position.at<double>(0,0);
          lon=gps_position.at<double>(1,0);
      }
      cv::Mat tf=cv::Mat::zeros(3,3,CV_64F);

      tf.row(0).col(0)=-sin(lat)*cos(lon);
      tf.row(0).col(1)=-sin(lat)*sin(lon);
      tf.row(0).col(2)=cos(lat);
      tf.row(1).col(0)=-sin(lon);
      tf.row(1).col(1)=cos(lon);
      tf.row(2).col(0)=-cos(lat)*cos(lon);
      tf.row(2).col(1)=-cos(lat)*sin(lon);
      tf.row(2).col(2)=sin(lat);
      *NF2ECEF=tf;
      return;
  }

  ///converts geodetic coors(lat,lon,alt) to ecef coordinates
  void geod2ecef(cv::Mat gps_position,cv::Mat *uav_ecef_posi,bool is_degrees){
      double lat,lon,h=gps_position.at<double>(2,0);
      double x_e,y_e,z_e;
      if(is_degrees){
          lat=gps_position.at<double>(0,0)*M_PI/180;
          lon=gps_position.at<double>(1,0)*M_PI/180;
      }
      else{
          lat=gps_position.at<double>(0,0);
          lon=gps_position.at<double>(1,0);
      }
      double cslon=cos(lon);
      //std::cout<<"cslon: "<<cslon<<std::endl;
      double snlon=sin(lon);
      //std::cout<<"snlon: "<<snlon<<std::endl;
      double cslat=cos(lat);
      //std::cout<<":cslat "<<cslat<<std::endl;
      double snlat=sin(lat);
      //std::cout<<":snlat "<<snlat<<std::endl;
      double root_N_den=sqrt(1-e2*pow(snlat,2));
      //std::cout<<":root_N_den "<<root_N_den<<std::endl;
      //double N=a/(sqrt(1-e2*pow(sin(lon),2)));
      double N=a/(root_N_den);
      //std::cout<<"N: "<<N<<std::endl;
      x_e=(N+h)*cslat*cslon;
      //std::cout<<"x: "<<x_e<<std::endl;
      y_e=(N+h)*cslat*snlon;
      //std::cout<<"y: "<<y_e<<std::endl;
      z_e=(N*(1-e2)+h)*snlat;
      //std::cout<<"z: "<<z_e<<std::endl;
      uav_ecef_posi->row(0).col(0)=x_e;
      uav_ecef_posi->row(1).col(0)=y_e;
      uav_ecef_posi->row(2).col(0)=z_e;
      return;
  }

  ///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_4(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi){
      /*
       * lat: latitud geodetica
       * lon:longitud geodetica
       * lat_par: latitud parametrica
      */
      double lat=0, lon, h, h_old=0, lat_old,preci=1e-9;
      double x,y,z,N,cs,sn,r,p;
      x=uav_ecef_posi.at<double>(0,0);
      y=uav_ecef_posi.at<double>(1,0);
      z=uav_ecef_posi.at<double>(2,0);
      lon=atan2(y,x);
      r=sqrt(x*x+y*y+z*z);
      p=sqrt(x*x+y*y);
      lat_old=atan2(p,z);

      cs=cos(lat);
      sn=sin(lat);
      double root_N_den=sqrt(1-e2*pow(sn,2));
      N=a/(root_N_den);
      h=p/cs-N;
      while(fabs((lat-lat_old)*180/M_PI)>preci){
          h_old=h;
          cs=cos(lat);
          sn=sin(lat);
          root_N_den=sqrt(1-e2*pow(sn,2));
          N=a/(root_N_den);

          lat=atan2(z,p*(1-e2*N/(N+h)));
          lat_old=lat;
          h=p/cs-N;
      }
      h=p/cos(lat)-N;
      uav_gps_posi->row(0).col(0)=lat*180/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180/M_PI;
      uav_gps_posi->row(2).col(0)=h;
      return;
  }

  ///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_1(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi){
      /*
       * lat: latitud geodetica
       * lon:longitud geodetica
       * lat_par: latitud parametrica
       * r=d_xy
      */
      double lat, lon, h;
      double x,y,z, d_xy,N,cs,sn;
      double E_2,F,G,C,S,P,Q,r0,U,V,z0;
      x=uav_ecef_posi.at<double>(0,0);
      y=uav_ecef_posi.at<double>(1,0);
      z=uav_ecef_posi.at<double>(2,0);
      lon=atan2(y,x);
      d_xy=sqrt(x*x+y*y);
      //std::cout<<"d_xy: "<<d_xy<<std::endl;
      //std::cout<<"Z: "<<z<<std::endl;
      //std::cout<<"e2: "<<e2<<std::endl;
      //std::cout<<"E2: "<<E2<<std::endl;
      E_2=a*a-b*b;
      //std::cout<<"E_2: "<<E_2<<std::endl;
      F=54*b*b*z*z;
      //std::cout<<"F: "<<F<<std::endl;
      G=d_xy*d_xy+(1-e2)*z*z-e2*E_2;
      //std::cout<<"G: "<<G<<std::endl;
      C=e2*e2*F*d_xy*d_xy/pow(G,3);
      //std::cout<<"C: "<<C<<std::endl;
      S=pow(1+C+sqrt(C*C+2*C),1/3);
      //std::cout<<"S: "<<S<<std::endl;
      P=F/(3*pow((S+1/S+1),2)*G*G);
      //std::cout<<"P: "<<P<<std::endl;
      Q=sqrt(1+2*e2*e2*P);
      //std::cout<<"Q: "<<Q<<std::endl;
      //r0=-(P*e2*d_xy)/(1+Q)+sqrt(1/2*a*a*(1/Q+1)-P*(1-e2)*z*z/(Q*(1+Q))-1/2*P*d_xy*d_xy);
      r0=-(P*e2*d_xy)/(1+Q)+sqrt(0.5*a*a*(1/Q+1)-P*(1-e2)*z*z/(Q*(1+Q))-0.5*P*d_xy*d_xy);
      //std::cout<<"r0: "<<r0<<std::endl;
      U=sqrt(pow(d_xy-e2*r0,2)+z*z);
      //std::cout<<"U: "<<U<<std::endl;
      V=sqrt(pow(d_xy-e2*r0,2)+(1-e2)*z*z);
      //std::cout<<"V: "<<V<<std::endl;
      z0=b*b*z/(a*V);
      //std::cout<<"z0: "<<z0<<std::endl;
      h=U*(1-b*b)/(a*V);
      std::cout<<"h: "<<h<<std::endl;
      lat=atan((z+E2*z0)/d_xy);
      //std::cout<<"lat: "<<lat<<std::endl;
      sn=sin(lat);
      cs=cos(lat);
      N=a*a/sqrt(pow(a*cs,2)+pow(b*sn,2));
      h=d_xy/cos(lat)-N;


      uav_gps_posi->row(0).col(0)=lat*180/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180/M_PI;
      uav_gps_posi->row(2).col(0)=h;
      return;
  }

///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_2(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi){
      /*
       * lat: latitud geodetica
       * lon:longitud geodetica
       * lat_par: latitud parametrica
      */
      double lat, lon, h, h_old=0,lat_par,preci=1e-6;
      double x,y,z, d_xy,N,cs,sn;
      x=uav_ecef_posi.at<double>(0,0);
      y=uav_ecef_posi.at<double>(1,0);
      z=uav_ecef_posi.at<double>(2,0);
      lon=atan2(y,x);
      d_xy=sqrt(x*x+y*y);
      lat=atan2(z,d_xy*(1-e2));
      cs=cos(lat);
      sn=sin(lat);
      N=a*a/sqrt(pow(a*cs,2)+pow(b*sn,2));
      h=d_xy/cos(lat)-N;
      while(fabs(h-h_old)>preci){
          h_old=h;
          lat=atan2(z,d_xy*(1-e2*N/(N+h)));
          sn=sin(lat);
          cs=cos(lat);
          N=a*a/sqrt(pow(a*cs,2)+pow(b*sn,2));
          h=d_xy/cos(lat)-N;
      }
      uav_gps_posi->row(0).col(0)=lat*180/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180/M_PI;
      uav_gps_posi->row(2).col(0)=h;
      return;
  }

  ///converts eceef coordinates togeodetic coors(lat,lon,alt)
  void ecef2geod_3(cv::Mat *uav_gps_posi,cv::Mat uav_ecef_posi){
      /*
       * lat: latitud geodetica
       * lon:longitud geodetica
       * lat_par: latitud padd_executable(projections_2 projections_2.cpp )
  target_link_libraries( projections_2 ${OpenCV_LIBS} )
  target_link_libraries( projections_2 ${catkin_LIBRARIES} )arametrica
      */

      double lat, lon, h,lat_par;
      double x,y,z, d_xy,N;
      x=uav_ecef_posi.at<double>(0,0);
      y=uav_ecef_posi.at<double>(1,0);
      z=uav_ecef_posi.at<double>(2,0);
      d_xy=sqrt(x*x+y*y);
      lat_par=atan2(z*a,d_xy*b);
      lon=atan2(y,x);
      N=a/(sqrt(1-e2*pow(sin(lat),2)));
      lat=atan2(z+E2*b*pow(sin(lat_par),3),d_xy-e2*a*pow(cos(lat_par),3));
      h=d_xy/cos(lat)-N;
      uav_gps_posi->row(0).col(0)=lat*180/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180/M_PI;
      uav_gps_posi->row(2).col(0)=h;
      return;
  }

  ///converts geodetic coors(lat,lon,alt) to navigation frame
  void geod2nf(cv::Mat uav_gps_posi, cv::Mat ref_gps_posi,
                          cv::Mat* navf_point, bool is_degrees){
      //Transformation matrices between frames
      /****
      tf: transform
      BF: Body Frame
      CF: Camera Frame
      WF: World Frame
      NF: Navigation frame
      ECEF: Earth center earth fix frame
      ****/
      cv::Mat ECEF2NF_tf=cv::Mat::zeros(3,3,CV_64F);
      cv::Mat uav_ecef_posi=cv::Mat(3,1,CV_64F);
      cv::Mat uav_navf_posi=cv::Mat(3,1,CV_64F);
      cv::Mat ref_point_ecef=cv::Mat(3,1,CV_64F);
      cv::Mat ref_point_navf=cv::Mat(3,1,CV_64F);

      std::cout<<"gps_ref:\n"<<ref_gps_posi<<std::endl;
      //Pasar del sc geodetico (coor gps) al sc ECEF
      geod2ecef(uav_gps_posi,&uav_ecef_posi,is_degrees);
      std::cout<<"ecif posi:\n"<<uav_ecef_posi<<std::endl;
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);
      //std::cout<<"ecif ref:\n"<<ref_point_ecef<<std::endl;

      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      //update_ECEF2NF(uav_gps_posi,&ECEF2NF_tf,is_degrees);
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      //std::cout<<"Ref poitn navf: "<<ref_point_navf<<std::endl;
      uav_navf_posi=ECEF2NF_tf*(uav_ecef_posi-ref_point_ecef);
      std::cout<<"uav navf posi: "<<uav_navf_posi<<std::endl;
      uav_navf_posi.copyTo(*navf_point);

      return;
  }

  ///given the image coordinates, uav pose and camera info, returns the geodetic coordinates
  void georef_image_point(double cam_ori, cv::Mat imu_ori, cv::Mat uav_gps_posi,
                          cv::Mat k,cv::Mat image_coord, cv::Mat ref_gps_posi,
                          cv::Mat* georeferenced_point, bool is_degrees, double scale, double fl){
      //Transformation matrices between frames
      /****
      tf: transform
      BF: Body Frame
      CF: Camera Frame
      WF: World Frame
      NF: Navigation frame
      ECEF: Earth center earth fix frame
      ****/
      cv::Mat CF2BF_tf=cv::Mat::zeros(3,3,CV_64F);
      cv::Mat BF2NF_tf=cv::Mat::zeros(3,3,CV_64F);
      cv::Mat ECEF2NF_tf=cv::Mat::zeros(3,3,CV_64F);
      cv::Mat CF2NF_tf=cv::  Mat::zeros(3,3,CV_64F);
      cv::Mat NF2CF_tf=cv::Mat::zeros(3,3,CV_64F);
      cv::Mat uav_ecef_posi=cv::Mat(3,1,CV_64F);
      cv::Mat uav_navf_posi=cv::Mat(3,1,CV_64F);
      cv::Mat ref_point_ecef=cv::Mat(3,1,CV_64F);
      cv::Mat ref_point_navf=cv::Mat(3,1,CV_64F);
      cv::Mat cam_coord=cv::Mat(3,1,CV_64F);

      std::cout<<"k\n"<<k<<std::endl;
      std::cout<<"gps_ref:\n"<<ref_gps_posi<<std::endl;

      //Trasladar coordenadas de imagen a camara
      /*cam_coord.row(0).row(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).row(0)=image_coord.at<double>(1,0)-k.at<double>(1,2);
      cam_coord.row(2).row(0)=image_coord.at<double>(2,0);*/

      cam_coord.row(0).row(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).row(0)=-(image_coord.at<double>(1,0)-k.at<double>(1,2));
      cam_coord.row(2).row(0)=image_coord.at<double>(2,0);
      std::cout<<"cam coor: "<<cam_coord<<std::endl;

      //Pasar del frame de la camara al frame del body (UAV)
      update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);
      //std::cout<<"CF2BF:\n"<< CF2BF_tf<<std::endl;
      //Pasar del frame del body al frame de navegación
      update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
      //std::cout<<"BF2NF:\n"<< BF2NF_tf<<std::endl;
      //tranformación completa del frame de la camara al frame de navegación
      CF2NF_tf=BF2NF_tf*CF2BF_tf;
      //std::cout<<"cf2NF: \n"<<CF2NF_tf<<std::endl;
      cv::invert(CF2NF_tf,NF2CF_tf);
      //std::cout<<"NF2cf: \n"<<CF2NF_tf.inv()<<std::endl;

      //Pasar del sc geodetico (coor gps) al sc ECEF
      geod2ecef(uav_gps_posi,&uav_ecef_posi,is_degrees);
      std::cout<<"ecif posi:\n"<<uav_ecef_posi<<std::endl;
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);
      std::cout<<"ecif ref:\n"<<ref_point_ecef<<std::endl;

      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      //update_ECEF2NF(uav_gps_posi,&ECEF2NF_tf,is_degrees);
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      std::cout<<"Ref poitn navf: "<<ref_point_navf<<std::endl;
      uav_navf_posi=ECEF2NF_tf*(uav_ecef_posi-ref_point_ecef);
      std::cout<<"uav navf posi: "<<uav_navf_posi<<std::endl;
      cv::Mat geor_coor_navf=cv::Mat(3,1,CV_64F);
      cv::Mat geor_coor_ecef=cv::Mat(3,1,CV_64F);
      cv::Mat geor_coor_geod=cv::Mat(3,1,CV_64F);

      //uav_navf_posi.row(2).col(0)=0;
      geor_coor_navf=scale*CF2NF_tf*cam_coord+uav_navf_posi;
      std::cout<<"Georef coord navf:\n"<< geor_coor_navf<<std::endl;
      geor_coor_ecef=ref_point_ecef+ECEF2NF_tf.inv()*geor_coor_navf;
      std::cout<<"Georef coord ecef:\n"<< geor_coor_ecef<<std::endl;
      ecef2geod_2(&geor_coor_geod,geor_coor_ecef);
      std::cout<<"Georef coord geodf:\n"<< geor_coor_geod<<std::endl;
      std::cout<<std::endl<<std::endl;

      georeferenced_point->row(0).col(0)=geor_coor_geod.at<double>(0,0);
      georeferenced_point->row(1).col(0)=geor_coor_geod.at<double>(1,0);
      georeferenced_point->row(2).col(0)=geor_coor_geod.at<double>(2,0);
      return;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  FlightPlanner fp;
  ROS_INFO("ready to geref points or plan flight");

  ros::spin();

  return 0;
}

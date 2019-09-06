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
#include  "hector_uav_msgs/YawrateCommand.h"
#include  "hector_uav_msgs/ThrustCommand.h"
#include  "hector_uav_msgs/AttitudeCommand.h"
#include  "hector_uav_msgs/TakeoffAction.h"
#include  "hector_uav_msgs/EnableMotors.h"
#include  "hector_uav_msgs/LandingAction.h"
#include  "hector_uav_msgs/PoseAction.h"
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


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/multi_point.hpp>

#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
typedef bg::model::d2::point_xy<double> bgPoint;
typedef bg::model::polygon<bgPoint> bgPoly;
typedef bg::model::linestring<bgPoint> bgLine;
typedef bg::model::segment<bgPoint> bgSegm;


const double a=6378137;//Semimajor axis
const double f=1/298.257223563;
const double b=a*(1-f);//Semiminor axis
const double e2=(a*a-b*b)/(a*a);//first excentricity
const double E2=e2/(1-e2);//second excentricity

namespace bg=boost::geometry;

class line_class{

public:
  cv::Mat line_;
  std::vector<bgPoint>  interceptions;
  bgSegm  line_segment;
  bool  intercepts;
  line_class(){
    line_=cv::Mat(2,2,CV_64F);
  }
  line_class(double  x1,double y1,double x2,double y2){
    //ecuación parametrica recta columna 0 componente x; columna 1 componente y
    //fila 0 punto inicial  fila 1 vector director
    line_=cv::Mat(2,2,CV_64F);
    line_.row(0).col(0)=x1;
    line_.row(0).col(1)=y1;
    line_.row(1).col(0)=x2-x1;
    line_.row(1).col(1)=y2-y1;
    intercepts=false;
  }

  line_class(double  x1,double y1,cv::Mat* vector){
    //ecuación parametrica recta columna 0 componente x; columna 1 componente y
    line_=cv::Mat(2,2,CV_64F);
    line_.row(0).col(0)=x1;
    line_.row(0).col(1)=y1;
    line_.row(1).col(0)=vector->at<double>(0,0);
    line_.row(1).col(1)=vector->at<double>(1,0);
  }

  line_class(double  x1,double y1,bgPoint){
    //ecuación parametrica recta columna 0 componente x; columna 1 componente y
    line_=cv::Mat(2,2,CV_64F);
    line_.row(0).col(0)=x1;
    line_.row(0).col(1)=y1;
    line_.row(1).col(0)=vector->at<double>(0,0);
    line_.row(1).col(1)=vector->at<double>(1,0);
  }

  /// finds the interception between the line to intercept and the line
  bool find_interception(cv::Mat* line_to_int){
    bool  success=false;
    cv::Mat A=cv::Mat(2,2,CV_64F);
    cv::Mat vec=cv::Mat(2,1,CV_64F);
    cv::Mat t=cv::Mat(2,1,CV_64F);
    intercpt_found=false;
    A.row(0).col(0)=line_.at<double>(1,0);
    A.row(0).col(1)=-line_to_int->at<double>(1,0);
    A.row(1).col(0)=line_.at<double>(1,1);
    A.row(1).col(1)=-line_to_int->at<double>(1,1);

    vec.row(0).col(0)=line_to_int->at<double>(0,0)-line_.at<double>(0,0);
    vec.row(1).col(0)=line_to_int->at<double>(0,1)-line_.at<double>(0,1);

    try{
      t=A.inv()*vec;
    }
    //To catch general exceptions
    catch(...){
      std::cout<<"exception found"<<std::endl;
      intercpt_found=false;
      return success;
    }
    intercept_point.x=line_.at<double>(0,0)+line_.at<double>(1,0)*t.at<double>(0,0);
    intercept_point.y=line_.at<double>(0,1)+line_.at<double>(1,1)*t.at<double>(0,0);
    success=true;
    intercpt_found=success;
    return  success;
  }

  double  calculate_y(double x){

  }

  void  line_move(double x,double y){
    line_.row(0).col(0)+=x;
    line_.row(0).col(1)+=y;
    return;
  }
};

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
  bool  debug_;
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
    debug_=true;

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
    cv::Mat georef_point=cv::Mat(3,1,CV_64F);
    cv::Mat crop_row_vector_navf=cv::Mat(3,1,CV_64F);
    cv::Mat crop_row_vector_image=cv::Mat(3,1,CV_64F);
    cv::Mat side_vector=cv::Mat(3,1,CV_64F);

    std::vector<std::vector<cv::Mat> > navf_plan_waypoints;

    line_class  row_line;
    std::vector<line_class> contour_lines;

    uint  lines_number;
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

    double  gr_height_cover;//ground coverage according to desired gsd
    double  gr_width_cover;

    uint nfl;//Number of flight lines
    uint ipfl;//number images per flight line

    endlap=req.endlap;
    sidelap=req.sidelap;
    /***Obtencion coordenadas de los limites del cultivo***/
    for(int i=0;i<numCropPoints;i++){
      crop_pixel_coord.at(i)=cv::Mat(3,1,CV_64F);
      crop_pixel_coord.at(i).row(0).col(0)=req.crop.contour.at(i).x;
      crop_pixel_coord.at(i).row(1).col(0)=req.crop.contour.at(i).y;
      crop_pixel_coord.at(i).row(2).col(0)=fl_pixel;
    }
    std::vector<cv::Mat>  crop_gps_coors;
    crop_gps_coors.resize(numCropPoints);
    res.crop_gps_coors.resize(numCropPoints);
    std::vector<cv::Mat>  crop_navf_coors;
    crop_navf_coors.resize(numCropPoints);
    if (debug_){std::cout<<"\n++++GEOREFERENCIACIÓN VERTICES+++"<<std::endl;}
    for(int i=0;i<numCropPoints;i++){
      georef_image_point(cam_ori,imu_ori,uav_gps,k,crop_pixel_coord.at(i),ref_gps,
                         &georef_point,are_degrees,scale_number,fl,false);
      georef_point.copyTo(crop_gps_coors.at(i));

      if2navf(cam_ori,imu_ori,uav_gps,k,crop_pixel_coord.at(i),ref_gps,
              &georef_point,are_degrees,scale_number,fl,debug_);
      //std::cout<<"llego"<<std::endl;
      georef_point.copyTo(crop_navf_coors.at(i));
      cvMat2gps(&res.crop_gps_coors[i],&georef_point);
    }

    /*********************/
    if (debug_){std::cout<<"\n+++Calculo varibles fotogrometricas+++"<<std::endl;}
    gr_height_cover=i_height*gsd;
    if (debug_){std::cout<<"ground height coverrage\n"<<gr_height_cover<<std::endl;}
    gr_width_cover=i_width*gsd;
    if (debug_){std::cout<<"ground width coverrage\n"<<gr_width_cover<<std::endl;}
    base=gr_height_cover*(1-endlap/100);
    if (debug_){std::cout<<"base\n"<<base<<std::endl;}
    line_space=gr_width_cover*(1-sidelap/100);
    if (debug_){std::cout<<"line space\n"<<line_space<<std::endl;}

    if (debug_){std::cout<<"\n+++TRANSFORMACIÖN VECTOR ORIENTACIÓN DEL IF AL NAVF+++"<<std::endl;}
    /***transformar el vector de orientación del cultivo***/
    polar2cart(&crop_row_vector_image,1,-req.crop.row_orientation,true);
    crop_row_vector_image.row(0).col(0)+=i_width/2;
    crop_row_vector_image.row(1).col(0)+=i_height/2;
    crop_row_vector_image.row(2).col(0)=fl_pixel;
    if (debug_){std::cout<<"row vector image: "<<crop_row_vector_image<<std::endl;}
    if2navf(cam_ori,imu_ori,uav_gps,k,crop_row_vector_image,ref_gps,
            &crop_row_vector_navf,are_degrees,scale_number,fl,true);
    crop_row_vector_navf.row(2).col(0)=0;
    normalize_2d(crop_row_vector_navf);
    if (debug_){std::cout<<"row vector navf: "<<crop_row_vector_navf<<std::endl;}
    crop_row_vector_navf*=base;
    if (debug_){std::cout<<"row vector navf: "<<crop_row_vector_navf<<std::endl;}


    /***Determinación del punto inicial del cultivo***/

    if (debug_){std::cout<<"\n+++DETERMINACIÓN max's min's x,y contorno+++"<<std::endl;}
    cv::Mat min_vector=cv::Mat(2,1,CV_64F);
    cv::Mat max_vector=cv::Mat(2,1,CV_64F);
    find_max_min_x_y(crop_navf_coors,&max_vector,&min_vector);
    if (debug_){std::cout<<"min x: "<<min_vector.at<double>(0,0)<<"\tmin y"<<min_vector.at<double>(1,0)<<std::endl;
    std::cout<<"max x: "<<max_vector.at<double>(0,0)<<"\tmax y"<<max_vector.at<double>(1,0)<<std::endl;}
    if (debug_){std::cout<<"\n+++DETERMINACIÓN ECUACIONES RECTAS CONTORNO+++"<<std::endl;}
    contour_lines.resize(numCropPoints);
    for(int i=0;i<numCropPoints;i++){
      if(i==0){
        contour_lines.at(i)=line_class(crop_navf_coors.at(i).at<double>(0,0),crop_navf_coors.at(i).at<double>(1,0),
                                       crop_navf_coors.at(numCropPoints-1).at<double>(0,0),crop_navf_coors.at(numCropPoints-1).at<double>(1,0));
      }
      else{
        contour_lines.at(i)=line_class(crop_navf_coors.at(i).at<double>(0,0),crop_navf_coors.at(i).at<double>(1,0),
                                       crop_navf_coors.at(i-1).at<double>(0,0),crop_navf_coors.at(i-1).at<double>(1,0));
      }
      std::cout<<"line "<<i<<": "<<contour_lines.at(i).line_<<std::endl;
    }
    std::cout<<"\n+++DETERMINACIÓN PUNTO INICIAL+++"<<std::endl;

    uint  initial_point_index;
    //initial_point_index= find_initial_point(&crop_gps_coors,ref_gps,are_degrees);
    /*initial_point_index= find_initial_point(&crop_pixel_coord,cam_ori,imu_ori,uav_gps,
                                            k,ref_gps,are_degrees,scale_number,fl);*/
    initial_point_index=find_initial_point_and_side_vector(&crop_navf_coors,&contour_lines,&crop_row_vector_navf,max_vector.at<double>(0,0),
                                                           max_vector.at<double>(1,0),min_vector.at<double>(0,0),min_vector.at<double>(1,0),
                                                           &side_vector,line_space,&lines_number,true);

    if (debug_){std::cout<<"inicial point"<<crop_navf_coors.at(initial_point_index)<<"\nlines number: "<<lines_number<<std::endl;}
    std::cout<<"\n+++GENERACIÓN PUNTOS+++"<<std::endl;
    navf_plan_waypoints;
    generate_points(&crop_navf_coors,&contour_lines,&crop_row_vector_navf,max_vector.at<double>(0,0),
                    max_vector.at<double>(1,0),min_vector.at<double>(0,0),min_vector.at<double>(1,0),
                    &side_vector,base,line_space,lines_number,initial_point_index, true,&navf_plan_waypoints);
    if(true){
      std::cout<<"x:\ty:"<<std::endl;
      for(int i=0;i<navf_plan_waypoints.size();i++){
        for(int j=0;j<navf_plan_waypoints.at(i).size();j++){
          std::cout<<navf_plan_waypoints.at(i).at(j).at<double>(0,0)<<"\t"<<navf_plan_waypoints.at(i).at(j).at<double>(1,0)<<std::endl;
        }
      }
    }

    std::cout<<"\n+++FIN+++"<<std::endl;

    return  true;
  }

  void  generate_points(std::vector<cv::Mat>* navf_points,std::vector<line_class>* lines,
                        cv::Mat* row_vector,double x_max,double y_max,double x_min,double y_min,
                        cv::Mat* side_vector,double base, double line_space,double lines_number,
                        uint first_point_index, bool debug,std::vector<std::vector<cv::Mat> >* waypoints){
    uint  num_points=navf_points->size();//numero de vertices identificados en el contorno
    std::vector<std::vector<cv::Mat> >  flight_lines;//conjunto de lineas con los puntos del vuelo
    std::vector<uint>  interc_index;//vector que contendra los indices de las rectas del controno cuyas intercepciones con la recta del surco esten en la región limitada
    line_class  row_line;//recta cuyo vector director es la dirección del surco
    uint num_interc;//numero de intersecciones validas
    double  dist;//distancia entre dos puntos p1,p2
    double  noph;//number of photografies
    uint  addittional_wp=2;//Numero de waypoints adicionales
    cv::Mat advance_vector=cv::Mat(3,1,CV_64F);//Vector de avance sibre las lineas
    row_vector->copyTo(advance_vector);
    uint  line_begin_index;//Encontrar un mejor nombre
    uint  line_end_index;
    //inicialización recta
    row_line=line_class(navf_points->at(first_point_index).at<double>(0,0),navf_points->at(first_point_index).at<double>(1,0),row_vector );

    flight_lines.resize(lines_number);
    //
    interc_index.resize(navf_points->size());


    return;
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
      georef_image_point(cam_ori,imu_ori,uav_gps,k,pixel_coord.at(i),ref_gps,
                         &georef_point,are_degrees,scale_number,fl,true);
      cvMat2gps(&res.gps_coors[i],&georef_point);
    }
    return  true;
  }

  void find_max_min_x_y(std::vector<cv::Mat>  points,cv::Mat* max_vector,
                        cv::Mat* min_vector){
    double  max_x,min_x,max_y,min_y;
    uint  num_points=points.size();
    min_x=points.at(0).at<double>(0,0);
    max_x=points.at(0).at<double>(0,0);
    min_y=points.at(0).at<double>(1,0);
    max_y=points.at(0).at<double>(1,0);
    for(int i=1;i<num_points;i++){
      if(points.at(i).at<double>(0,0)<min_x){
        min_x=points.at(i).at<double>(0,0);
      }
      if(points.at(i).at<double>(0,0)>max_x){
        max_x=points.at(i).at<double>(0,0);
      }
      if(points.at(i).at<double>(1,0)<min_y){
        min_y=points.at(i).at<double>(1,0);
      }
      if(points.at(i).at<double>(1,0)>max_y){
        max_y=points.at(i).at<double>(1,0);
      }
    }
    min_vector->row(0).col(0)=min_x;
    min_vector->row(1).col(0)=min_y;
    max_vector->row(0).col(0)=max_x;
    max_vector->row(1).col(0)=max_y;

    return;
  }

  uint  find_initial_point_and_side_vector(std::vector<cv::Mat>* navf_points,std::vector<line_class>* lines,
                                           cv::Mat* row_vector,double x_max,double y_max,double x_min,double y_min,
                                           cv::Mat* side_vector, double line_space,uint* line_number, bool debug){

  }
  void  rotate_vetor(cv::Mat* orig ,cv::Mat* rotated,double magnitude, double  rotation,bool are_degrees){
    double theta;
    if(are_degrees){
      rotation*=CV_PI/180.0;
    }
    theta=atan2(orig->at<double>(1,0),orig->at<double>(0,0));
    if(theta<0){
      theta+=2*CV_PI;
    }
    theta+=rotation;
    polar2cart(rotated,magnitude,theta,false);
    return;
  }

  void  polar2cart(double *x,double* y,double r,double theta,bool is_deg){
    if(is_deg){
      theta*=CV_PI/180;
    }
    *x=r*cos(theta);
    *y=r*sin(theta);
    return;
  }
  void  polar2cart(cv::Mat *vector,double r,double theta,bool is_deg){
    if(is_deg){
      theta*=CV_PI/180;
    }
    //*vector->at<double>(0,0)=r*cos(theta);
    vector->row(0).col(0)=r*cos(theta);//x=r*cos(theta)
    vector->row(1).col(0)=r*sin(theta);//y=r*sin(theta)
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
      cv::Mat Rz=cv::Mat::zeros(3,3,CV_64F);
      Rz.row(2).col(2)=1;
      Rz.row(0).col(0)=cos(cam_ori);
      Rz.row(0).col(1)=-sin(cam_ori);
      Rz.row(1).col(0)=sin(cam_ori);
      Rz.row(1).col(1)=cos(cam_ori);
      *CF2BF=Rz;
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
      double snlon=sin(lon);
      double cslat=cos(lat);
      double snlat=sin(lat);
      double root_N_den=sqrt(1-e2*pow(snlat,2));
      double N=a/(root_N_den);
      x_e=(N+h)*cslat*cslon;
      y_e=(N+h)*cslat*snlon;
      z_e=(N*(1-e2)+h)*snlat;
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
      E_2=a*a-b*b;
      F=54*b*b*z*z;
      G=d_xy*d_xy+(1-e2)*z*z-e2*E_2;
      C=e2*e2*F*d_xy*d_xy/pow(G,3);
      S=pow(1+C+sqrt(C*C+2*C),1/3);
      P=F/(3*pow((S+1/S+1),2)*G*G);
      Q=sqrt(1+2*e2*e2*P);
      r0=-(P*e2*d_xy)/(1+Q)+sqrt(0.5*a*a*(1/Q+1)-P*(1-e2)*z*z/(Q*(1+Q))-0.5*P*d_xy*d_xy);
      U=sqrt(pow(d_xy-e2*r0,2)+z*z);
      V=sqrt(pow(d_xy-e2*r0,2)+(1-e2)*z*z);
      z0=b*b*z/(a*V);
      h=U*(1-b*b)/(a*V);
      lat=atan((z+E2*z0)/d_xy);
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

  void  normalize_2d(cv::Mat vector){
    double  mag=sqrt(pow(vector.at<double>(0,0),2)+pow(vector.at<double>(1,0),2));
    vector.row(0).col(0)/=mag;
    vector.row(1).col(0)/=mag;
    return;
  }
  void  normalize_3d(cv::Mat vector){
    double  mag=sqrt(pow(vector.at<double>(0,0),2)+pow(vector.at<double>(1,0),2)+pow(vector.at<double>(2,0),2));
    vector.row(0).col(0)/=mag;
    vector.row(1).col(0)/=mag;
    vector.row(2).col(0)/=mag;
    return;
  }


  ///converts geodetic coors(lat,lon,alt) to navigation frame
  void geod2nf(cv::Mat uav_gps_posi, cv::Mat ref_gps_posi,
                          cv::Mat* navf_point, bool is_degrees,bool debug){
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

      //Pasar del sc geodetico (coor gps) al sc ECEF
      geod2ecef(uav_gps_posi,&uav_ecef_posi,is_degrees);
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);

      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      uav_navf_posi=ECEF2NF_tf*(uav_ecef_posi-ref_point_ecef);
      if (debug){std::cout<<"uav navf posi: "<<uav_navf_posi<<std::endl;}
      uav_navf_posi.copyTo(*navf_point);

      return;
  }

  void if2navf(double cam_ori, cv::Mat imu_ori, cv::Mat uav_gps_posi,
                          cv::Mat k,cv::Mat image_coord, cv::Mat ref_gps_posi,
                          cv::Mat* navf_point, bool is_degrees, double scale, double fl,bool debug){
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

      //Trasladar coordenadas de imagen a camara
      cam_coord.row(0).col(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).col(0)=-(image_coord.at<double>(1,0)-k.at<double>(1,2));
      cam_coord.row(2).col(0)=image_coord.at<double>(2,0);
      if (debug){std::cout<<"cam coor: "<<cam_coord<<std::endl;}

      //Pasar del frame de la camara al frame del body (UAV)
      update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);
      //Pasar del frame del body al frame de navegación
      update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
      //tranformación completa del frame de la camara al frame de navegación
      CF2NF_tf=BF2NF_tf*CF2BF_tf;
      cv::invert(CF2NF_tf,NF2CF_tf);
      //Pasar del sc geodetico (coor gps) al sc ECEF
      geod2ecef(uav_gps_posi,&uav_ecef_posi,is_degrees);
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);
      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      uav_navf_posi=ECEF2NF_tf*(uav_ecef_posi-ref_point_ecef);
      cv::Mat geor_coor_navf=cv::Mat(3,1,CV_64F);
      geor_coor_navf=scale*CF2NF_tf*cam_coord+uav_navf_posi;

      if (debug){std::cout<<"Georef coord navf:\n"<< geor_coor_navf<<std::endl;}
      navf_point->row(0).col(0)=geor_coor_navf.at<double>(0,0);
      navf_point->row(1).col(0)=geor_coor_navf.at<double>(1,0);
      navf_point->row(2).col(0)=geor_coor_navf.at<double>(2,0);
      return;
  }

  ///given the image coordinates, uav pose and camera info, returns the geodetic coordinates
  void georef_image_point(double cam_ori, cv::Mat imu_ori, cv::Mat uav_gps_posi,
                          cv::Mat k,cv::Mat image_coord, cv::Mat ref_gps_posi,
                          cv::Mat* georeferenced_point, bool is_degrees, double scale, double fl, bool debug){
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

      //Trasladar coordenadas de imagen a camara
      /*cam_coord.row(0).row(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).row(0)=image_coord.at<double>(1,0)-k.at<double>(1,2);
      cam_coord.row(2).row(0)=image_coord.at<double>(2,0);*/
      cam_coord.row(0).col(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).col(0)=-(image_coord.at<double>(1,0)-k.at<double>(1,2));
      cam_coord.row(2).col(0)=image_coord.at<double>(2,0);

      //Pasar del frame de la camara al frame del body (UAV)
      update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);
      //Pasar del frame del body al frame de navegación
      update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
      //tranformación completa del frame de la camara al frame de navegación
      CF2NF_tf=BF2NF_tf*CF2BF_tf;
      cv::invert(CF2NF_tf,NF2CF_tf);

      //Pasar del sc geodetico (coor gps) al sc ECEF
      geod2ecef(uav_gps_posi,&uav_ecef_posi,is_degrees);
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);

      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      uav_navf_posi=ECEF2NF_tf*(uav_ecef_posi-ref_point_ecef);
      cv::Mat geor_coor_navf=cv::Mat(3,1,CV_64F);
      cv::Mat geor_coor_ecef=cv::Mat(3,1,CV_64F);
      cv::Mat geor_coor_geod=cv::Mat(3,1,CV_64F);


      geor_coor_navf=scale*CF2NF_tf*cam_coord+uav_navf_posi;
      geor_coor_ecef=ref_point_ecef+ECEF2NF_tf.inv()*geor_coor_navf;
      ecef2geod_2(&geor_coor_geod,geor_coor_ecef);
      if(debug){
        std::cout<<"Georef coord geodf:\n"<< geor_coor_geod<<std::endl;
      }
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
  std::setprecision(6);
  FlightPlanner fp;
  ROS_INFO("ready to geref points or plan flight");

  ros::spin();

  return 0;
}

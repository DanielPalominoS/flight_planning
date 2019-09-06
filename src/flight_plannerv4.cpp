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

class line_class{

public:
  cv::Mat line_;
  cv::Point2f intercept_point;
  bool  intercpt_found;
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
    intercept_point.x=0;
    intercept_point.y=0;
    intercpt_found=false;
  }

  line_class(double  x1,double y1,cv::Mat* vector){
    //ecuación parametrica recta columna 0 componente x; columna 1 componente y
    line_=cv::Mat(2,2,CV_64F);
    line_.row(0).col(0)=x1;
    line_.row(0).col(1)=y1;
    line_.row(1).col(0)=vector->at<double>(0,0);
    line_.row(1).col(1)=vector->at<double>(1,0);
    intercept_point.x=0;
    intercept_point.y=0;
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
    s_height=req.s_heigth;
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

    std::cout<<req<<std::endl;

    /*******************************************************************************/
    if (debug_){std::cout<<"\n+++Calculo varibles fotogrometricas+++"<<std::endl;}
    gr_height_cover=i_height*gsd;
    if (debug_){std::cout<<"ground height coverrage\n"<<gr_height_cover<<std::endl;}
    gr_width_cover=i_width*gsd;
    if (debug_){std::cout<<"ground width coverrage\n"<<gr_width_cover<<std::endl;}
    base=gr_height_cover*(1-endlap/100);
    if (debug_){std::cout<<"base\n"<<base<<std::endl;}
    line_space=gr_width_cover*(1-sidelap/100);
    if (debug_){std::cout<<"line space\n"<<line_space<<std::endl;}
    if(req.frame_rate>0){
      if(base/req.frame_rate>req.max_speed){
        res.uav_speed=base*req.frame_rate;
      }
      else{
        res.uav_speed=req.max_speed;
      }
    }
    else{
      res.uav_speed=1;//Velocidad por defecto
    }
    res.flight_height=calculate_height(fl,gsd,s_width,i_width);
    flight_height=res.flight_height;
    res.trig_interval=base/res.uav_speed;

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
      georef_point.at<double>(2,0)=ref_gps_pose.z+flight_height;
      georef_point.copyTo(crop_gps_coors.at(i));
      cvMat2gps(&res.crop_gps_coors[i],&georef_point);
      if2navf(cam_ori,imu_ori,uav_gps,k,crop_pixel_coord.at(i),ref_gps,
              &georef_point,are_degrees,scale_number,fl,false);
      //std::cout<<"llego"<<std::endl;
      georef_point.copyTo(crop_navf_coors.at(i));
      /*geometry_msgs::Vector3  temp;
      georef_nav_point(crop_navf_coors.at(i),ref_gps,
                         &georef_point,are_degrees,false);*/

    }

    if (debug_){std::cout<<"\n+++TRANSFORMACIÓN VECTOR ORIENTACIÓN DEL IF AL NAVF+++"<<std::endl;}
    /***transformar el vector de orientación del cultivo***/
    std::cout<<req.crop.row_orientation<<std::endl;
    polar2cart(&crop_row_vector_image,1.0,req.crop.row_orientation+90,are_degrees);
    crop_row_vector_image.row(0).col(0)+=k.at<double>(0,2);//se le agrega esta cantidad para que el vector quede en el origen de la camara
    crop_row_vector_image.row(1).col(0)+=k.at<double>(1,2);
    crop_row_vector_image.row(2).col(0)=fl_pixel;
    cv::Mat temp;
    ref_gps.copyTo (temp);
    temp.row(2).col(0)=uav_gps.at<double>(2.0);
    if (debug_){std::cout<<"row vector image: \n"<<crop_row_vector_image<<std::endl;}

    if2navf(cam_ori,imu_ori,temp,k,crop_row_vector_image,ref_gps,
            &crop_row_vector_navf,are_degrees,scale_number,fl,true);
    crop_row_vector_navf.row(2).col(0)=0;
    normalize_2d(crop_row_vector_navf);
    if (true){std::cout<<"row vector navf unit: \n"<<crop_row_vector_navf<<std::endl;}
    crop_row_vector_navf*=base;
    if (true){std::cout<<"row vector navf: \n"<<crop_row_vector_navf<<std::endl;}
    temp.release ();

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
      //std::cout<<"line "<<i<<": "<<contour_lines.at(i).line_<<std::endl;
    }
    //std::cout<<"\n+++DETERMINACIÓN PUNTO INICIAL+++"<<std::endl;

    uint  initial_point_index;
    //initial_point_index= find_initial_point(&crop_gps_coors,ref_gps,are_degrees);
    /*initial_point_index= find_initial_point(&crop_pixel_coord,cam_ori,imu_ori,uav_gps,
                                            k,ref_gps,are_degrees,scale_number,fl);*/
    initial_point_index=find_initial_point_and_side_vector(&crop_navf_coors,&contour_lines,&crop_row_vector_navf,max_vector.at<double>(0,0),
                                                           max_vector.at<double>(1,0),min_vector.at<double>(0,0),min_vector.at<double>(1,0),
                                                           &side_vector,line_space,&lines_number,true);

    //if (debug_){std::cout<<"inicial point"<<crop_navf_coors.at(initial_point_index)<<"\nlines number: "<<lines_number<<std::endl;}
    //std::cout<<"\n+++GENERACIÓN PUNTOS+++"<<std::endl;
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
    /****Georeferenciación puntos generados*********/
    int counter=0;
    for(int i=0 ;i<navf_plan_waypoints.size();i++){
      for(int j=0;j<navf_plan_waypoints.at(i).size();j++){
        geometry_msgs::Vector3  temp;
        georef_nav_point(navf_plan_waypoints.at(i).at(j),ref_gps,
                           &georef_point,are_degrees,false);
        cvMat2gps(&temp,&georef_point);
        temp.z=ref_gps.at<double>(2,0)+flight_height;
        res.plan_gps_coors.push_back(temp);
        counter++;
      }
    }

    std::cout<<"\n+++FIN+++"<<std::endl;
    //std::cout<<res<<std::endl;
    res.success=true;
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

    /******Inicializacion primera linea******/
    dist=-1;
    double  dist_temp;
    //Se recorren todos los vertices del contorno del cultivo
    for(int i=0;i<num_points;i++){
      //Se valida si la intercepción entre la recta del surco y la recta del contorno existe
      if(lines->at(i).find_interception(&row_line.line_)){
        //Si la intercepción existe, se valida que este dentro de los limites definidos por los maximos y los minimos
        if(lines->at(i).intercept_point.x<=x_max+2&&lines->at(i).intercept_point.x>=x_min-2&&
           lines->at(i).intercept_point.y<=y_max+2&&lines->at(i).intercept_point.y>=y_min-2){
          num_interc++;
          //Se calcula la distancia del vertice (navf_pint) a la intercepción encontrada
          dist_temp=calculate_distance(lines->at(i).intercept_point.x,lines->at(i).intercept_point.y,
                                       navf_points->at(first_point_index).at<double>(0,0),navf_points->at(first_point_index).at<double>(1,0));
          if(dist<0){
            dist=dist_temp;
            line_begin_index=i;
          }
          //se guarda el indice de la linea cuya intercepción es mas distante al vertice (punto inicial)
          if(dist_temp>dist){
            dist=dist_temp;
            line_begin_index=i;
          }
        }
      }
      //std::cout<<"num int: "<<num_interc<<std::endl;
    }
    //NOTA:considerar poner un umbral
    //Si la distancia es mayor a la longitud de la base se define el vector de avance con base
    //en el vertice del punto inicial y la intercepción seleccionada como la mas lejana. En caso contrario se usa el vector del surco definido como argumento d ela función

    std::cout<<"intercepción final\nx: "<<lines->at(line_begin_index).intercept_point.x<<"\ty: "<<lines->at(line_begin_index).intercept_point.y<<std::endl;
    std::cout<<"dist: "<<dist<<std::endl;
    if(dist>base){
      //Calcular numero de fotografías
      //Nota: ceil aproxima el siguiente entero hacia arriba
      noph=(uint)(ceil(dist/base)+1);//se calcula el numero de fotografias para la linea inicial. Se agrega una linea adicional como recomendación para garantizar el cubirmiento del terreno
      advance_vector.row(0).col(0)=lines->at(line_begin_index).intercept_point.x-navf_points->at(first_point_index).at<double>(0,0);
      advance_vector.row(1).col(0)=lines->at(line_begin_index).intercept_point.y-navf_points->at(first_point_index).at<double>(1,0);
      advance_vector.row(2).col(0)=0;
    }
    else{
      row_vector->copyTo(advance_vector);
      noph=2;
    }
    //Se normaliza el vector de avance y se multiplica por la longitud de la base
    normalize_2d(advance_vector);
    advance_vector*=base;
    std::cout<<"\nAdvance vector\nx: "<<advance_vector.at<double>(0,0)<<"\ty: "<<advance_vector.at<double>(1,0)<<std::endl;
    std::cout<<"side vector\nx: "<<side_vector->at<double>(0,0)<<"\ty: "<<side_vector->at<double>(1,0)<<std::endl<<std::endl;
    //Se define la cantidad de puntos en la linea con el numero de fotografias necesarias, mas una cantidad adicional
    //flight_lines.at(0).resize(noph+addittional_wp,cv::Mat(3,1,CV_64F));//se recomienda colocar waypoints adicionales para que el uav se alinie adecuadamente cuando cambia de linea
    flight_lines.at(0).resize(noph+addittional_wp);//se recomienda colocar waypoints adicionales para que el uav se alinie adecuadamente cuando cambia de linea
    // A partir del vertice se retrocede la distancia de la mitad de los puntos adicionales, en la dirección opuesta al vector de avance, para luego generar los demas
    //puntos llendo en la dirección de avance


    double  x_temp,y_temp;

    flight_lines[0][0]=cv::Mat(3,1,CV_64F);
    flight_lines[0][0].row(0).col(0)=navf_points->at(first_point_index).at<double>(0,0)-addittional_wp/2*advance_vector.at<double>(0,0);
    flight_lines[0][0].row(1).col(0)=navf_points->at(first_point_index).at<double>(1,0)-addittional_wp/2*advance_vector.at<double>(1,0);
    flight_lines[0][0].row(2).col(0)=flight_height;
    flight_lines[0][0].row(2).col(0)=0;
    /*x_temp= navf_points->at(first_point_index).at<double>(0,0)-addittional_wp/2*advance_vector.at<double>(0,0);
    y_temp=navf_points->at(first_point_index).at<double>(1,0)-addittional_wp/2*advance_vector.at<double>(1,0);*/

    /*std::cout<<"x: \t\ty: "<<std::endl;
    std::cout<<flight_lines.at(0).at(0).at<double>(0,0)<<"\t"<<flight_lines.at(0).at(0).at<double>(1,0)<<std::endl;*/
    for(int i=1;i<flight_lines.at(0).size();i++){
      /*flight_lines.at(0).at(i).row(0).col(0)=flight_lines.at(0).at(i-1).at<double>(0,0)+advance_vector.at<double>(0,0);
      flight_lines.at(0).at(i).row(1).col(0)=flight_lines.at(0).at(i-1).at<double>(1,0)+advance_vector.at<double>(1,0);*/
      flight_lines[0][i]=cv::Mat(3,1,CV_64F);
      flight_lines[0][i].row(0).col(0)=flight_lines[0][i-1].at<double>(0,0)+advance_vector.at<double>(0,0);
      flight_lines[0][i].row(1).col(0)=flight_lines[0][i-1].at<double>(1,0)+advance_vector.at<double>(1,0);
      flight_lines[0][i].row(2).col(0)=flight_height;
      flight_lines[0][i].row(2).col(0)=0;
      //std::cout<<flight_lines[0][i].at<double>(0,0)<<"\t"<<flight_lines[0][i].at<double>(1,0)<<std::endl;
    }

    /*std::cout<<std::endl<<std::endl;
    for(int i=0;i<flight_lines.at(0).size();i++){
      std::cout<<flight_lines[0][i].at<double>(0,0)<<"\t"<<flight_lines[0][i].at<double>(1,0)<<std::endl;
    }
    std::cout<<std::endl<<std::endl;*/
    //Desplazar la linea para hallar las nuevas intercepciones
    row_line.line_move(side_vector->at<double>(0,0),side_vector->at<double>(1,0));
    /**********************************************/
    /******Generación puntos lineas restantes******/
    /**********************************************/
    for(int i=1;i<lines_number;i++){
      num_interc=0;
      dist=-1;
      //Determinación de intercepciones de la recta surco con las rectas del contorno
      for(int j=0;j<num_points;j++){
        if(lines->at(j).find_interception(&row_line.line_)){
          if(lines->at(j).intercept_point.x<=x_max+2&&lines->at(j).intercept_point.x>=x_min-2&&
             lines->at(j).intercept_point.y<=y_max+2&&lines->at(j).intercept_point.y>=y_min-2){
            //calculo de la distancia de la intercepción al ultimo punto de la linea anterior
            //modificar punto
            dist_temp=calculate_distance(lines->at(j).intercept_point.x,lines->at(j).intercept_point.y,
                                         flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).at<double>(0,0),
                                         flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).at<double>(1,0));
            //condición para inicialización
            if(dist<0){
              dist=dist_temp;
              line_begin_index=j;
            }
            //Se selecciona la intercepción cuya distancia al punto de la linea anterior sea la menor
            //siendo esta la corresponiente al punto inicial parcial de la siguiente linea
            if(dist_temp<dist){
              dist=dist_temp;
              line_begin_index=j;
            }
            //Se almacenan los indices de aquellas intercepciones que estan dentro de los limites
            //asi como el numero de intercepciones
            interc_index.at(num_interc)=j;
            num_interc++;
          }
        }
      }
      if(num_interc>1){
        dist=-1;
        for(int j=0;j<num_interc;j++){
          //Se calcula la distancia del punto inicial a las demas intercepciones
          dist_temp=calculate_distance(lines->at(interc_index.at(j)).intercept_point.x,lines->at(interc_index.at(j)).intercept_point.y,
                                       lines->at(line_begin_index).intercept_point.x,
                                       lines->at(line_begin_index).intercept_point.y);
          if(dist<0){
            dist=dist_temp;
            line_end_index=j;
          }
          //Se seleccione la intercepción cuya distancia al punto inicial sea la mayor
          if(dist_temp>dist){
            dist=dist_temp;
            line_end_index=j;
          }
        }

        //Si la distancia es mayor a la base se calcula el numero de lineas.
        if(dist>base){
          //Calcular número de fotografías
          noph=(uint)(ceil(dist/base)+1);//se calcula el numero de fotografias para la linea inicial
        }
        else{
          //Calcular número de fotografías
          noph=2;
        }
        //flight_lines.at(i).resize(noph+addittional_wp,cv::Mat(3,1,CV_64F));
        flight_lines.at(i).resize(noph+addittional_wp);
        // A partir del vertice se retrocede la distancia de la mitad de los puntos adicionales, en la dirección opuesta al vector de avance, para luego generar los demas
            //puntos llendo en la dirección de avance
        flight_lines[i][0]=cv::Mat(3,1,CV_64F);
        flight_lines.at(i).at(0).row(0).col(0)=lines->at(line_begin_index).intercept_point.x+(pow(-1,i+1))*addittional_wp/2*advance_vector.at<double>(0,0);
        flight_lines.at(i).at(0).row(1).col(0)=lines->at(line_begin_index).intercept_point.y+(pow(-1,i+1))*addittional_wp/2*advance_vector.at<double>(1,0);
        flight_lines.at(i).at(0).row(2).col(0)=flight_height;
        flight_lines.at(i).at(0).row(2).col(0)=0;
        //std::cout<<flight_lines.at(i).at(0).at<double>(0,0)<<"\t"<<flight_lines.at(i).at(0).at<double>(1,0)<<std::endl;
        for(int j=1;j<flight_lines.at(i).size();j++){
          flight_lines[i][j]=cv::Mat(3,1,CV_64F);
          flight_lines[i][j].row(0).col(0)=flight_lines.at(i).at(j-1).at<double>(0,0)+(pow(-1,i))*advance_vector.at<double>(0,0);
          flight_lines[i][j].row(1).col(0)=flight_lines.at(i).at(j-1).at<double>(1,0)+(pow(-1,i))*advance_vector.at<double>(1,0);
          flight_lines[i][j].row(2).col(0)=flight_height;
          flight_lines[i][j].row(2).col(0)=0;
          //std::cout<<flight_lines.at(i).at(j).at<double>(0,0)<<"\t"<<flight_lines.at(i).at(j).at<double>(1,0)<<std::endl;
        }
      }
      else if(num_interc==1){
        noph=2;
        //flight_lines.at(i).resize(noph+addittional_wp,cv::Mat(3,1,CV_64F));
        flight_lines.at(i).resize(noph+addittional_wp);
        // A partir del vertice se retrocede la distancia de la mitad de los puntos adicionales, en la dirección opuesta al vector de avance, para luego generar los demas
            //puntos llendo en la dirección de avance
        flight_lines[i][0]=cv::Mat(3,1,CV_64F);
        flight_lines.at(i).at(0).row(0).col(0)=lines->at(line_begin_index).intercept_point.x+(pow(-1,i+1))*addittional_wp/2*advance_vector.at<double>(0,0);
        flight_lines.at(i).at(0).row(1).col(0)=lines->at(line_begin_index).intercept_point.y+(pow(-1,i+1))*addittional_wp/2*advance_vector.at<double>(1,0);
        flight_lines.at(i).at(0).row(2).col(0)=flight_height;
        flight_lines.at(i).at(0).row(2).col(0)=0;
        //std::cout<<flight_lines.at(i).at(0).at<double>(0,0)<<"\t"<<flight_lines.at(i).at(0).at<double>(1,0)<<std::endl;
        for(int j=1;j<flight_lines.at(i).size();j++){
          flight_lines[i][j]=cv::Mat(3,1,CV_64F);
          flight_lines[i][j].row(0).col(0)=flight_lines.at(i).at(j-1).at<double>(0,0)+(pow(-1,i))*advance_vector.at<double>(0,0);
          flight_lines[i][j].row(1).col(0)=flight_lines.at(i).at(j-1).at<double>(1,0)+(pow(-1,i))*advance_vector.at<double>(1,0);
          flight_lines[i][j].row(2).col(0)=flight_height;
          flight_lines[i][j].row(2).col(0)=0;
          //std::cout<<flight_lines.at(i).at(j).at<double>(0,0)<<"\t"<<flight_lines.at(i).at(j).at<double>(1,0)<<std::endl;

        }
      }
      else{
        dist=-1;
        for(int j=0;j<num_points;j++){
          if(lines->at(j).intercpt_found){
              //calculo de la distancia de la intercepción al ultimo punto de la linea anterior
              //modificar punto
              dist_temp=calculate_distance(lines->at(j).intercept_point.x,lines->at(j).intercept_point.y,
                                           flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).at<double>(0,0),
                                           flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).at<double>(1,0));
              //condición para inicialización
              if(dist<0){
                dist=dist_temp;
                line_begin_index=j;
              }
              //Se selecciona la intercepción cuya distancia al punto de la linea anterior sea la menor
              //siendo esta la corresponiente al punto inicial parcial de la siguiente linea
              if(dist_temp<dist){
                dist=dist_temp;
                line_begin_index=j;
              }
              //Se almacenan los indices de aquellas intercepciones que estan dentro de los limites
              //asic omo el numero de intercepciones
          }
        }
        dist=-1;
        for(int j=0;j<num_points;j++){
          //Se calcula la distancia del punto inicial a las demas intercepciones
          if(j!=line_begin_index){
            dist_temp=calculate_distance(lines->at(j).intercept_point.x,lines->at(j).intercept_point.y,
                                         lines->at(line_begin_index).intercept_point.x,
                                         lines->at(line_begin_index).intercept_point.y);
            if(dist<0){
              dist=dist_temp;
              line_end_index=j;
            }
            //Se seleccione la intercepción cuya distancia al punto inicial sea la menor
            if(dist_temp<dist){
              dist=dist_temp;
              line_end_index=j;
            }
          }

        }
        /*std::cout<<"intercepción inicial\nx: "<<lines->at(line_begin_index).intercept_point.x<<"\ty: "<<lines->at(line_begin_index).intercept_point.y<<std::endl;
        std::cout<<"intercepción final\nx: "<<lines->at(line_end_index).intercept_point.x<<"\ty: "<<lines->at(line_end_index).intercept_point.y<<std::endl;
        std::cout<<"dist: "<<dist<<std::endl;*/
        if(dist>base){
          //Calcular numero de fotografías
          noph=(uint)(ceil(dist/base)+1);//se calcula el numero de fotografias para la linea inicial
        }
        else{
          row_vector->copyTo(advance_vector);
          noph=2;
        }
        //flight_lines.at(i).resize(noph+addittional_wp,cv::Mat(3,1,CV_64F));
        flight_lines.at(i).resize(noph+addittional_wp);
        // A partir del vertice se retrocede la distancia de la mitad de los puntos adicionales, en la dirección opuesta al vector de avance, para luego generar los demas
            //puntos llendo en la dirección de avance
        flight_lines[i][0]=cv::Mat(3,1,CV_64F);
        flight_lines.at(i).at(0).row(0).col(0)=lines->at(line_begin_index).intercept_point.x+(pow(-1,i+1))*addittional_wp/2*advance_vector.at<double>(0,0);
        flight_lines.at(i).at(0).row(1).col(0)=lines->at(line_begin_index).intercept_point.y+(pow(-1,i+1))*addittional_wp/2*advance_vector.at<double>(1,0);
        flight_lines.at(i).at(0).row(2).col(0)=flight_height;
        flight_lines.at(i).at(0).row(2).col(0)=0;
        //std::cout<<flight_lines.at(i).at(0).at<double>(0,0)<<"\t"<<flight_lines.at(i).at(0).at<double>(1,0)<<std::endl;
        for(int j=1;j<flight_lines.at(i).size();j++){
          flight_lines[i][j]=cv::Mat(3,1,CV_64F);
          flight_lines[i][j].row(0).col(0)=flight_lines.at(i).at(j-1).at<double>(0,0)+(pow(-1,i))*advance_vector.at<double>(0,0);
          flight_lines[i][j].row(1).col(0)=flight_lines.at(i).at(j-1).at<double>(1,0)+(pow(-1,i))*advance_vector.at<double>(1,0);
          flight_lines[i][j].row(2).col(0)=flight_height;
          flight_lines[i][j].row(2).col(0)=0;
          //std::cout<<flight_lines.at(i).at(j).at<double>(0,0)<<"\t"<<flight_lines.at(i).at(j).at<double>(1,0)<<std::endl;
        }
      }
      //desplazar linea
      row_line.line_move(side_vector->at<double>(0,0),side_vector->at<double>(1,0));
    }
    //Inicialización primera linea


    *waypoints=flight_lines;
    return;
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
      georef_image_point(cam_ori,imu_ori,uav_gps,k,pixel_coord.at(i),ref_gps,
                         &georef_point,are_degrees,scale_number,fl,true);
      cvMat2gps(&res.gps_coors[i],&georef_point);
    }
    return  true;
  }

  uint  find_initial_point_and_side_vector(std::vector<cv::Mat>* navf_points,std::vector<line_class>* lines,
                                           cv::Mat* row_vector,double x_max,double y_max,double x_min,double y_min,
                                           cv::Mat* side_vector, double line_space,uint* line_number, bool debug){
    uint  index=0;
    uint  num_points=navf_points->size();
    line_class  row_line;
    std::vector<uint> num_lines;
    std::vector<cv::Mat>  direction;
    double  n_lines;
    double  num_interc;
    double  thres=1;
    num_lines.resize(num_points);
    direction.resize(num_points);
    std::cout<<"row vector"<<*row_vector<<std::endl;
    for(int i=0;i<num_points;i++){
      //std::cout<<"+++CORNER "<<i<<" +++"<<std::endl;
      //std::cout<<"point :\n"<<navf_points->at(i)<<std::endl;

      n_lines=0;
      direction.at(i)=cv::Mat(2,1,CV_64F);
      num_lines.at(i)=0;
      row_line=line_class(navf_points->at(i).at<double>(0,0),navf_points->at(i).at<double>(1,0),row_vector);
      rotate_vetor(row_vector,side_vector,line_space/5,CV_PI/2,false);
      side_vector->copyTo(direction.at(i));
      do{
        num_interc=0;
        for(int j=0;j<num_points;j++){
          if(lines->at(j).find_interception(&row_line.line_)){
            if(lines->at(j).intercept_point.x<=x_max+thres&&lines->at(j).intercept_point.x>=x_min-thres&&
               lines->at(j).intercept_point.y<=y_max+thres&&lines->at(j).intercept_point.y>=y_min-thres){
               num_interc++;
            }
          }
        }
        if(num_interc>=2){
          n_lines++;
        }
        row_line.line_move(side_vector->at<double>(0,0),side_vector->at<double>(1,0));

      }while (num_interc>=2);
      //std::cout<<"n lines(90) "<<n_lines<<std::endl;
      side_vector->copyTo(direction.at(i));
      num_lines.at(i)=n_lines;

      n_lines=0;
      row_line=line_class(navf_points->at(i).at<double>(0,0),navf_points->at(i).at<double>(1,0),row_vector);
      rotate_vetor(row_vector,side_vector,line_space/5,-CV_PI/2,false);
      do{
        num_interc=0;
        for(int j=0;j<num_points;j++){
          if(lines->at(j).find_interception(&row_line.line_)){
            if(lines->at(j).intercept_point.x<=x_max&&lines->at(j).intercept_point.x>=x_min&&
               lines->at(j).intercept_point.y<=y_max&&lines->at(j).intercept_point.y>=y_min){
               num_interc++;
            }
          }
        }
        if(num_interc>=2){
          n_lines++;
        }
        row_line.line_move(side_vector->at<double>(0,0),side_vector->at<double>(1,0));
      }while (num_interc>=2);
      //std::cout<<"n lines(-90) "<<n_lines<<std::endl;
      if(num_lines.at(i)<n_lines){
        side_vector->copyTo(direction.at(i));
        num_lines.at(i)=n_lines;
      }
      //std::cout<<"side vec"<<direction.at(i)<<std::endl;
      //std::cout<<"\nn l: "<<num_lines.at(i)<<std::endl;

    }
    index=0;
    direction.at(0).copyTo(*side_vector);
    for(int i=1;i<num_points;i++){
      if(num_lines.at(i)>num_lines.at(index)){
        index=i;
        direction.at(index).copyTo(*side_vector);
      }
    }
    rotate_vetor(side_vector,side_vector,line_space,0,false);
    //if(true){std::cout<<"Index: "<<index<<"\tnum lines:"<<num_lines.at(index)<<"\tside vector: \n"<<*side_vector<<std::endl;}
    *line_number=(uint)(ceil(num_lines.at(index)/5.0)+1.0);
    return  index;
  }
  void  rotate_vetor(cv::Mat* orig ,cv::Mat* rotated,double magnitude, double  rotation,bool are_degrees){
    double theta;
    if(are_degrees){
      rotation*=CV_PI/180.0;
    }
    //std::cout<<"orig "<<*orig<<std::endl;
    theta=atan2(orig->at<double>(1,0),orig->at<double>(0,0));
    //std::cout<<"ang "<<theta*180/CV_PI<<std::endl;
    if(theta<0){
      theta+=2*CV_PI;
    }
    theta+=rotation;
    polar2cart(rotated,magnitude,theta,false);
    //std::cout<<"rotated "<<*rotated<<std::endl;
    return;
  }

  double calculate_height(double fl,double gsd,double sw,double iw){
    /****
     * f: focal lenght
     *gsd: ground sample distance
     * iw: Image width
     * sw: Sensor width
     * Fh: flight height
     *
     *FH=gsd*f*iw/sw;
     *
     * */
    double  height;
    height=gsd*fl*iw/sw;
    return  height;
  }

  double  calculate_gsd(double fl,double fh,double sw,double iw){
    /****
     * f: focal lenght
     *gsd: ground sample distance
     * iw: Image width
     * sw: Sensor width
     * Fh: flight height
     *
     * */
    double gsd;
    gsd=fh*sw/(fl*iw);
    return  gsd;
  }


  void  polar2cart(double *x,double* y,double r,double theta,bool is_deg){
    if(is_deg){
      theta*=M_PI/180.0;
    }
    *x=r*cos(theta);
    *y=r*sin(theta);
    return;
  }
  void  polar2cart(cv::Mat *vector,double r,double theta,bool is_deg){
    if(is_deg){
      theta*=M_PI/180.0;
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
      //if (debug){};
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
          cam_ori*=M_PI/180.0;
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
          y=IMU_ori.at<double>(0,0)*M_PI/180.0;
          p=IMU_ori.at<double>(1,0)*M_PI/180.0;
          r=IMU_ori.at<double>(2,0)*M_PI/180.0;
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
          y=IMU_ori.at<double>(0,0)*M_PI/180.0;
          p=IMU_ori.at<double>(1,0)*M_PI/180.0;
          r=IMU_ori.at<double>(2,0)*M_PI/180.0;
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
          lat=gps_position.at<double>(0,0)*M_PI/180.0;
          lon=gps_position.at<double>(1,0)*M_PI/180.0;
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
          lat=gps_position.at<double>(0,0)*M_PI/180.0;
          lon=gps_position.at<double>(1,0)*M_PI/180.0;
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
          lat=gps_position.at<double>(0,0)*M_PI/180.0;
          lon=gps_position.at<double>(1,0)*M_PI/180.0;
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
          lat=gps_position.at<double>(0,0)*M_PI/180.0;
          lon=gps_position.at<double>(1,0)*M_PI/180.0;
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
      while(fabs((lat-lat_old)*180.0/M_PI)>preci){
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
      uav_gps_posi->row(0).col(0)=lat*180.0/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180.0/M_PI;
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


      uav_gps_posi->row(0).col(0)=lat*180.0/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180.0/M_PI;
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
      uav_gps_posi->row(0).col(0)=lat*180.0/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180.0/M_PI;
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
      uav_gps_posi->row(0).col(0)=lat*180.0/M_PI;
      uav_gps_posi->row(1).col(0)=lon*180.0/M_PI;
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

      //std::cout<<"gps_ref:\n"<<ref_gps_posi<<std::endl;
      //Pasar del sc geodetico (coor gps) al sc ECEF
      geod2ecef(uav_gps_posi,&uav_ecef_posi,is_degrees);
      //std::cout<<"ecif posi:\n"<<uav_ecef_posi<<std::endl;
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);
      //std::cout<<"ecif ref:\n"<<ref_point_ecef<<std::endl;

      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      //update_ECEF2NF(uav_gps_posi,&ECEF2NF_tf,is_degrees);
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      //std::cout<<"Ref poitn navf: "<<ref_point_navf<<std::endl;
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
      //std::cout<<"k\n"<<k<<std::endl;
      //std::cout<<"gps_ref:\n"<<ref_gps_posi<<std::endl;
      //Trasladar coordenadas de imagen a camara
      cam_coord.row(0).col(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).col(0)=-(image_coord.at<double>(1,0)-k.at<double>(1,2));
      cam_coord.row(2).col(0)=image_coord.at<double>(2,0);
      if (debug){std::cout<<"uav_gps: \n"<<uav_gps_posi<<std::endl;}
      if (debug){std::cout<<"uav_ori: \n"<<imu_ori<<std::endl;}
      if (debug){std::cout<<"cam coor: \n"<<cam_coord<<std::endl;}

      //Pasar del frame de la camara al frame del body (UAV)
      update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);

      //Pasar del frame del body al frame de navegación
      update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
      //tranformación completa del frame de la camara al frame de navegación
      CF2NF_tf=BF2NF_tf*CF2BF_tf;
      cv::invert(CF2NF_tf,NF2CF_tf);
      //Pasar del sc geodetico (coor gps) al sc ECEF
      if (debug){std::cout<<"cf2nf: \n"<<CF2NF_tf<<std::endl;}
      geod2ecef(uav_gps_posi,&uav_ecef_posi,is_degrees);
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);
      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      if (debug){std::cout<<"ecef2nf: \n"<<ECEF2NF_tf<<std::endl;}
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      if (debug){std::cout<<"ref point _navf : \n"<<ref_point_navf<<std::endl;}
      uav_navf_posi=ECEF2NF_tf*(uav_ecef_posi-ref_point_ecef);
      if (debug){std::cout<<"uav_navf posi: \n"<<uav_navf_posi<<std::endl;}
      cv::Mat geor_coor_navf=cv::Mat(3,1,CV_64F);
      //uav_navf_posi.row(2).col(0)=0;
      geor_coor_navf=scale*CF2NF_tf*cam_coord+uav_navf_posi;

      if (debug){std::cout<<"coord navf:\n"<< geor_coor_navf<<std::endl;}
      navf_point->row(0).col(0)=geor_coor_navf.at<double>(0,0);
      navf_point->row(1).col(0)=geor_coor_navf.at<double>(1,0);
      navf_point->row(2).col(0)=geor_coor_navf.at<double>(2,0);
      return;
  }

  void navf2if(double cam_ori, cv::Mat imu_ori,
                          cv::Mat k,cv::Mat navf_coord,
                          cv::Mat* image_point, bool is_degrees, double scale, double fl,bool debug){
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
      cv::Mat CF2NF_tf=cv::  Mat::zeros(3,3,CV_64F);
      cv::Mat NF2CF_tf=cv::Mat::zeros(3,3,CV_64F);
      cv::Mat cam_coord=cv::Mat(3,1,CV_64F);
      //Pasar del frame de la camara al frame del body (UAV)
      update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);
      //Pasar del frame del body al frame de navegación
      update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
      //tranformación completa del frame de la camara al frame de navegación
      CF2NF_tf=BF2NF_tf*CF2BF_tf;
      cv::invert(CF2NF_tf,NF2CF_tf);
      cam_coord=NF2CF_tf*navf_coord;


      image_point->row(0).col(0)=cam_coord.at<double>(0,0)+k.at<double>(0,2);
      image_point->row(1).col(0)=cam_coord.at<double>(0,0)+k.at<double>(1,2);
      image_point->row(2).col(0)=fl_pixel;
      if (debug){std::cout<<"Georef coord navf:\n"<< image_point<<std::endl;}
      return;
  }


  void georef_nav_point( cv::Mat navf_coord, cv::Mat ref_gps_posi,
                          cv::Mat* georeferenced_point, bool is_degrees, bool debug){
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
      cv::Mat ref_point_ecef=cv::Mat(3,1,CV_64F);
      //Pasar del sc geodetico (coor gps) al sc ECEF
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);

      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      //update_ECEF2NF(uav_gps_posi,&ECEF2NF_tf,is_degrees);
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      cv::Mat geor_coor_ecef=cv::Mat(3,1,CV_64F);
      cv::Mat geor_coor_geod=cv::Mat(3,1,CV_64F);
      //std::cout<<"Georef coord navf:\n"<< geor_coor_navf<<std::endl;
      geor_coor_ecef=ref_point_ecef+ECEF2NF_tf.inv()*navf_coord;
      //std::cout<<"Georef coord ecef:\n"<< geor_coor_ecef<<std::endl;
      ecef2geod_2(&geor_coor_geod,geor_coor_ecef);
      if(debug){
        std::cout<<"Georef coord geodf:\n"<< geor_coor_geod<<std::endl;
      }
      georeferenced_point->row(0).col(0)=geor_coor_geod.at<double>(0,0);
      georeferenced_point->row(1).col(0)=geor_coor_geod.at<double>(1,0);
      georeferenced_point->row(2).col(0)=geor_coor_geod.at<double>(2,0);
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

      //std::cout<<"k\n"<<k<<std::endl;
      //std::cout<<"gps_ref:\n"<<ref_gps_posi<<std::endl;

      //Trasladar coordenadas de imagen a camara
      /*cam_coord.row(0).row(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).row(0)=image_coord.at<double>(1,0)-k.at<double>(1,2);
      cam_coord.row(2).row(0)=image_coord.at<double>(2,0);*/

      cam_coord.row(0).col(0)=image_coord.at<double>(0,0)-k.at<double>(0,2);
      cam_coord.row(1).col(0)=-(image_coord.at<double>(1,0)-k.at<double>(1,2));
      cam_coord.row(2).col(0)=image_coord.at<double>(2,0);
      //std::cout<<"cam coor: "<<cam_coord<<std::endl;

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
      //std::cout<<"ecif posi:\n"<<uav_ecef_posi<<std::endl;
      geod2ecef(ref_gps_posi,&ref_point_ecef,is_degrees);
      //std::cout<<"ecif ref:\n"<<ref_point_ecef<<std::endl;

      //Pasar la ubicación del UAV del sc ECEF al sc de navegación
      //update_ECEF2NF(uav_gps_posi,&ECEF2NF_tf,is_degrees);
      update_ECEF2NF_2(ref_gps_posi,&ECEF2NF_tf,is_degrees);
      ref_point_navf=ECEF2NF_tf*(ref_point_ecef-ref_point_ecef);
      //std::cout<<"Ref poitn navf: "<<ref_point_navf<<std::endl;
      uav_navf_posi=ECEF2NF_tf*(uav_ecef_posi-ref_point_ecef);
      //std::cout<<"uav navf posi: "<<uav_navf_posi<<std::endl;
      cv::Mat geor_coor_navf=cv::Mat(3,1,CV_64F);
      cv::Mat geor_coor_ecef=cv::Mat(3,1,CV_64F);
      cv::Mat geor_coor_geod=cv::Mat(3,1,CV_64F);

      //uav_navf_posi.row(2).col(0)=0;
      geor_coor_navf=scale*CF2NF_tf*cam_coord+uav_navf_posi;
      //std::cout<<"Georef coord navf:\n"<< geor_coor_navf<<std::endl;
      geor_coor_ecef=ref_point_ecef+ECEF2NF_tf.inv()*geor_coor_navf;
      //std::cout<<"Georef coord ecef:\n"<< geor_coor_ecef<<std::endl;
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

  std::setprecision (10);
  ros::init(argc, argv, "test");
  std::setprecision(6);
  FlightPlanner fp;
  ROS_INFO("ready to geref points or plan flight");

  ros::spin();

  return 0;
}

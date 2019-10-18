#include "flight_planner.h"

/*****************************************************************
 *                  metodos line class                           *
 * **************************************************************/
line_class::line_class(){
  line_=cv::Mat(2,2,CV_64F);
}

line_class::line_class(double  x1,double y1,double x2,double y2){
  //ecuación parametrica recta columna 0 componente x; columna 1 componente y
  //fila 0 punto inicial  fila 1 vector director
  line_=cv::Mat(2,2,CV_64F);
  line_.row(0).col(0)=x1;
  line_.row(0).col(1)=y1;
  line_.row(1).col(0)=x2-x1;
  line_.row(1).col(1)=y2-y1;
  intercect_found=false;
}

line_class::line_class(double  x1,double y1,cv::Mat* vector){
  //ecuación parametrica recta columna 0 componente x; columna 1 componente y
  line_=cv::Mat(2,2,CV_64F);
  line_.row(0).col(0)=x1;
  line_.row(0).col(1)=y1;
  line_.row(1).col(0)=vector->at<double>(0,0);
  line_.row(1).col(1)=vector->at<double>(1,0);
  intercect_found=false;
}
line_class::line_class(double  x1,double y1,geometry_msgs::Vector3* vector){
  //ecuación parametrica recta columna 0 componente x; columna 1 componente y
  line_=cv::Mat(2,2,CV_64F);
  line_.row(0).col(0)=x1;
  line_.row(0).col(1)=y1;
  line_.row(1).col(0)=vector->x;
  line_.row(1).col(1)=vector->y;
  intercect_found=false;
}

uint line_class::find_interception(bg_polygon poly){
  uint  num_interc=0;
  intercect_points_.clear();
  if(!bg::intersects(segment_,poly)){
      intercect_found=false;
      return  num_interc;
  }
  bg::intersection(segment_, poly, intercect_points_);
  intercect_found=true;
  num_interc=intercect_points_.size();
  #ifdef DEBUG
  std::cout << "Interception: \n";
  for(int i=0;i<num_interc;i++){
    std::cout<< bg::dsv(intercect_points_.at(i)) << std::endl;
  }
  #endif
  return  num_interc;
}
bool line_class::find_interception(bg_line line){
  bool  success=false;
  if(!bg::intersects(segment_,line)){
      intercect_found=false;
      return  success;
  }
  bg::intersection(segment_, line, intercect_points_);
  intercect_found=success;
  return  success;
}


/// finds the interception between the line to intercept and the line
bool line_class::find_interception(cv::Mat* line_to_int){
  bool  success=false;
  cv::Mat A=cv::Mat(2,2,CV_64F);
  cv::Mat vec=cv::Mat(2,1,CV_64F);
  cv::Mat t=cv::Mat(2,1,CV_64F);
  intercect_found=false;
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
    intercect_found=false;
    return success;
  }
  intercect_points_.resize(1);
  intercect_points_[0]=bg_point(line_.at<double>(0,0)+line_.at<double>(1,0)*t.at<double>(0,0),
                               line_.at<double>(0,1)+line_.at<double>(1,1)*t.at<double>(0,0));
  /*intercept_points.at(0).x(line_.at<double>(0,0)+line_.at<double>(1,0)*t.at<double>(0,0));
  intercept_points.at(0).y(line_.at<double>(0,1)+line_.at<double>(1,1)*t.at<double>(0,0));*/
  success=true;
  intercect_found=success;
  return  success;
}
void  line_class::move_line(double x,double y){
  line_.row(0).col(0)+=x;
  line_.row(0).col(1)+=y;
  return;
}
bool  line_class::find_y(double x, double* y){
  bool  uniq_solution=false;
  //Se verifica que la componente del vector director no sea nula(tiende a cero)
  //if(line_.at<double>(1,0)){
  if(line_.at<double>(1,0)==0){
    return  uniq_solution;
  }
  double  t=((x-line_.at<double>(0,0))/line_.at<double>(1,0));
  *y=line_.at<double>(0,1)+line_.at<double>(1,1)*t;
  #ifdef DEBUG
  //std::cout << "y: " << *y << std::endl;
  #endif
  uniq_solution=true;
  return uniq_solution;
}
void  line_class::set_segment(double x1, double y1, double x2,double y2){
  bg_point  p1(x1,y1),p2(x2,y2);
  bg_line line_aux;
  bg::append(line_aux,p1);
  bg::append(line_aux,p2);
  segment_=line_aux;
  #ifdef DEBUG
  //std::cout << "Segment: " << bg::dsv(segment_) << std::endl;
  #endif
  return;
}

/*****************************************************************
 *                  metodos contour class                        *
 * **************************************************************/
contour_class::contour_class(){
  area_=0;
}

contour_class::contour_class(std::vector<geometry_msgs::Vector3> contour_vec, bool is_closed){
  for(int i=0;i<contour_vec.size();i++){
    bg_point  pt(contour_vec.at(i).x,contour_vec.at(i).y);
    bg::append(polygon_,pt);
  }
  if(!is_closed){
    bg_point  pt(contour_vec.at(0).x,contour_vec.at(0).y);
    bg::append(polygon_,pt);
  }
  area_=get_area();
  get_centroid(&centroid_);
}

contour_class::contour_class(crop_image_processing::Crop crop, bool is_closed){
  for(int i=0;i<crop.contour.size();i++){
    bg_point  pt(crop.contour.at(i).x,crop.contour.at(i).y);
    bg::append(polygon_,pt);
  }
  if(!is_closed){
    bg_point  pt(crop.contour.at(0).x,crop.contour.at(0).y);
    bg::append(polygon_,pt);
  }

  area_=get_area();
  get_centroid(&centroid_);
}

double  contour_class::get_area(){
  double  area=0;
  try{
    area=bg::area(polygon_);
  }
  //To catch general exceptions
  catch(...){
    std::cout<<"exception found"<<std::endl;
    area_=area;
    return area;
  }
  area_=area;
  return  area;
}
void  contour_class::get_centroid(bg_point *centroid){
  try{
    bg::centroid(polygon_,centroid);
  }
  //To catch general exceptions
  catch(...){
    std::cout<<"exception found"<<std::endl;
    return;
  }
  centroid_=*centroid;
  return;
}
void contour_class::append(double x, double y){
  bg::append(polygon_,bg_point(x,y));
  return;
}
/*****************************************************************
 *                  metodos flight planner                       *
 * **************************************************************/
flight_planner::flight_planner():
  rate_(kDefaultRate)
  {
    ros::NodeHandle pnh("~");
    std::string plan_flight_topic;
    std::string georef_points_topic;
    pnh.param("PlanFlightTopic",plan_flight_topic,kDefaultPlanFlightTopic);
    pnh.param("LocalPoseSubTopic",georef_points_topic,kDefaultGeorefPointsTopic);
    i_width=1280;
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
    //debug_=true;
    //navf=gl::LocalCartesian(ref_gps_pose.x,ref_gps_pose.y,
    //                        ref_gps_pose.z,ecef);
    navf_=gl::LocalCartesian(ecef);
    planner=nh.advertiseService(plan_flight_topic, &flight_planner::plannerCB,this);
    georeferencer=nh.advertiseService(georef_points_topic, &flight_planner::georeferencerCB,this);
  }
///Object destructor
flight_planner::~flight_planner()
{

}

///call back that handles the process to georeference a point
bool  flight_planner::georeferencerCB(flight_planning::georef_points::Request &req,
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
  std::vector<geometry_msgs::Vector3> pixel_coord;
  //inicializar variables
  uav_alt=uav_gps_pose.z-ref_gps_pose.z;
  focal_plane_res=i_width/s_width;
  fl_pixel=focal_plane_res*fl;
  scale_number=uav_alt/fl_pixel;
  image_scale=1/scale_number;
  pixel_coord.resize(numPoints);
  for(int i=0;i<numPoints;i++){
    pixel_coord.at(i).x=req.im_coors[i].x;
    pixel_coord.at(i).y=req.im_coors[i].y;
    pixel_coord.at(i).z=fl_pixel;
  }
  res.gps_coors.resize(numPoints);
  for(int i=0;i<numPoints;i++){
    georef_image_point(cam_ori,imu_uav,uav_gps_pose,k,pixel_coord.at(i),ref_gps_pose,
                       &res.gps_coors[i],are_degrees,scale_number,fl);
  }
  #ifdef DEBUG
  std::cout<<res<<std::endl;
  #endif
  return  true;
}


///call back that handles the flight planning
bool flight_planner::plannerCB(flight_planning::generate_plan::Request &req,
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
  std::vector<geometry_msgs::Vector3> crop_pixel_coord;
  std::vector<cv::Mat> plan_pixel_coord;
  geometry_msgs::Vector3 crop_row_vector_navf;//=cv::Mat(3,1,CV_64F);
  geometry_msgs::Vector3 crop_row_vector_image;//=cv::Mat(3,1,CV_64F);
  geometry_msgs::Vector3 side_vector;//=cv::Mat(3,1,CV_64F);
  std::vector<std::vector<geometry_msgs::Vector3> > navf_plan_waypoints;
  contour_class crop_contour_;
  contour_class bounding_contour_;
  res.success=false;

  uint  lines_number;
  //inicializar variables
  uav_alt=uav_gps_pose.z-ref_gps_pose.z;
  focal_plane_res=i_width/s_width;
  fl_pixel=focal_plane_res*fl;
  scale_number=uav_alt/fl_pixel;
  image_scale=1/scale_number;
  uint  numCropPoints=req.crop.contour.size();
  crop_pixel_coord.resize(numCropPoints);

  double  gr_height_cover;//ground coverage according to desired gsd
  double  gr_width_cover;

  endlap=req.endlap;
  sidelap=req.sidelap;
  geometry_msgs::Vector3  tempVec;
  #ifdef DEBUG
  std::cout<<req<<std::endl;
  #endif
  /*******************************************************************************/
  #ifdef DEBUG
  std::cout<<"\n+++Calculo varibles fotogrometricas+++"<<std::endl;
  #endif
  gr_height_cover=i_height*gsd;
  #ifdef DEBUG
  std::cout<<"ground height coverrage\n"<<gr_height_cover<<std::endl;
  #endif
  gr_width_cover=i_width*gsd;
  #ifdef DEBUG
  std::cout<<"ground width coverrage\n"<<gr_width_cover<<std::endl;
  #endif
  base=gr_height_cover*(1-endlap/100);
  #ifdef DEBUG
  std::cout<<"base\n"<<base<<std::endl;
  #endif
  line_space=gr_width_cover*(1-sidelap/100);
  #ifdef DEBUG
  std::cout<<"line space\n"<<line_space<<std::endl;
  #endif

  /*#ifdef DEBUG
  std::cout<<"\n+++Calculo varibles fotogrometricas+++"<<std::endl;
  std::cout<<"ground height coverrage\n"<<gr_height_cover<<std::endl;
  std::cout<<"ground width coverrage\n"<<gr_width_cover<<std::endl;
  std::cout<<"base\n"<<base<<std::endl;
  std::cout<<"line space\n"<<line_space<<std::endl;
  #endif*/
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
    crop_pixel_coord.at(i).x=req.crop.contour.at(i).x;
    crop_pixel_coord.at(i).y=req.crop.contour.at(i).y;
    crop_pixel_coord.at(i).z=fl_pixel;
  }
  res.crop_gps_coors.resize(numCropPoints);
  std::vector<geometry_msgs::Vector3>  crop_navf_coors;
  crop_navf_coors.resize(numCropPoints);
  #ifdef DEBUG
  //std::cout<<"\n++++GEOREFERENCIACIÓN VERTICES+++"<<std::endl;
  #endif

  std::fstream  crop_contour_file;
  crop_contour_file.open ("/home/daniel/Pictures/planning/crop.dat",std::ios::trunc |std::ios::out|std::ios::in);
  crop_contour_file<< "x\ty\n";
  for(int i=0;i<numCropPoints;i++){
    georef_image_point(cam_ori,imu_uav,uav_gps_pose,k,crop_pixel_coord.at(i),ref_gps_pose,
                       &res.crop_gps_coors[i],are_degrees,scale_number,fl);
    if2navf(cam_ori,imu_uav,uav_gps_pose,k,crop_pixel_coord.at(i),ref_gps_pose,
            &crop_navf_coors.at(i),are_degrees,scale_number,fl);
    crop_contour_file<<crop_navf_coors[i].x<<"\t"<<crop_navf_coors[i].y<<"\n";
  }

  crop_contour_file.close();

  #ifdef DEBUG
  //std::cout<<"\n+++TRANSFORMACIÓN VECTOR ORIENTACIÓN DEL IF AL NAVF+++"<<std::endl;
  //std::cout<<req.crop.row_orientation<<std::endl;
  #endif
  /***transformar el vector de orientación del cultivo***/

  //polar2cart(&crop_row_vector_image,1.0,req.crop.row_orientation+90,are_degrees);
  polar2cart(&crop_row_vector_image,1.0,req.crop.row_orientation,are_degrees);
  crop_row_vector_image.x+=k.at<double>(0,2);//se le agrega esta cantidad para que el vector quede en el origen de la camara
  crop_row_vector_image.y+=k.at<double>(1,2);
  crop_row_vector_image.z=fl_pixel;

  tempVec=ref_gps_pose;
  tempVec.z=uav_gps_pose.z;
  #ifdef DEBUG
  //std::cout<<"row vector image: \n"<<crop_row_vector_image<<std::endl;
  #endif

  if2navf(cam_ori,imu_uav,tempVec,k,crop_row_vector_image,ref_gps_pose,
          &crop_row_vector_navf,are_degrees,scale_number,fl);
  crop_row_vector_navf.z=0;
  normalize_2d(&crop_row_vector_navf);
  #ifdef DEBUG
  //std::cout<<"row vector navf unit: \n"<<crop_row_vector_navf<<std::endl;
  #endif
  crop_row_vector_navf.x*=base;
  crop_row_vector_navf.y*=base;
  #ifdef DEBUG
  //std::cout<<"row vector navf: \n"<<crop_row_vector_navf<<std::endl;
  #endif
  /***Determinación del punto inicial del cultivo***/
  #ifdef DEBUG
  //std::cout<<"\n+++DETERMINACIÓN max's min's x,y contorno+++"<<std::endl;
  #endif

  cv::Point2f min_vector;//=cv::Mat(2,1,CV_64F);
  cv::Point2f max_vector;//=cv::Mat(2,1,CV_64F);
  find_max_min_x_y(crop_navf_coors,&max_vector,&min_vector);
  #ifdef DEBUG
  //std::cout<<"min x: "<<min_vector.x<<"\tmin y"<<min_vector.y<<std::endl;
  //std::cout<<"max x: "<<max_vector.x<<"\tmax y"<<max_vector.y<<std::endl;
  std::cout<<"\n+++GENERACION CONTORNOS+++"<<std::endl;
  #endif
  bounding_contour_.append(min_vector.x,min_vector.y);
  bounding_contour_.append(min_vector.x,max_vector.y);
  bounding_contour_.append(max_vector.x,max_vector.y);
  bounding_contour_.append(max_vector.x,min_vector.y);
  bounding_contour_.append(min_vector.x,min_vector.y);

  crop_contour_=contour_class(crop_navf_coors,false);
  #ifdef DEBUG
  //std::cout << "Interception: " << geometry::dsv(intercep) << std::endl;
  std::cout<<"\nbounding contour: \n"<<bg::dsv(bounding_contour_.polygon_)<<std::endl;
  std::cout<<"\ncrop contour: \n"<<bg::dsv(crop_contour_.polygon_)<<std::endl;
  std::cout<<"\n+++DETERMINACIÓN PUNTO INICIAL+++"<<std::endl;
  #endif
  uint  initial_point_index;  
  initial_point_index=find_initial_point_and_side_vector(crop_navf_coors,crop_contour_,line_space,min_vector.x,max_vector.y,
                                                         min_vector.y,max_vector.y,&crop_row_vector_navf,&side_vector,&lines_number);
  #ifdef DEBUG
  std::cout<<"inicial point"<<crop_navf_coors.at(initial_point_index)<<"\nlines number: "<<lines_number<<std::endl;
  std::cout<<"\n+++GENERACIÓN PUNTOS+++"<<std::endl;
  #endif
  std::vector<double> heading_vector;
  generate_points(crop_navf_coors.at(initial_point_index),crop_contour_,bounding_contour_,base,
                  min_vector,max_vector, crop_row_vector_navf,side_vector,lines_number,&navf_plan_waypoints, &heading_vector);
  #ifdef DEBUG
  /*std::cout<<"x:\ty:\tangle"<<std::endl;
  for(int i=0;i<navf_plan_waypoints.size();i++){
    for(int j=0;j<navf_plan_waypoints.at(i).size();j++){
      std::cout<<navf_plan_waypoints.at(i).at(j).x<<"\t"<<navf_plan_waypoints.at(i).at(j).y<<std::endl;
    }
  }*/
  #endif
  /****Georeferenciación puntos generados*********/
  int counter=0;  
  #ifdef DEBUG
  //std::cout <<"angles: \n"<<std::endl;
  std::cout<<"x:\ty:\tangle"<<std::endl;
  #endif
  std::fstream  plan_file;
  plan_file.open ("/home/daniel/Pictures/planning/plan.dat",std::ios::trunc |std::ios::out|std::ios::in);
  plan_file<< "x\ty\n";
  for(int i=0 ;i<navf_plan_waypoints.size();i++){
    for(int j=0;j<navf_plan_waypoints.at(i).size();j++){
      #ifdef DEBUG
      std::cout<<navf_plan_waypoints.at(i).at(j).x<<"\t"<<
                 navf_plan_waypoints.at(i).at(j).y<<"\t"<<
                 heading_vector.at(counter)<<std::endl;
                 //heading_vector.at(counter)*180/M_PI<<std::endl;
      #endif
      geometry_msgs::Vector3  temp;
      plan_file<<navf_plan_waypoints[i][j].x<<"\t"<<navf_plan_waypoints[i][j].y<<"\n";
      georef_navf_point(navf_plan_waypoints.at(i).at(j),ref_gps_pose,
                         &temp,are_degrees);
      //*********************************
      //temp.z=ref_gps_pose.z+flight_height;
      res.plan_gps_coors.push_back(temp);
      counter++;
    }
  }
  res.uav_heading=heading_vector;
  plan_file.close();
  #ifdef DEBUG
  std::cout<<"\n+++FIN+++"<<std::endl;
  //std::cout<<res<<std::endl;
  #endif
  res.success=true;
  return  true;
}

// *************************************************
void  flight_planner::generate_points(geometry_msgs::Vector3 first_point,contour_class crop_contour,contour_class  bounding_contour,double base, cv::Point2f min_vector,cv::Point2f max_vector,
                                      geometry_msgs::Vector3 row_vector,geometry_msgs::Vector3 side_vector,uint lines_number,std::vector<std::vector<geometry_msgs::Vector3> >* waypoints,std::vector<double>* heading_vector){
  std::vector<std::vector<geometry_msgs::Vector3> >  flight_lines;//conjunto de lineas con los puntos del vuelo

  std::vector<std::vector<geometry_msgs::Vector3> >  flight_lines2;
  std::vector<double> heading_vector2;

  line_class  row_line;//recta cuyo vector director es la dirección del surco
  uint num_interc;//numero de intersecciones validas
  double  dist_max,dist_min;//distancia entre dos puntos p1,p2
  uint  noph;//number of photografies
  uint  addittional_wp=2;//Numero de waypoints adicionales
  geometry_msgs::Vector3 advance_vector;//=cv::Mat(3,1,CV_64F);//Vector de avance sibre las lineas
  uint  line_begin_index;//Encontrar un mejor nombre
  uint  line_end_index;
  double  y1,y2,x1,x2;

  heading_vector->clear();

  //inicialización recta
  row_line=line_class(first_point.x,first_point.y,&row_vector );
  //Esta condición verifica  si el sistema de ecuaciones asociado a la recta teien solución unica
  if(row_line.find_y(min_vector.x-10*fabs(min_vector.x),&y1)){
    row_line.find_y(max_vector.x+10*fabs(max_vector.x),&y2);
    x1=min_vector.x-10*fabs(min_vector.x);
    x2=max_vector.x+10*fabs(max_vector.x);
  }
  //Si no es asi, significa que la recta es paralela al eje y
  else{
    y1=min_vector.y-10.0*fabs(min_vector.y);
    y2=max_vector.y+10.0*fabs(max_vector.y);
    x1=row_line.line_.at<double>(0,0);
    x2=x1;
  }
  #ifdef DEBUG
  //std::cout<<"\tx1: "<<x1<<"\ty1: "<<y1<<"\tx2: "<<x2<<"\ty2: "<<y2<<std::endl;
  #endif

  row_line.set_segment(x1,y1,x2,y2);
  flight_lines.resize(lines_number);
  /**********************************************/
  /******   Inicialización primera linea   ******/
  /**********************************************/
  //Si al desplazarse en la dirección de row vector el punto no esta dentro del contorno, la dirección de avance es hacia el lado opuesto
  /*if(bg::within(bg_point(first_point.x+row_vector.x,first_point.y+row_vector.y),crop_contour.polygon_)){
    advance_vector=row_vector;
  }else{
    rotate_vetor(&row_vector,&advance_vector,base,M_PI,false);
  }*/
  //Se haya la interecepción del segmento de recta con el el contorno de los limites de maximos y minimos
  uint  index_max=0,index_min=0;
  num_interc=row_line.find_interception(bounding_contour.polygon_);
  #ifdef DEBUG
  //std::cout<<"num interc: "<<num_interc<<std::endl;
  #endif
  // Se determina cual punto de las intercepciones del contorno es el mas lejano para elegirlo como el punto incial
  dist_max=bg::distance(bg_point(first_point.x,first_point.y),row_line.intercect_points_.at(0));
  if(num_interc>1){
    for(int i=1;i<num_interc;i++){
      double  dist_temp=bg::distance(bg_point(first_point.x,first_point.y),row_line.intercect_points_.at(i));
      if(dist_temp>dist_max){
        dist_max=dist_temp;
        index_max=i;
      }
    }
    advance_vector.x=row_line.intercect_points_.at(index_max).x()-first_point.x;
    advance_vector.y=row_line.intercect_points_.at(index_max).y()-first_point.y;
    rotate_vetor(&advance_vector,&advance_vector,base,0,false);
  }
  else{
    dist_max=1;
    advance_vector=row_vector;
  }

  noph=(uint) ceil(dist_max/base)+1;
  #ifdef DEBUG
  //std::cout<<"dist_max: "<<num_interc<<"\tnoph: "<<noph<<std::endl;
  #endif
  std::vector<geometry_msgs::Vector3> temp_vec;
  geometry_msgs::Vector3 temp_point;

  flight_lines.at(0).resize(noph+2*addittional_wp);
  flight_lines.at(0).at(0).x=first_point.x-advance_vector.x*addittional_wp-side_vector.x/2;
  flight_lines.at(0).at(0).y=first_point.y-advance_vector.y*addittional_wp-side_vector.y/2;
  //Se utiliza atan2(x/y) ya que en el frame global, el angulo yaw o de heading se mide desde el eje que coincide con el norte
  //que para el frame local, donde se estan realizando los calculos coincide con el eje y
  double  heading=atan2(advance_vector.y,advance_vector.x);
  if(are_degrees){heading*=180/M_PI;}
  heading_vector->push_back(heading);
  /*flight_lines.at(0).at(0).x=first_point.x-advance_vector.x*addittional_wp;
  flight_lines.at(0).at(0).y=first_point.y-advance_vector.y*addittional_wp;*/
  #ifdef DEBUG
  //std::cout<<"dist_max: "<<num_interc<<"\tnoph: "<<noph<<std::cout;
  #endif
  for(int i=1;i<flight_lines.at(0).size();i++){
    flight_lines.at(0).at(i).x=flight_lines.at(0).at(i-1).x+advance_vector.x;
    flight_lines.at(0).at(i).y=flight_lines.at(0).at(i-1).y+advance_vector.y;
    heading_vector->push_back(heading);
  }

  flight_lines2.clear();
  temp_vec.clear();
  temp_point.x=first_point.x-advance_vector.x*addittional_wp-side_vector.x/2;
  temp_point.y=first_point.y-advance_vector.y*addittional_wp-side_vector.y/2;
  //temp_point.z=0;
  temp_point.z=flight_height;
  if(validate_point(crop_contour,bg_point(temp_point.x,temp_point.y),line_space)){
    temp_vec.push_back(temp_point);
    heading_vector2.push_back(heading);
    //std::cout<<"entro primera linea"<<std::endl;
  }
  for(int i=1;i<noph+2*addittional_wp;i++){
    temp_point.x+=advance_vector.x;
    temp_point.y+=advance_vector.y;
    if(validate_point(crop_contour,bg_point(temp_point.x,temp_point.y),line_space)){
      //std::cout<<"entro primera linea *"<<std::endl;
      temp_vec.push_back(temp_point);
      heading_vector2.push_back(heading);
    }
  }
  if(!temp_vec.empty()){
    flight_lines2.push_back(temp_vec);
  }


  //Desplazar la linea para hallar las nuevas intercepciones
  row_line.move_line(side_vector.x/2,side_vector.y/2);
  /**********************************************/
  /******Generación puntos lineas restantes******/
  /**********************************************/

  for(int i=1;i<lines_number;i++){
    if(row_line.find_y(min_vector.x-10*fabs(min_vector.x),&y1)){
      row_line.find_y(max_vector.x+10*fabs(max_vector.x),&y2);
      x1=min_vector.x-10*fabs(min_vector.x);
      x2=max_vector.x+10*fabs(max_vector.x);
    }
    else{
      y1=min_vector.y-10.0*fabs(min_vector.y);
      y2=max_vector.y+10.0*fabs(max_vector.y);
      x1=row_line.line_.at<double>(0,0);
      x2=x1;
    }
    #ifdef DEBUG
    //std::cout<<"\tx1: "<<x1<<"\ty1: "<<y1<<"\tx2: "<<x2<<"\ty2: "<<y2<<std::endl;
    #endif

    row_line.set_segment(x1,y1,x2,y2);
    bg_point  aux1;
    index_max=0;index_min=0;
    bg_point  begin_point;
    num_interc=row_line.find_interception(crop_contour.polygon_);
    #ifdef DEBUG
    //std::cout<<"num interc crop contour: "<<num_interc<<std::endl;
    #endif
    if(num_interc==0){
      uint  num_interc2=row_line.find_interception(bounding_contour.polygon_);
      if(num_interc2>1){
        //Se hayan los puntos inicial y final con el contorno de los valores minimos y maximos
        dist_max=bg::distance(bg_point(flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).x,flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).y),
                              row_line.intercect_points_.at(0));
        dist_min=dist_max;
        for(int j=1;j<row_line.intercect_points_.size();j++){
          double  dist_temp=bg::distance(bg_point(flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).x,flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).y),
                                         row_line.intercect_points_.at(j));
          if(dist_temp>dist_max){
            dist_max=dist_temp;
            index_max=j;
          }
          if(dist_temp<dist_min){
            dist_min=dist_temp;
            index_min=j;
          }
        }
        advance_vector.x=row_line.intercect_points_.at(index_max).x()-row_line.intercect_points_.at(index_min).x();
        advance_vector.y=row_line.intercect_points_.at(index_max).y()-row_line.intercect_points_.at(index_min).y();
        rotate_vetor(&advance_vector,&advance_vector,base,0,false);
        begin_point=row_line.intercect_points_.at(index_min);
        noph=(uint) ceil(bg::distance(row_line.intercect_points_.at(index_max),row_line.intercect_points_.at(index_min))/base)+1;
      }else{
        //repetir patron de linea anterior
        begin_point=bg_point(flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1-addittional_wp).x+side_vector.x,
                             flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1-addittional_wp).y+side_vector.y);
        noph=flight_lines.at(i-1).size()-2*addittional_wp;
        rotate_vetor(&advance_vector,&advance_vector,base,M_PI,false);
      }
    }else if(num_interc==1){
      bg_point  intercect=row_line.intercect_points_.at(0);
      uint  num_interc2=row_line.find_interception(bounding_contour.polygon_);
      if(num_interc2>1){
        dist_max=bg::distance(intercect,row_line.intercect_points_.at(0));
        for(int j=1;j<num_interc;j++){
          double  dist_temp=bg::distance(intercect,row_line.intercect_points_.at(i));
          if(dist_temp>dist_max){
            dist_max=dist_temp;
            index_max=j;
          }
        }
        advance_vector.x=row_line.intercect_points_.at(index_max).x()-intercect.x();
        advance_vector.y=row_line.intercect_points_.at(index_max).y()-intercect.y();
        aux1=bg_point(flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).x,
                      flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).y);
        if(bg::distance(intercect,aux1)>
           bg::distance(row_line.intercect_points_.at(index_max),aux1)){
          begin_point=intercect;
          rotate_vetor(&advance_vector,&advance_vector,base,0,false);
        }
        else{
          begin_point=row_line.intercect_points_.at(index_max);
          rotate_vetor(&advance_vector,&advance_vector,base,M_PI,false);
        }
        noph=(uint) ceil(bg::distance(row_line.intercect_points_.at(index_max),intercect)/base)+1;
      }else{
        //repetir el patron de la linea anterior
        begin_point=bg_point(flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1-addittional_wp).x+side_vector.x,
                             flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1-addittional_wp).y+side_vector.y);
        noph=flight_lines.at(i-1).size()-2*addittional_wp;
        rotate_vetor(&advance_vector,&advance_vector,base,M_PI,false);
      }
    }
    else{

      dist_max=bg::distance(bg_point(flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).x,flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).y),
                            row_line.intercect_points_.at(0));
      dist_min=dist_max;
      for(int j=1;j<row_line.intercect_points_.size();j++){
        double  dist_temp=bg::distance(bg_point(flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).x,flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).y),
                                       row_line.intercect_points_.at(j));
        if(dist_temp>dist_max){
          dist_max=dist_temp;
          index_max=j;
        }
        if(dist_temp<dist_min){
          dist_min=dist_temp;
          index_min=j;
        }
      }
      #ifdef DEBUG
      //std::cout<<"llego"<<std::endl;
      #endif
      advance_vector.x=row_line.intercect_points_.at(index_max).x()-row_line.intercect_points_.at(index_min).x();
      advance_vector.y=row_line.intercect_points_.at(index_max).y()-row_line.intercect_points_.at(index_min).y();
      rotate_vetor(&advance_vector,&advance_vector,base,0,false);
      begin_point=row_line.intercect_points_.at(index_min);
      noph=(uint) ceil(bg::distance(row_line.intercect_points_.at(index_max),row_line.intercect_points_.at(index_min))/base)+1;
    }
    //Generación de los puntos con base en el vector de avance y el numero de fotografias calculados
    flight_lines.at(i).resize(noph+2*addittional_wp);
    /*flight_lines.at(i).at(0).x=begin_point.x()-advance_vector.x*addittional_wp-side_vector.x/2;
    flight_lines.at(i).at(0).y=begin_point.y()-advance_vector.y*addittional_wp-side_vector.y/2;*/
    flight_lines.at(i).at(0).x=begin_point.x()-advance_vector.x*addittional_wp;
    flight_lines.at(i).at(0).y=begin_point.y()-advance_vector.y*addittional_wp;
    //Se utiliza atan2(x/y) ya que en el frame global, el angulo yaw o de heading se mide desde el eje que coincide con el norte
    //que para el frame local, donde se estan realizando los calculos coincide con el eje y
    heading=atan2(advance_vector.y,advance_vector.x);
    if(are_degrees){heading*=180/M_PI;}
    heading_vector->push_back(heading);
    for(int j=1;j<flight_lines.at(i).size();j++){
      flight_lines.at(i).at(j).x=flight_lines.at(i).at(j-1).x+advance_vector.x;
      flight_lines.at(i).at(j).y=flight_lines.at(i).at(j-1).y+advance_vector.y;
      heading_vector->push_back(heading);
    }

    temp_point.x=begin_point.x()-advance_vector.x*addittional_wp;
    temp_point.y=begin_point.y()-advance_vector.y*addittional_wp;
    temp_point.z=flight_height;
    temp_vec.clear();
    if(validate_point(crop_contour,bg_point(temp_point.x,temp_point.y),line_space)){
      temp_vec.push_back(temp_point);
      heading_vector2.push_back(heading);
    }
    for(int i=1;i<noph+2*addittional_wp;i++){
      temp_point.x+=advance_vector.x;
      temp_point.y+=advance_vector.y;
      if(validate_point(crop_contour,bg_point(temp_point.x,temp_point.y),line_space)){
        temp_vec.push_back(temp_point);
        heading_vector2.push_back(heading);
      }
    }
    if(!temp_vec.empty()){
      flight_lines2.push_back(temp_vec);
    }
    //Mover la linea
    row_line.move_line(side_vector.x,side_vector.y);
  }

  int counter=0;
  std::cout<<"Flight lines 2"<<std::endl;
  //#ifdef DEBUG
  std::cout <<"fligh lines size: \n"<<flight_lines2.size()<<std::endl;

  std::cout<<"x:\ty:\tangle"<<std::endl;
  //#endif
  for(int i=0 ;i<flight_lines2.size();i++){
    std::cout <<"line_"<<i<< "size: "<<flight_lines2.at(i).size()<<std::endl;
    for(int j=0;j<flight_lines2.at(i).size();j++){
      //#ifdef DEBUG
      std::cout<<flight_lines2.at(i).at(j).x<<"\t";//<<
      std::cout<<flight_lines2.at(i).at(j).y<<"\t";//<<
      std::cout<<heading_vector2.at(counter)<<std::endl;
                 //heading_vector.at(counter)*180/M_PI<<std::endl;
      //#endif
      /*geometry_msgs::Vector3  temp;
      georef_navf_point(navf_plan_waypoints.at(i).at(j),ref_gps_pose,
                         &temp,are_degrees);
      temp.z=ref_gps_pose.z+flight_height;
      res.plan_gps_coors.push_back(temp);*/
      counter++;
    }
  }

  //***********
  heading_vector->clear();
  *heading_vector=heading_vector2;
  std::cout<<"llego"<<std::endl;
  waypoints->resize(flight_lines2.size());
  for(int i=0;i<flight_lines2.size();i++){
    waypoints->push_back(flight_lines2.at(i));
  }
  //***********
  /*waypoints->resize(flight_lines.size());
  for(int i=0;i<flight_lines.size();i++){
    //waypoints->push_back(flight_lines2.at(i));
    waypoints->at(i).resize(flight_lines.at(i).size());//GPU Time : 153.615 ms
    for(int j=0;j<flight_lines.at(i).size();j++){
      flight_lines.at(i).at(j).z=0;
      waypoints->at(i).at(j)=flight_lines.at(i).at(j);
    }
  }*/

  return;
}
uint  flight_planner::find_initial_point_and_side_vector(std::vector<geometry_msgs::Vector3> navf_points,contour_class crop_contour,double line_space,
                                                         double x_min,double x_max,double y_min,double y_max,
                                                         geometry_msgs::Vector3* row_vector,geometry_msgs::Vector3* side_vector,uint* line_number){
  uint  index=0;
  line_class  aux_line;// linea auxiliar creada para realizar operaciones
  double  n_lines;
  double  num_interc;
  double  thres=1;
  double  y1, y2,x1,x2;
  double  dist=0,dist2;
  //
  bg_point  centroid;
  crop_contour.get_centroid(&centroid);
  #ifdef DEBUG
  std::cout<<"Centroid\nx1: "<<centroid.x()<<"\ty1: "<<centroid.y()<<std::endl;
  #endif
  //Se inicializa la linea auxiliar de manera que pase por el centroide
  aux_line=line_class(centroid.x(),centroid.y(),row_vector);
  //Asginar el segmento de manera que la distancia a los puntos del contorno sea como a la de una linea infinita
  if(aux_line.find_y(x_min-10*fabs(x_min),&y1)){
    aux_line.find_y(x_max+10*fabs(x_max),&y2);
    x1=x_min-10*fabs(x_min);
    x2=x_max+10*fabs(x_max);
  }
  else{
    y1=y_min-10.0*fabs(y_min);
    y2=y_max+10.0*fabs(y_max);
    x1=aux_line.line_.at<double>(0,0);
    x2=x1;
  }
  #ifdef DEBUG
  std::cout<<"\tx1: "<<x1<<"\ty1: "<<y1<<"\tx2: "<<x2<<"\ty2: "<<y2<<std::endl;
  #endif

  aux_line.set_segment(x1,y1,x2,y2);
  // Se encuentra el punto cuya distancia al segmento que pasa por el centroide sea la mayor
  // Se guarda el indice de dicho punto, que sera el punto inicial
  dist=bg::distance(aux_line.segment_,bg_point(navf_points.at(0).x,
                                               navf_points.at(0).y));
  for(int i=1;i<navf_points.size();i++){
    dist2=bg::distance(aux_line.segment_,bg_point(navf_points.at(i).x,
                                                  navf_points.at(i).y));
    if(dist2>dist){
      dist=dist2;
      index=i;
    }
  }
  //con base en el punto identificado se determina cual es la dirección de avance lateral
  rotate_vetor(row_vector,side_vector,line_space,CV_PI/2,false);
  bg_point  p1(navf_points.at(index).x+side_vector->x,
               navf_points.at(index).y+side_vector->y);
  //Si el punto esta dentro del poligono se obtuvo la dirección de avance correcta
  //De no ser asi, la dirección se recalcula hacia el aldo opuesto
  if(!bg::within(p1,crop_contour.polygon_)){
    rotate_vetor(side_vector,side_vector,line_space,CV_PI,false);
  }
  #ifdef DEBUG
  //std::cout<<"side vector"<<*side_vector<<std::endl;
  #endif
  //Con la dirección de avance lateral se pued determinar el numero de lineas para
  //lo cual desde le punto inicial se desplaza la linea cuyo vector director va en
  //la dirección de los surcos (row_vector) y se comprueba cuantas veces esta linea
  //se intercepta con el contorno del cultivo
  aux_line=line_class(navf_points.at(index).x,navf_points.at(index).y,row_vector);
  #ifdef DEBUG
  //std::cout<<"row vector"<<*row_vector<<std::endl;
  #endif
  bool  conti=true;
  n_lines=1;
  aux_line.move_line(side_vector->x/2,side_vector->y/2);
  while (conti) {
    if(aux_line.find_y(x_min-10*fabs(x_min),&y1)){
      aux_line.find_y(x_max+10*fabs(x_max),&y2);
      x1=x_min-10*fabs(x_min);
      x2=x_max+10*fabs(x_max);
    }
    else{
      y1=y_min-10.0*fabs(y_min);
      y2=y_max+10.0*fabs(y_max);
      x1=aux_line.line_.at<double>(0,0);
      x2=x1;
    }
    #ifdef DEBUG
    //std::cout<<"\tx1: "<<x1<<"\ty1: "<<y1<<"\tx2: "<<x2<<"\ty2: "<<y2<<std::endl;
    #endif

    aux_line.set_segment(x1,y1,x2,y2);
    conti=bg::intersects(aux_line.segment_,crop_contour.polygon_);
    //Mover linea
    #ifdef DEBUG
    //std::cout<<"\t Interc "<< std::boolalpha<<conti;
    #endif
    if(conti){n_lines++;}
    aux_line.move_line(side_vector->x,side_vector->y);
  }
  #ifdef DEBUG
  //std::cout<<"\n";
  #endif

  // Se utiliza una linea adicional
  n_lines++;
  *line_number=n_lines;
  return  index;
}

void flight_planner::find_max_min_x_y(std::vector<geometry_msgs::Vector3>  points, cv::Point2f *max_vector,
                      cv::Point2f *min_vector){
  double  max_x,min_x,max_y,min_y;
  uint  num_points=points.size();
  min_x=points.at(0).x;
  max_x=points.at(0).x;
  min_y=points.at(0).y;
  max_y=points.at(0).y;
  for(int i=1;i<num_points;i++){
    if(points.at(i).x<min_x){
      min_x=points.at(i).x;
    }
    if(points.at(i).x>max_x){
      max_x=points.at(i).x;
    }
    if(points.at(i).y<min_y){
      min_y=points.at(i).y;
    }
    if(points.at(i).y>max_y){
      max_y=points.at(i).y;
    }
  }
  min_vector->x=min_x*1.01;
  min_vector->y=min_y*1.01;
  max_vector->x=max_x*1.01;
  max_vector->y=max_y*1.01;

  return;
}


void  flight_planner::rotate_vetor(geometry_msgs::Vector3 *orig , geometry_msgs::Vector3 *rotated, double magnitude, double  rotation, bool are_degrees){
  double theta;
  if(are_degrees){
    rotation*=CV_PI/180.0;
  }
  #ifdef DEBUG
  //std::cout<<"orig "<<*orig<<std::endl;
  #endif
  theta=atan2(orig->y,orig->x);
  #ifdef DEBUG
  //std::cout<<"ang "<<theta*180/CV_PI<<std::endl;
  #endif
  if(theta<0){
    theta+=2*CV_PI;
  }
  theta+=rotation;
  polar2cart(rotated,magnitude,theta,false);
  #ifdef DEBUG
  //std::cout<<"rotated "<<*rotated<<std::endl;
  #endif
  return;
}

void  flight_planner::normalize_2d(geometry_msgs::Vector3 *vector){
  double  mag=sqrt(pow(vector->x,2)+pow(vector->y,2));
  vector->x/=mag;
  vector->y/=mag;
  return;
}
void  flight_planner::polar2cart(geometry_msgs::Vector3 *vector,double r,double theta,bool is_deg){
  if(is_deg){
    theta*=M_PI/180.0;
  }
  //*vector->at<double>(0,0)=r*cos(theta);
  vector->x=r*cos(theta);//x=r*cos(theta)
  vector->y=r*sin(theta);//y=r*sin(theta)
  return;
}


double flight_planner::calculate_height(double fl,double gsd,double sw,double iw){
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

double  flight_planner::calculate_gsd(double fl,double fh,double sw,double iw){
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

/// caclulates cartesian distance between two points p1, p2
double  flight_planner::calculate_distance(double x1,double y1,double x2, double  y2){
  double  dist=sqrt(pow(x2-x1,2)+pow(y2-y1,2));
  return  dist;
}

///converts imu coord(sensor_msgs/Vector3) to cv_Mat column vector;
void  flight_planner::vector3_2cvMat(geometry_msgs::Vector3 vec3,cv::Mat* cv_mat){
  cv_mat->row(0).col(0)=vec3.x;
  cv_mat->row(1).col(0)=vec3.y;
  cv_mat->row(2).col(0)=vec3.z;
  return;
}

void  flight_planner::cvMat2vector3(geometry_msgs::Vector3 *gps, cv::Mat gps_mat){
  gps->x=gps_mat.at<double>(0,0);
  gps->y=gps_mat.at<double>(1,0);
  gps->z=gps_mat.at<double>(2,0);
  return;
}



///updates intrinsic parameters matrix
void flight_planner::update_k(cv::Mat *k,double fx,double fy,double cx,double cy,double skew){
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

///updates rotation matrix from camera frame to body frame(uav)
/// cam ori is the angle between flight direction and camera width
void flight_planner::update_CF2BF(double cam_ori,cv::Mat* CF2BF,bool is_degrees){
    if(is_degrees){
        cam_ori*=M_PI/180.0;
    }
    #ifdef DEBUG
    //std::cout<<"UpdatingCF2BF\ncam ori:"<< cam_ori<<std::endl;
    #endif
    cv::Mat Rz=cv::Mat::zeros(3,3,CV_64F);
    Rz.row(2).col(2)=1;
    Rz.row(0).col(0)=cos(cam_ori);
    #ifdef DEBUG
    //std::cout<<"cos(cam_ori):"<< cos(cam_ori)<<std::endl;
    #endif
    Rz.row(0).col(1)=-sin(cam_ori);
    Rz.row(1).col(0)=sin(cam_ori);
    Rz.row(1).col(1)=cos(cam_ori);
    #ifdef DEBUG
    //std::cout<<"rz:"<< Rz<<std::endl;
    #endif
    *CF2BF=Rz;
    #ifdef DEBUG
    //std::cout<<"cf2bf:"<< CF2BF<<std::endl;
    #endif
    return;
}
///updates rotation matrix from body frame to navigation frame
void flight_planner::update_BF2NF(geometry_msgs::Vector3 IMU_ori, cv::Mat* BF2NF, bool is_degrees){
    /***********
     * yaw: orientation.x
     * pitch: orientation.y
     * roll: orientation.z
    ***********/
    double y,p,r;
    y=IMU_ori.x;
    p=IMU_ori.y;
    r=IMU_ori.z;
    if(is_degrees){
        y*=M_PI/180.0;
        p*=M_PI/180.0;
        r*=M_PI/180.0;
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
///converts geodetic coors(lat,lon,alt) to navigation frame
void flight_planner::geod2nf(geometry_msgs::Vector3 uav_gps_posi, geometry_msgs::Vector3 ref_gps_posi,
                        geometry_msgs::Vector3 *navf_point, bool is_degrees){
    //navf_=gl::LocalCartesian(ref_gps_posi.x,ref_gps_posi.y,ref_gps_posi.z,ecef);
    double  factor=1;
    if(!is_degrees){
      factor=180.0/M_PI;
    }
    navf_.Reset(ref_gps_posi.x*factor,ref_gps_posi.y*factor,ref_gps_posi.z);
    navf_.Forward(uav_gps_posi.x*factor,uav_gps_posi.y*factor,uav_gps_posi.z,
                  navf_point->x,navf_point->y,navf_point->z);
    enu2ned(navf_point);
    return;
}
void  flight_planner::enu2ned(geometry_msgs::Vector3* coord){
  /*transformation matriz for enu to ned standard and vicebersa
   *|0  1   0|
   *|1  0   0|
   *|0  0  -1|
   * */
   cv::Mat enu2ned_=cv::Mat::zeros(3,3,CV_64F);
   enu2ned_.row(0).col(1)=1;
   enu2ned_.row(1).col(0)=1;
   enu2ned_.row(2).col(2)=-1;
   cv::Mat coord_copy=cv::Mat(3,1,CV_64F);
   coord_copy.row(0).col(0)=coord->x;
   coord_copy.row(1).col(0)=coord->y;
   coord_copy.row(2).col(0)=coord->z;
   cv::Mat result=cv::Mat(3,1,CV_64F);
   result=enu2ned_*coord_copy;
   #ifdef DEBUG
   //std::cout<<"result"<<result<<std::endl;
   #endif
   coord->x=result.at<double>(0,0);
   coord->y=result.at<double>(1,0);
   coord->z=result.at<double>(2,0);
   return;
}

void flight_planner::if2navf(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                        cv::Mat k, geometry_msgs::Vector3 image_coord, geometry_msgs::Vector3 ref_gps_posi,
                        geometry_msgs::Vector3 *navf_point, bool is_degrees, double scale, double fl){
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
    cv::Mat uav_navf_posi=cv::Mat(3,1,CV_64F);
    cv::Mat cam_coord=cv::Mat(3,1,CV_64F);
    cv::Mat geor_navf_coord=cv::Mat(3,1,CV_64F);
    #ifdef DEBUG
    //std::cout<<"k\n"<<k<<std::endl;
    //std::cout<<"gps_ref:\n"<<ref_gps_posi<<std::endl;
    #endif
    //Trasladar coordenadas de imagen a camara

    cam_coord.row(0).col(0)=image_coord.x-k.at<double>(0,2);
    cam_coord.row(1).col(0)=-(image_coord.y-k.at<double>(1,2));
    cam_coord.row(2).col(0)=image_coord.z;
    //se usa image_coord temporalmente para almacenar la coordenada del dron en en navf
    geod2nf(uav_gps_posi,ref_gps_posi,&image_coord,is_degrees);
    vector3_2cvMat(image_coord,&uav_navf_posi);
    #ifdef DEBUG
    /*std::cout<<"uav_gps: \n"<<uav_gps_posi<<std::endl;
    std::cout<<"uav_navf: \n"<<image_coord<<std::endl;
    std::cout<<"uav_ori: \n"<<imu_ori<<std::endl;
    std::cout<<"cam coor: \n"<<cam_coord<<std::endl;*/
    #endif

    //Pasar del frame de la camara al frame del body (UAV)
    update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);
    //Pasar del frame del body al frame de navegación
    update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
    //tranformación completa del frame de la camara al frame de navegación
    CF2NF_tf=BF2NF_tf*CF2BF_tf;

    geor_navf_coord=scale*CF2NF_tf*cam_coord+uav_navf_posi;
    #ifdef DEBUG
    //std::cout<<"coord navf:\n"<< geor_navf_coord<<std::endl;
    #endif
    navf_point->x=geor_navf_coord.at<double>(0,0);
    navf_point->y=geor_navf_coord.at<double>(1,0);
    navf_point->z=geor_navf_coord.at<double>(2,0);
    return;
}

void flight_planner::navf2if(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                             cv::Mat k,cv::Mat navf_coord, geometry_msgs::Vector3 ref_gps_posi,
                             geometry_msgs::Vector3* image_coord, bool is_degrees, double scale, double fl){
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
    cv::Mat uav_navf_posi=cv::Mat(3,1,CV_64F);
    //se usa image_coord temporalmente para almacenar la coordenada del dron en en navf
    geod2nf(uav_gps_posi,ref_gps_posi,image_coord,is_degrees);
    vector3_2cvMat(*image_coord,&uav_navf_posi);
    //Pasar del frame de la camara al frame del body (UAV)
    update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);
    //Pasar del frame del body al frame de navegación
    update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
    //tranformación completa del frame de la camara al frame de navegación
    CF2NF_tf=BF2NF_tf*CF2BF_tf;
    cv::invert(CF2NF_tf,NF2CF_tf);
    cam_coord=NF2CF_tf*((navf_coord-uav_navf_posi)/scale);
    image_coord->x=cam_coord.at<double>(0,0)+k.at<double>(0,2);
    image_coord->y=-cam_coord.at<double>(1,0)+k.at<double>(1,2);
    image_coord->z=cam_coord.at<double>(2,0);
    return;
}

void flight_planner::georef_navf_point(geometry_msgs::Vector3 navf_coord, geometry_msgs::Vector3 ref_gps_posi,
                        geometry_msgs::Vector3 *gps_coord, bool is_degrees){
  double  factor=1;
  if(!is_degrees){
    factor=180.0/M_PI;
  }
  navf_.Reset(ref_gps_posi.x*factor,ref_gps_posi.y*factor,ref_gps_posi.z);
  #ifdef DEBUG
  //std::cout<<std::setprecision(10)<<"navf bef enu2ned\n"<<navf_coord<<std::endl;
  #endif
  navf_.Reverse(navf_coord.x,navf_coord.y,navf_coord.z,gps_coord->x,gps_coord->y,gps_coord->z);
  #ifdef DEBUG
  //std::cout<<std::setprecision(10)<<"goed bef enu2ned\n"<<*gps_coord<<std::endl;
  #endif

  enu2ned(&navf_coord);
  #ifdef DEBUG
  //std::cout<<std::setprecision(10)<<"navf after enu2ned\n"<<navf_coord<<std::endl;
  #endif
  navf_.Reverse(navf_coord.x,navf_coord.y,-navf_coord.z,gps_coord->x,gps_coord->y,gps_coord->z);
  #ifdef DEBUG
  //std::cout<<std::setprecision(10)<<"goed after enu2ned\n"<<*gps_coord<<std::endl;
  #endif
  gps_coord->x/=factor;
  gps_coord->y/=factor;
  //gps_coord->z=factor;
  return;
}

///given the image coordinates, uav pose and camera info, returns the geodetic coordinates
void flight_planner::georef_image_point(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                        cv::Mat k, geometry_msgs::Vector3 image_coord, geometry_msgs::Vector3 ref_gps_posi,
                        geometry_msgs::Vector3 *georeferenced_point, bool is_degrees, double scale, double fl){
  geometry_msgs::Vector3  navf_coord;
  if2navf(cam_ori,imu_ori,uav_gps_posi,k,image_coord,ref_gps_posi,&navf_coord,
          is_degrees,scale,fl);
  georef_navf_point(navf_coord,ref_gps_posi,georeferenced_point,is_degrees);
    return;
}

bool flight_planner::validate_point(contour_class contour,bg_point point,double dist_threshold){
  /*bool success;
  success=(bg::distance(point,contour.polygon_)<dist_threshold)?true:false;
  return success;*/
  if(fabs(bg::distance(point,contour.polygon_))<=dist_threshold*1.1){
    return true;
  }else{
    return false;
  }
}

void flight_planner::main_task(){
  #ifdef DEBUG
  ROS_INFO_ONCE("Ready to plan flight");
  #endif
  while (ros::ok()) {
    ros::spinOnce();
    rate_.sleep();
  }
}

int main(int argc, char **argv)
{

  std::setprecision (10);
  ros::init(argc, argv, "flight_planner_server");
  flight_planner fp;
  fp.main_task();
  //ros::spin();

  return 0;
}

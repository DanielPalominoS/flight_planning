#include "flight_planner.h"
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
  intercept_point.x=0;
  intercept_point.y=0;
  intercpt_found=false;
}

line_class::line_class(double  x1,double y1,cv::Mat* vector){
  //ecuación parametrica recta columna 0 componente x; columna 1 componente y
  line_=cv::Mat(2,2,CV_64F);
  line_.row(0).col(0)=x1;
  line_.row(0).col(1)=y1;
  line_.row(1).col(0)=vector->at<double>(0,0);
  line_.row(1).col(1)=vector->at<double>(1,0);
  intercept_point.x=0;
  intercept_point.y=0;
}
line_class::line_class(double  x1,double y1,geometry_msgs::Vector3* vector){
  //ecuación parametrica recta columna 0 componente x; columna 1 componente y
  line_=cv::Mat(2,2,CV_64F);
  line_.row(0).col(0)=x1;
  line_.row(0).col(1)=y1;
  line_.row(1).col(0)=vector->x;
  line_.row(1).col(1)=vector->y;
  intercept_point.x=0;
  intercept_point.y=0;
}

/// finds the interception between the line to intercept and the line
bool line_class::find_interception(cv::Mat* line_to_int){
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



void  line_class::move_line(double x,double y){
  line_.row(0).col(0)+=x;
  line_.row(0).col(1)+=y;
  return;
}

flight_planner::flight_planner()
  :i_width(1280)
{
  ROS_INFO("ready to geref points or plan flight");
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
  //navf=gl::LocalCartesian(ref_gps_pose.x,ref_gps_pose.y,
  //                        ref_gps_pose.z,ecef);
  navf_=gl::LocalCartesian(ecef);
  planner=nh.advertiseService("/flight_planning/plan_flight", &flight_planner::plannerCB,this);
  georeferencer=nh.advertiseService("/flight_planning/georef_points", &flight_planner::georeferencerCB,this);
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
                       &res.gps_coors[i],are_degrees,scale_number,fl,true);
  }
  if(true){std::cout<<res<<std::endl;}
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

  line_class  row_line;
  std::vector<line_class> contour_lines;

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
    crop_pixel_coord.at(i).x=req.crop.contour.at(i).x;
    crop_pixel_coord.at(i).y=req.crop.contour.at(i).y;
    crop_pixel_coord.at(i).z=fl_pixel;
  }
  res.crop_gps_coors.resize(numCropPoints);
  std::vector<geometry_msgs::Vector3>  crop_navf_coors;
  crop_navf_coors.resize(numCropPoints);
  if (debug_){std::cout<<"\n++++GEOREFERENCIACIÓN VERTICES+++"<<std::endl;}
  for(int i=0;i<numCropPoints;i++){
    georef_image_point(cam_ori,imu_uav,uav_gps_pose,k,crop_pixel_coord.at(i),ref_gps_pose,
                       &res.crop_gps_coors[i],are_degrees,scale_number,fl,false);
    if2navf(cam_ori,imu_uav,uav_gps_pose,k,crop_pixel_coord.at(i),ref_gps_pose,
            &crop_navf_coors.at(i),are_degrees,scale_number,fl,false);
  }

  if (debug_){std::cout<<"\n+++TRANSFORMACIÓN VECTOR ORIENTACIÓN DEL IF AL NAVF+++"<<std::endl;}

  /***transformar el vector de orientación del cultivo***/
  std::cout<<req.crop.row_orientation<<std::endl;
  polar2cart(&crop_row_vector_image,1.0,req.crop.row_orientation+90,are_degrees);
  crop_row_vector_image.x+=k.at<double>(0,2);//se le agrega esta cantidad para que el vector quede en el origen de la camara
  crop_row_vector_image.y+=k.at<double>(1,2);
  crop_row_vector_image.z=fl_pixel;

  tempVec=ref_gps_pose;
  tempVec.z=uav_gps_pose.z;
  if (debug_){std::cout<<"row vector image: \n"<<crop_row_vector_image<<std::endl;}

  if2navf(cam_ori,imu_uav,tempVec,k,crop_row_vector_image,ref_gps_pose,
          &crop_row_vector_navf,are_degrees,scale_number,fl,true);
  crop_row_vector_navf.z=0;
  normalize_2d(&crop_row_vector_navf);
  if (true){std::cout<<"row vector navf unit: \n"<<crop_row_vector_navf<<std::endl;}
  crop_row_vector_navf.x*=base;
  crop_row_vector_navf.y*=base;
  if (true){std::cout<<"row vector navf: \n"<<crop_row_vector_navf<<std::endl;}

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
      contour_lines.at(i)=line_class(crop_navf_coors.at(i).x,crop_navf_coors.at(i).y,
                                     crop_navf_coors.at(numCropPoints-1).x,crop_navf_coors.at(numCropPoints-1).y);
    }
    else{
      contour_lines.at(i)=line_class(crop_navf_coors.at(i).x,crop_navf_coors.at(i).y,
                                     crop_navf_coors.at(i-1).x,crop_navf_coors.at(i-1).y);
    }
    std::cout<<"line "<<i<<": "<<contour_lines.at(i).line_<<std::endl;
  }
  if (debug_){std::cout<<"\n+++DETERMINACIÓN PUNTO INICIAL+++"<<std::endl;}
  uint  initial_point_index;
  initial_point_index=find_initial_point_and_side_vector(&crop_navf_coors,&contour_lines,&crop_row_vector_navf,max_vector.at<double>(0,0),
                                                         max_vector.at<double>(1,0),min_vector.at<double>(0,0),min_vector.at<double>(1,0),
                                                         &side_vector,line_space,&lines_number,true);

  if (debug_){std::cout<<"inicial point"<<crop_navf_coors.at(initial_point_index)<<"\nlines number: "<<lines_number<<std::endl;}
  if (debug_){std::cout<<"\n+++GENERACIÓN PUNTOS+++"<<std::endl;}
  generate_points(&crop_navf_coors,&contour_lines,&crop_row_vector_navf,max_vector.at<double>(0,0),
                  max_vector.at<double>(1,0),min_vector.at<double>(0,0),min_vector.at<double>(1,0),
                  &side_vector,base,line_space,lines_number,initial_point_index, true,&navf_plan_waypoints);
  if(true){
    std::cout<<"x:\ty:"<<std::endl;
    for(int i=0;i<navf_plan_waypoints.size();i++){
      for(int j=0;j<navf_plan_waypoints.at(i).size();j++){
        std::cout<<navf_plan_waypoints.at(i).at(j).x<<"\t"<<navf_plan_waypoints.at(i).at(j).y<<std::endl;
      }
    }
  }
  /****Georeferenciación puntos generados*********/
  int counter=0;
  for(int i=0 ;i<navf_plan_waypoints.size();i++){
    for(int j=0;j<navf_plan_waypoints.at(i).size();j++){
      geometry_msgs::Vector3  temp;
      georef_navf_point(navf_plan_waypoints.at(i).at(j),ref_gps_pose,
                         &temp,are_degrees,false);
      temp.z=ref_gps_pose.z+flight_height;
      res.plan_gps_coors.push_back(temp);
      counter++;
    }
  }

  std::cout<<"\n+++FIN+++"<<std::endl;
  if(true){std::cout<<res<<std::endl;}
  res.success=true;
  return  true;
}

void  flight_planner::generate_points(std::vector<geometry_msgs::Vector3>* navf_points,std::vector<line_class>* lines,
                      geometry_msgs::Vector3* row_vector,double x_max,double y_max,double x_min,double y_min,
                      geometry_msgs::Vector3* side_vector,double base, double line_space,double lines_number,
                      uint first_point_index, bool debug,std::vector<std::vector<geometry_msgs::Vector3> >* waypoints){
  uint  num_points=navf_points->size();//numero de vertices identificados en el contorno
  // Aca empezo todo
  std::vector<std::vector<geometry_msgs::Vector3> >  flight_lines;//conjunto de lineas con los puntos del vuelo
  std::vector<uint>  interc_index;//vector que contendra los indices de las rectas del controno cuyas intercepciones con la recta del surco esten en la región limitada
  line_class  row_line;//recta cuyo vector director es la dirección del surco
  uint num_interc;//numero de intersecciones validas
  double  dist;//distancia entre dos puntos p1,p2
  double  noph;//number of photografies
  uint  addittional_wp=2;//Numero de waypoints adicionales
  geometry_msgs::Vector3 advance_vector;//=cv::Mat(3,1,CV_64F);//Vector de avance sibre las lineas
  uint  line_begin_index;//Encontrar un mejor nombre
  uint  line_end_index;
  //inicialización recta
  row_line=line_class(navf_points->at(first_point_index).x,navf_points->at(first_point_index).y,row_vector );

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
                                     navf_points->at(first_point_index).x,navf_points->at(first_point_index).y);
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
  std::cout<<"Line[begin_index]: "<<lines->at(line_begin_index).intercept_point<<std::endl;
  std::cout<<"NavfPoint[first_point_index]: "<<navf_points->at(first_point_index)<<std::endl;
  if(dist>base){
    //Calcular numero de fotografías
    //Nota: ceil aproxima el siguiente entero hacia arriba
    noph=(uint)(ceil(dist/base)+1);//se calcula el numero de fotografias para la linea inicial. Se agrega una linea adicional como recomendación para garantizar el cubirmiento del terreno
    advance_vector.x=lines->at(line_begin_index).intercept_point.x-navf_points->at(first_point_index).x;
    advance_vector.y=lines->at(line_begin_index).intercept_point.y-navf_points->at(first_point_index).y;
    advance_vector.z=0;
  }
  else{
    advance_vector=*row_vector;
    noph=2;
  }
  std::cout<<"\nAdvance vector no norm\nx: "<<advance_vector.x<<"\ty: "<<advance_vector.y<<std::endl;
  //Se normaliza el vector de avance y se multiplica por la longitud de la base
  normalize_2d(&advance_vector);
  advance_vector.x*=base;
  advance_vector.y*=base;
  std::cout<<"\nAdvance vector\nx: "<<advance_vector.x<<"\ty: "<<advance_vector.y<<std::endl;
  std::cout<<"side vector\nx: "<<side_vector->x<<"\ty: "<<side_vector->y<<std::endl<<std::endl;
  //Se define la cantidad de puntos en la linea con el numero de fotografias necesarias, mas una cantidad adicional
  flight_lines.at(0).resize(noph+addittional_wp);//se recomienda colocar waypoints adicionales para que el uav se alinie adecuadamente cuando cambia de linea
  // A partir del vertice se retrocede la distancia de la mitad de los puntos adicionales, en la dirección opuesta al vector de avance, para luego generar los demas
  //puntos llendo en la dirección de avance
  double  x_temp,y_temp;
  //flight_lines[0][0]=cv::Mat(3,1,CV_64F);
  flight_lines[0][0].x=navf_points->at(first_point_index).x-addittional_wp/2*advance_vector.x;
  flight_lines[0][0].y=navf_points->at(first_point_index).y-addittional_wp/2*advance_vector.y;
  flight_lines[0][0].z=0;

  //std::cout<<"x: \t\ty: "<<std::endl;
  //std::cout<<flight_lines.at(0).at(0).at<double>(0,0)<<"\t"<<flight_lines.at(0).at(0).at<double>(1,0)<<std::endl;
  for(int i=1;i<flight_lines.at(0).size();i++){
    flight_lines[0][i].x=flight_lines[0][i-1].x+advance_vector.x;
    flight_lines[0][i].y=flight_lines[0][i-1].y+advance_vector.y;
    flight_lines[0][i].z=0;
    //std::cout<<flight_lines[0][i].at<double>(0,0)<<"\t"<<flight_lines[0][i].at<double>(1,0)<<std::endl;
  }
  //Desplazar la linea para hallar las nuevas intercepciones
  row_line.move_line(side_vector->x,side_vector->y);
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
                                       flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).x,
                                       flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).y);
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
    }
    else if(num_interc==1){
      noph=2;
    }
    else{
      dist=-1;
      for(int j=0;j<num_points;j++){
        if(lines->at(j).intercpt_found){
            //calculo de la distancia de la intercepción al ultimo punto de la linea anterior
            //modificar punto
            dist_temp=calculate_distance(lines->at(j).intercept_point.x,lines->at(j).intercept_point.y,
                                         flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).x,
                                         flight_lines.at(i-1).at(flight_lines.at(i-1).size()-1).y);
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
        advance_vector=*row_vector;
        noph=2;
      }
    }
    flight_lines.at(i).resize(noph+addittional_wp);
    // A partir del vertice se retrocede la distancia de la mitad de los puntos adicionales, en la dirección opuesta al vector de avance, para luego generar los demas
        //puntos llendo en la dirección de avance
    flight_lines.at(i).at(0).x=lines->at(line_begin_index).intercept_point.x+(pow(-1,i+1))*addittional_wp/2*advance_vector.x;
    flight_lines.at(i).at(0).y=lines->at(line_begin_index).intercept_point.y+(pow(-1,i+1))*addittional_wp/2*advance_vector.y;
    flight_lines.at(i).at(0).z=0;
    //std::cout<<flight_lines.at(i).at(0).at<double>(0,0)<<"\t"<<flight_lines.at(i).at(0).at<double>(1,0)<<std::endl;
    for(int j=1;j<flight_lines.at(i).size();j++){
      flight_lines[i][j].x=flight_lines.at(i).at(j-1).x+(pow(-1,i))*advance_vector.x;
      flight_lines[i][j].y=flight_lines.at(i).at(j-1).y+(pow(-1,i))*advance_vector.y;
      flight_lines[i][j].z=0;
    }

    //desplazar linea
    row_line.move_line(side_vector->x,side_vector->y);
  }
  waypoints->resize(flight_lines.size());
  for(int i=0;i<flight_lines.size();i++){
    waypoints->at(i).resize(flight_lines.at(i).size());
    for(int j=0;j<flight_lines.at(i).size();j++){
      waypoints->at(i).at(j)=flight_lines.at(i).at(j);
    }
  }
  return;
}

void flight_planner::find_max_min_x_y(std::vector<geometry_msgs::Vector3>  points,cv::Mat* max_vector,
                      cv::Mat* min_vector){
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
  min_vector->row(0).col(0)=min_x;
  min_vector->row(1).col(0)=min_y;
  max_vector->row(0).col(0)=max_x;
  max_vector->row(1).col(0)=max_y;

  return;
}

uint  flight_planner::find_initial_point_and_side_vector(std::vector<geometry_msgs::Vector3>* navf_points,std::vector<line_class>* lines,
                                         geometry_msgs::Vector3* row_vector,double x_max,double y_max,double x_min,double y_min,
                                         geometry_msgs::Vector3* side_vector, double line_space,uint* line_number, bool debug){
  uint  index=0;
  uint  num_points=navf_points->size();
  line_class  row_line;
  std::vector<uint> num_lines;
  std::vector<geometry_msgs::Vector3>  direction;
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
    num_lines.at(i)=0;
    row_line=line_class(navf_points->at(i).x,navf_points->at(i).y,row_vector);
    rotate_vetor(row_vector,side_vector,line_space/5,CV_PI/2,false);
    direction.at(i)=*side_vector;
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
      row_line.move_line(side_vector->x,side_vector->y);

    }while (num_interc>=2);
    //std::cout<<"n lines(90) "<<n_lines<<std::endl;
    direction.at(i)=*side_vector;
    num_lines.at(i)=n_lines;
    n_lines=0;
    row_line=line_class(navf_points->at(i).x,navf_points->at(i).y,row_vector);
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
      row_line.move_line(side_vector->x,side_vector->y);
    }while (num_interc>=2);
    //std::cout<<"n lines(-90) "<<n_lines<<std::endl;
    if(num_lines.at(i)<n_lines){
      direction.at(i)=*side_vector;
      num_lines.at(i)=n_lines;
    }
    //std::cout<<"side vec"<<direction.at(i)<<std::endl;
    //std::cout<<"\nn l: "<<num_lines.at(i)<<std::endl;

  }
  index=0;

  *side_vector=direction.at(index);
  for(int i=1;i<num_points;i++){
    if(num_lines.at(i)>num_lines.at(index)){
      index=i;
      direction.at(i)=*side_vector;
    }
  }
  rotate_vetor(side_vector,side_vector,line_space,0,false);
  if(true){std::cout<<"Index: "<<index<<"\tnum lines:"<<num_lines.at(index)<<"\tside vector: \n"<<*side_vector<<std::endl;}
  *line_number=(uint)(ceil(num_lines.at(index)/5.0)+1.0);

  return  index;
}
void  flight_planner::rotate_vetor(geometry_msgs::Vector3 *orig , geometry_msgs::Vector3 *rotated, double magnitude, double  rotation, bool are_degrees){
  double theta;
  if(are_degrees){
    rotation*=CV_PI/180.0;
  }
  //std::cout<<"orig "<<*orig<<std::endl;
  theta=atan2(orig->y,orig->x);
  //std::cout<<"ang "<<theta*180/CV_PI<<std::endl;
  if(theta<0){
    theta+=2*CV_PI;
  }
  theta+=rotation;
  polar2cart(rotated,magnitude,theta,false);
  //std::cout<<"rotated "<<*rotated<<std::endl;
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
    //if (debug){};
    return;
}

///updates rotation matrix from camera frame to body frame(uav)
/// cam ori is the angle between flight direction and camera width
void flight_planner::update_CF2BF(double cam_ori,cv::Mat* CF2BF,bool is_degrees){
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
                        geometry_msgs::Vector3 *navf_point, bool is_degrees, bool debug){
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
   std::cout<<"result"<<result<<std::endl;
   coord->x=result.at<double>(0,0);
   coord->y=result.at<double>(1,0);
   coord->z=result.at<double>(2,0);
   return;
}

void flight_planner::if2navf(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                        cv::Mat k, geometry_msgs::Vector3 image_coord, geometry_msgs::Vector3 ref_gps_posi,
                        geometry_msgs::Vector3 *navf_point, bool is_degrees, double scale, double fl, bool debug){
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
    //std::cout<<"k\n"<<k<<std::endl;
    //std::cout<<"gps_ref:\n"<<ref_gps_posi<<std::endl;
    //Trasladar coordenadas de imagen a camara

    cam_coord.row(0).col(0)=image_coord.x-k.at<double>(0,2);
    cam_coord.row(1).col(0)=-(image_coord.y-k.at<double>(1,2));
    cam_coord.row(2).col(0)=image_coord.z;
    //se usa image_coord temporalmente para almacenar la coordenada del dron en en navf
    geod2nf(uav_gps_posi,ref_gps_posi,&image_coord,is_degrees,debug);
    vector3_2cvMat(image_coord,&uav_navf_posi);
    if (debug){std::cout<<"uav_gps: \n"<<uav_gps_posi<<std::endl;}
    if (debug){std::cout<<"uav_navf: \n"<<image_coord<<std::endl;}
    if (debug){std::cout<<"uav_ori: \n"<<imu_ori<<std::endl;}
    if (debug){std::cout<<"cam coor: \n"<<cam_coord<<std::endl;}

    //Pasar del frame de la camara al frame del body (UAV)
    update_CF2BF(cam_ori,&CF2BF_tf,is_degrees);
    //Pasar del frame del body al frame de navegación
    update_BF2NF(imu_ori,&BF2NF_tf,is_degrees);
    //tranformación completa del frame de la camara al frame de navegación
    CF2NF_tf=BF2NF_tf*CF2BF_tf;

    geor_navf_coord=scale*CF2NF_tf*cam_coord+uav_navf_posi;
    if (debug){std::cout<<"coord navf:\n"<< geor_navf_coord<<std::endl;}
    navf_point->x=geor_navf_coord.at<double>(0,0);
    navf_point->y=geor_navf_coord.at<double>(1,0);
    navf_point->z=geor_navf_coord.at<double>(2,0);
    return;
}

void flight_planner::navf2if(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                             cv::Mat k,cv::Mat navf_coord, geometry_msgs::Vector3 ref_gps_posi,
                             geometry_msgs::Vector3* image_coord, bool is_degrees, double scale, double fl, bool debug){
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
    geod2nf(uav_gps_posi,ref_gps_posi,image_coord,is_degrees,debug);
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
                        geometry_msgs::Vector3 *gps_coord, bool is_degrees, bool debug){
  double  factor=1;
  if(!is_degrees){
    factor=180.0/M_PI;
  }
  navf_.Reset(ref_gps_posi.x*factor,ref_gps_posi.y*factor,ref_gps_posi.z);
  if(true){std::cout<<std::setprecision(10)<<"navf bef enu2ned\n"<<navf_coord<<std::endl;}
  navf_.Reverse(navf_coord.x,navf_coord.y,navf_coord.z,gps_coord->x,gps_coord->y,gps_coord->z);
  if(true){std::cout<<std::setprecision(10)<<"goed bef enu2ned\n"<<*gps_coord<<std::endl;}

  enu2ned(&navf_coord);
  if(true){std::cout<<std::setprecision(10)<<"navf after enu2ned\n"<<navf_coord<<std::endl;}
  navf_.Reverse(navf_coord.x,navf_coord.y,navf_coord.z,gps_coord->x,gps_coord->y,gps_coord->z);
  if(true){std::cout<<std::setprecision(10)<<"goed bef enu2ned\n"<<*gps_coord<<std::endl;}
  gps_coord->x/=factor;
  gps_coord->y/=factor;
  gps_coord->z/=factor;
  return;
}

///given the image coordinates, uav pose and camera info, returns the geodetic coordinates
void flight_planner::georef_image_point(double cam_ori, geometry_msgs::Vector3 imu_ori, geometry_msgs::Vector3 uav_gps_posi,
                        cv::Mat k, geometry_msgs::Vector3 image_coord, geometry_msgs::Vector3 ref_gps_posi,
                        geometry_msgs::Vector3 *georeferenced_point, bool is_degrees, double scale, double fl, bool debug){
  geometry_msgs::Vector3  navf_coord;
  if2navf(cam_ori,imu_ori,uav_gps_posi,k,image_coord,ref_gps_posi,&navf_coord,
          is_degrees,scale,fl,debug);
  georef_navf_point(navf_coord,ref_gps_posi,georeferenced_point,is_degrees,debug);
    return;
}

int main(int argc, char **argv)
{

  std::setprecision (10);
  ros::init(argc, argv, "flight_planner_server");
  std::setprecision(6);
  flight_planner fp;

  ros::spin();

  return 0;
}

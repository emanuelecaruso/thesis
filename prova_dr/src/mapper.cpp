#include "mapper.h"
#include "dtam.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include <cstdlib>
#include <chrono>

void EpipolarLine::printMembers() const {

  sharedCout("\n"+cam->name_+", epipolar line:");
  sharedCout("slope: "+std::to_string(slope));
  sharedCout("c0: "+std::to_string(c0));
  sharedCout("u_or_v: "+std::to_string(u_or_v));
  sharedCout("start: "+std::to_string(start));
  sharedCout("end: "+std::to_string(end));
}

void EpipolarLine::coordToUV(float& coord, Eigen::Vector2f& uv){
  if (u_or_v){
    uv.x()=coord;
    uv.y()= slope*coord+c0;
  }
  else{
    uv.y()= coord;
    uv.x()= coord/slope+c0;
  }
}

void EpipolarLine::UVToCoord(Eigen::Vector2f& uv, float& coord){
  if(u_or_v)
    coord=uv.x();
  else
    coord=uv.y();
}


bool EpipolarLine::resizeLine(){

    float width = cam->cam_parameters_->width;
    float height = cam->cam_parameters_->height;
    float pixel_width = width/((float)cam->cam_parameters_->resolution_x);

    std::vector<float> axes_v {0+(pixel_width/2),width-(pixel_width/2)};
    std::vector<float> axes_h {0+(pixel_width/2),height-(pixel_width/2)};

    bool axis_v_idx = 0;
    bool axis_h_idx;
    if (slope>0)  // 2 and 4 quadrant (y points down)
      axis_h_idx=0;
    else  // 1 and 3 quadrant (y points down)
      axis_h_idx=1;


    Eigen::Vector2f start_uv, end_uv;
    coordToUV(start,start_uv);
    coordToUV(end,end_uv);


    // bring to axis v
    float axis_v= axes_v[axis_v_idx];
    float delta = axis_v-start_uv.x();
    start_uv.y()=start_uv.y()+slope*delta;
    start_uv.x()=axis_v;
    // if out of the square
    if ( (start_uv.y()<axes_h[0]) || (start_uv.y()>axes_h[1]) ){
      // bring to axis h

      float axis_h= axes_h[axis_h_idx];
      float delta = axis_h-start_uv.y();
      start_uv.x()=start_uv.x()+delta/slope;
      start_uv.y()=axis_h;
      // if out of the square again, line does not intersect the rectangle
      if ( (start_uv.x()<axes_v[0]) || (start_uv.x()>axes_v[1]) ){
        sharedCoutDebug("error on start of segment");
        return false;
      }
    }
    UVToCoord(start_uv,start);
    // sharedCoutDebug("slope: "+std::to_string(slope) );
    // sharedCoutDebug("START c0: "+std::to_string(c0) );
    // sharedCoutDebug("vx for end is: "+std::to_string(start_uv.x()));
    // sharedCoutDebug("vy for end is: "+std::to_string(start_uv.y()));
    // sharedCoutDebug("vc for end is: "+std::to_string(start));
    //

    // it may be simplified?? TODO
    // bring to axis v
    axis_v= axes_v[!axis_v_idx];
    delta = axis_v-end_uv.x();
    end_uv.y()=end_uv.y()+slope*delta;
    end_uv.x()=axis_v;
    // if out of the square
    // sharedCoutDebug("\nEND" );
    // sharedCoutDebug("axis_h: "+std::to_string(axis_h));
    // sharedCoutDebug("vx for end is: "+std::to_string(end_uv.x()));
    // sharedCoutDebug("vy for end is: "+std::to_string(end_uv.y()));
    // sharedCoutDebug("vc for end is: "+std::to_string(end));
    if ( (end_uv.y()<axes_h[0]) || (end_uv.y()>axes_h[1]) ){
      // bring to axis h
      float axis_h= axes_h[!axis_h_idx];
      float delta = axis_h-end_uv.y();
      end_uv.x()=end_uv.x()+delta/slope;
      end_uv.y()=axis_h;
      // if out of the square again, line does not intersect the rectangle

      if ( (end_uv.x()<axes_v[0]) || (end_uv.x()>axes_v[1]) ){
        sharedCoutDebug("error on end of segment");
        return false;
      }
    }
    UVToCoord(end_uv,end);




    // pay attention on lineTraverse! TODO
    lineTraverse();
    return true;
}

void EpipolarLine::lineTraverse()
{

    float distance = abs(end-start);

    float pixel_width=cam->cam_parameters_->width/(float)cam->cam_parameters_->resolution_x;

    // alternative solutions? TODO
    int n_uvs= (distance/pixel_width)+1;

    uvs = new std::vector<Eigen::Vector2f>(n_uvs);

    for (int i=0; i<n_uvs; i++){
      float ratio = (float)i/(n_uvs-1);
      float coord = (1-ratio)*start+(ratio)*end;

      Eigen::Vector2f uv;
      coordToUV(coord,uv);
      uvs->at(i)= uv;
    }
}

void EpipolarLine::showEpipolar(float size){
    Image<cv::Vec3b>* image_rgb_new = createEpipolarImg();
    image_rgb_new->show(size);
}

void EpipolarLine::showEpipolarComparison(EpipolarLine* ep_line_2, float size=1){
    Image<cv::Vec3b>* image_rgb_new = createEpipolarImg();
    Image<cv::Vec3b>* image_rgb_new_2 = ep_line_2->createEpipolarImg();
    image_rgb_new->showWithOtherImage(image_rgb_new_2,size);
}

Image<cv::Vec3b>* EpipolarLine::createEpipolarImg(){
    Image<cv::Vec3b>* image_rgb_new = cam->image_rgb_->clone("epipolar_"+cam->name_);

    for( int i=0; i<uvs->size(); i++){
      Eigen::Vector2i pixel;

      cam->uv2pixelCoords(uvs->at(i),pixel);

      if(i==0)
        image_rgb_new->setPixel(pixel, blue);
      else if (i==uvs->size()-1)
        image_rgb_new->setPixel(pixel, red);
      else
        image_rgb_new->setPixel(pixel, magenta);
    }
    return image_rgb_new;

}







// frame coupling
void Mapper::frameCouplingRandom(int& frame_1, int& frame_2 ){
  int max_frame=dtam_->getFrameCurrent();
  frame_1=rand() % max_frame;
  while(true){
    frame_2=rand() % max_frame;
    if (frame_2!=frame_1)
      break;
  }

  sharedCoutDebug("chosen frames: "+std::to_string(frame_1)+ " " +std::to_string(frame_2));
}

void Mapper::frameCouplingLast(int& frame_1, int& frame_2 ){
  int max_frame=dtam_->getFrameCurrent();
  frame_1=max_frame-1;
  frame_2=max_frame-2;

  sharedCoutDebug("chosen frames: "+std::to_string(frame_1)+ " " +std::to_string(frame_2));
}

void Mapper::frameCouplingOpposite(int& frame_1, int& frame_2 ){
  int max_frame=dtam_->getFrameCurrent();
  frame_1=max_frame-1;
  frame_2=0;

  sharedCoutDebug("chosen frames: "+std::to_string(frame_1)+ " " +std::to_string(frame_2));
}

void getParametersABCD(EpipolarLine* ep_line_1, EpipolarLine* ep_line_2,
                      float& A_M,float& B_M,float& C_M,float& D_M,
                      float& A_m,float& B_m,float& C_m,float& D_m){

  // from cam_2 to cam_1
  Eigen::Isometry3f T = (*(ep_line_1->cam->frame_world_wrt_camera_))*(*(ep_line_2->cam->frame_camera_wrt_world_));
  Eigen::Matrix3f r=T.linear();
  Eigen::Vector3f t=T.translation();
  float slope1 = ep_line_1->slope;
  float slope2 = ep_line_2->slope;
  float f = ep_line_1->cam->cam_parameters_->lens;
  float f2 = f*f;
  float w = ep_line_1->cam->cam_parameters_->width;
  float w2 = w*w;
  float h = ep_line_1->cam->cam_parameters_->height;
  float max_depth = ep_line_1->cam->cam_parameters_->max_depth;
  float min_depth = ep_line_1->cam->cam_parameters_->min_depth;
  float d1;
  bool u_or_v = ep_line_1->u_or_v;
  float vh = ep_line_1->c0;

  // max depth
  d1=max_depth;
  if(u_or_v){
    //(4 d1 f r11 - 2 d1 r31 width - 4 d1 f r12 slope1 + 2 d1 r32 slope1 width)
    A_M=(4*d1*f*r(0,0) - 2*d1*r(2,1)*w - 4*d1*f*r(0,1)*slope1 +  4*d1*f*r(0,1)*slope1 + 2*d1*r(2,1)*slope1*w );

    // 4 f  t1 + d1 r31 width  - 2 f t3 width - 4 d1 f  r13
    //  + 2 d1 f height r12 - 4 d1 f r12 vh - 2 d1 f r11 width + 2 d1 f r33 width - d1 height r32 width
    //  + 2 d1 r32 vh width)
    B_M=4*f2*t(0) + d1*r(2,0)*w2 - 2*f*t(2)*w - 4*d1*f2*r(0,2)
        + 2*d1*f*h*r(0,1) - 4*d1*f*r(0,1)*vh - 2*d1*f*r(0,0)*w + 2*d1*f*r(2,2)*w - d1*h*r(2,1)*w
        + 2*d1*r(2,1)*vh*w ;

    // (4 d1 r32 slope1 - 4 d1 r31)
    C_M=4*d1*r(2,1)*slope1 - 4*d1*r(2,0);

    // 4 d1 f r33 - 4 f t3 - 2 d1 height r32 + 4 d1 r32 vh + 2 d1 r31 width
    D_M=4*d1*f*r(2,2) - 4*f*t(2) - 2*d1*h*r(2,1) + 4*d1*r(2,1)*vh + 2*d1*r(2,0)*w;
  }
  else{

  }

  // min depth
  d1=min_depth;
  if(u_or_v){
    //(4 d1 f r11 - 2 d1 r31 width - 4 d1 f r12 slope1 + 2 d1 r32 slope1 width)
    A_m=(4*d1*f*r(0,0) - 2*d1*r(2,1)*w - 4*d1*f*r(0,1)*slope1 +  4*d1*f*r(0,1)*slope1 + 2*d1*r(2,1)*slope1*w );

    // 4 f  t1 + d1 r31 width  - 2 f t3 width - 4 d1 f  r13
    //  + 2 d1 f height r12 - 4 d1 f r12 vh - 2 d1 f r11 width + 2 d1 f r33 width - d1 height r32 width
    //  + 2 d1 r32 vh width)
    B_m=4*f2*t(0) + d1*r(2,0)*w2 - 2*f*t(2)*w - 4*d1*f2*r(0,2)
        + 2*d1*f*h*r(0,1) - 4*d1*f*r(0,1)*vh - 2*d1*f*r(0,0)*w + 2*d1*f*r(2,2)*w - d1*h*r(2,1)*w
        + 2*d1*r(2,1)*vh*w ;

    // (4 d1 r32 slope1 - 4 d1 r31)
    C_m=4*d1*r(2,1)*slope1 - 4*d1*r(2,0);

    // 4 d1 f r33 - 4 f t3 - 2 d1 height r32 + 4 d1 r32 vh + 2 d1 r31 width
    D_m=4*d1*f*r(2,2) - 4*f*t(2) - 2*d1*h*r(2,1) + 4*d1*r(2,1)*vh + 2*d1*r(2,0)*w;
  }
  else{

  }


}

bool buildFeatureVec(EpipolarLine*& ep_line_1, EpipolarLine*& ep_line_2,
          std::vector<Feature*>*& feats_1, std::vector<Feature*>*& feats_2)
{
  int feats_1_size = ep_line_1->uvs->size();
  int feats_2_size = ep_line_2->uvs->size();
  feats_1 = new std::vector<Feature*>(feats_1_size);
  feats_2 = new std::vector<Feature*>(feats_2_size);

  float A_M, B_M, C_M, D_M, A_m, B_m, C_m, D_m;

  getParametersABCD(ep_line_1, ep_line_2,
                     A_M, B_M, C_M, D_M,
                     A_m, B_m, C_m, D_m);

  for(int i=0; i<feats_1_size; i++){
    cv::Vec3b gradient;
    cv::Vec3b color;
    float upperbound;
    float lowerbound;
    feats_1->at(i)= new Feature(i,color,gradient,upperbound,lowerbound);
  }

}

bool Mapper::computeEpipolarLineCouple(const Camera* cam_1, const Camera* cam_2,
        Eigen::Vector2f& uv_1, EpipolarLine*& ep_line_1,EpipolarLine*& ep_line_2)
{

  Eigen::Vector2f cam_2_on_cam_1;
  cam_1->projectCam(cam_2, cam_2_on_cam_1);

  ep_line_1 = new EpipolarLine(cam_1, cam_2_on_cam_1,uv_1);


  if(!ep_line_1->resizeLine()){
    sharedCoutDebug("ep1 returned false");
    return false;
  }

  Eigen::Vector2f cam_1_on_cam_2;
  cam_2->projectCam(cam_1, cam_1_on_cam_2);

  Eigen::Vector3f p;
  // select better uv TODO
  float depth = cam_1->cam_parameters_->max_depth;
  cam_1->pointAtDepth(uv_1, depth, p);
  Eigen::Vector2f uv_2;
  cam_2->projectPoint(p,uv_2);

  ep_line_2 = new EpipolarLine(cam_2, cam_1_on_cam_2,uv_2);

  if(!ep_line_2->resizeLine()){
    sharedCoutDebug("ep2 returned false");
    return false;}

  return true;
}



void Mapper::doMapping(){

  int frame_1;
  int frame_2;

  std::unique_lock<std::mutex> locker(dtam_->mu_frame_);
  dtam_->first_2_frames_available_.wait(locker, [&](){return Mapper::dtam_->frame_current_>=2;});
  locker.unlock();
  while(true){
    // frameCouplingRandom(frame_1, frame_2 );
    // frameCouplingLast(frame_1, frame_2 );
    frameCouplingOpposite(frame_1, frame_2 );

    Camera* cam_1 = dtam_->camera_vector_->at(frame_1);
    Camera* cam_2 = dtam_->camera_vector_->at(frame_2);
    EpipolarLine* ep_line_1; EpipolarLine* ep_line_2;

    // cam_1->printMembers();
    Eigen::Vector2f uv_1;
    cam_2->getCentreAsUV(uv_1);
    // cam_2->sampleRandomUv(uv_2);

    if (!computeEpipolarLineCouple(cam_1, cam_2, uv_1, ep_line_1, ep_line_2)){
      break;
      continue; //take other frame couple TODO
    }

    locker.lock();
    // float size = 1;
    float size = 1.2;
    // float size = 2;
    // ep_line_1->printMembers();
    // ep_line_2->printMembers();
    // ep_line_1->showEpipolarComparison(ep_line_2,size);
    cam_1->curvature_->showWithOtherImage(cam_2->curvature_,size);
    // cam_1->grad_x_->showWithOtherImage(cam_2->grad_x_,size);
    // cam_1->grad_y_->showWithOtherImage(cam_2->grad_y_,size);
    cv::waitKey(0);
    cv::destroyAllWindows();
    locker.unlock();

  }
}

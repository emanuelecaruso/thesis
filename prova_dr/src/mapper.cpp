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
  sharedCout("vh: "+std::to_string(vh));
  sharedCout("u_or_v: "+std::to_string(u_or_v));
  sharedCout("start: "+std::to_string(start));
  sharedCout("end: "+std::to_string(end));
}

void EpipolarLine::coordToUV(float& coord, Eigen::Vector2f& uv){
  if (u_or_v){
    uv.x()=coord;
    uv.y()= slope*coord+vh;
  }
  else{
    uv.y()= coord;
    uv.x()= (coord-vh)/slope;
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
      start_uv.x()=start_uv.x()+slope*delta;
      start_uv.y()=axis_h;
      // if out of the square again, line does not intersect the rectangle
      if ( (start_uv.x()<axes_v[0]) || (start_uv.x()>axes_v[1]) ){
        return false;
      }
    }
    UVToCoord(start_uv,start);



    // it may be simplified?? TODO
    // bring to axis v
    axis_v= axes_v[!axis_v_idx];
    delta = axis_v-end_uv.x();
    end_uv.y()=end_uv.y()+slope*delta;
    end_uv.x()=axis_v;
    // if out of the square
    if ( (end_uv.y()<axes_h[0]) || (end_uv.y()>axes_h[1]) ){
      // bring to axis h
      float axis_h= axes_h[!axis_h_idx];
      float delta = axis_h-end_uv.y();
      end_uv.x()=end_uv.x()+slope*delta;
      end_uv.y()=axis_h;
      // if out of the square again, line does not intersect the rectangle
      if ( (end_uv.x()<axes_v[0]) || (end_uv.x()>axes_v[1]) ){
        return false;
      }
    }
    UVToCoord(end_uv,end);

    // pay attention on traverseLine! TODO
    lineTraverse();
    return true;
}

void EpipolarLine::lineTraverse()
{
    float distance = end-start;

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

void EpipolarLine::showEpipolar(int size){
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
    image_rgb_new->show(size);
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


bool Mapper::computeEpipolarLineCouple(const Camera* cam_1, const Camera* cam_2,
            Eigen::Vector2f& uv_1, EpipolarLine*& ep_line_1,EpipolarLine*& ep_line_2)
{

  Eigen::Vector2f cam_2_on_cam_1;
  cam_1->projectCam(cam_2, cam_2_on_cam_1);

  ep_line_1 = new EpipolarLine(cam_1, cam_2_on_cam_1,uv_1);

  // sharedCoutDebug("cam2 projected on cam1: "+ std::to_string(cam_2_on_cam_1.x()) +", "+std::to_string(cam_2_on_cam_1.y()));

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

}



void Mapper::doMapping(){

  int frame_1;
  int frame_2;

  std::unique_lock<std::mutex> locker(dtam_->mu_frame_);
  dtam_->first_2_frames_available_.wait(locker, [&](){return Mapper::dtam_->frame_current_>=2;});
  locker.unlock();
  while(true){
    frameCouplingRandom(frame_1, frame_2 );
    // frameCouplingLast(frame_1, frame_2 );

    Camera* cam_1 = dtam_->camera_vector_->at(frame_1);
    Camera* cam_2 = dtam_->camera_vector_->at(frame_2);
    EpipolarLine* ep_line_1; EpipolarLine* ep_line_2;

    // cam_1->printMembers();
    Eigen::Vector2f uv_1;
    cam_2->getCentreAsUV(uv_1);
    // cam_2->sampleRandomUv(uv_2);

    if (!computeEpipolarLineCouple(cam_1, cam_2, uv_1, ep_line_1, ep_line_2)){
      continue; //take other frame couple TODO
    }

    locker.lock();
    int size = 2;
    // ep_line_1->printMembers();
    // ep_line_2->printMembers();
    ep_line_1->showEpipolar(size);
    ep_line_2->showEpipolar(size);
    cv::waitKey(0);
    cv::destroyAllWindows();
    locker.unlock();

  }
}

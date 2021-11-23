#include "mapper.h"
#include "epline.h"
#include "dtam.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include <cstdlib>
#include <chrono>

void CamCouple::getSlopeParameters(){
  // 4*r20*t1 - 4*f*r10*t2 - 2*h*r20*t2
  A_s=4*r(2,0)*t(1) - 4*f*r(1,0)*t(2) - 2*h*r(2,0)*t(2);

  // 4*f*r11*t2 - 4*r12*t1 + 2*h*r12*t2
  B_s=4*f*r(1,1)*t(2) - 4*r(1,2)*t(1) + 2*h*r(1,2)*t(2);

  //4*f^2*r12*t2 - 2*f*h*t2^2 - 4*f^2*t1*t2 - h^2*r12*t2 - 4*f*r22*t1 + 4*f*t1*t2 + 2*h*r12*t1 - 2*r20*t1*w - 2*f*h*r11*t2 + 2*f*h*r22*t2 + 2*f*r10*t2*w + h*r20*t2*w
  C_s=4*f2*r(1,2)*t(2) - 2*f*h*t22 - 4*f2*t(1)*t(2) - h2*r(1,2)*t(2) - 4*f*r(2,2)*t(1) + 4*f*t(1)*t(2) + 2*h*r(1,2)*t(1) - 2*r(2,0)*t(1)*w - 2*f*h*r(1,1)*t(2) + 2*f*h*r(2,2)*t(2) + 2*f*r(1,0)*t(2)*w + h*r(2,0)*t(2)*w;

  // 4*r20*t0 + 4*f*r00*t2 - 2*r20*t2*w
  D_s=4*r(2,0)*t(0) + 4*f*r(0,0)*t(2) - 2*r(2,0)*t(2)*w;

  // 2*r12*t2*w - 4*f*r01*t2 - 4*r12*t0
  E_s=2*r(1,2)*t(2)*w - 4*f*r(0,1)*t(2) - 4*r(1,2)*t(0);

  // 4*f^2*t0*t2 - 4*f^2*r02*t2 - 2*f*t2^2*w + r20*t2*w^2 - 4*f*r22*t0 + 4*f*t0*t2 + 2*h*r12*t0 - 2*r20*t0*w + 2*f*h*r01*t2 - 2*f*r00*t2*w + 2*f*r22*t2*w - h*r12*t2*w
  F_s=4*f2*t(0)*t(2) - 4*f2*r(0,2)*t(2) - 2*f*t22*w + r(2,0)*t(2)*w2 - 4*f*r(2,2)*t(0) + 4*f*t(0)*t(2) + 2*h*r(1,2)*t(0) - 2*r(2,0)*t(0)*w + 2*f*h*r(0,1)*t(2) - 2*f*r(0,0)*t(2)*w + 2*f*r(2,2)*t(2)*w - h*r(1,2)*t(2)*w;
}

void CamCouple::getBoundsParameters(){

  // 2*r20*w - 4*f*r00
  A_bu=2*r(2,0)*w - 4*f*r(0,0);

  // 4*f*r01 - 2*r12*w
  B_bu=4*f*r(0,1) - 2*r(1,2)*w;

  // 4*f^2*r02 - r20*w^2 - 2*f*h*r01 + 2*f*r00*w - 2*f*r22*w + h*r12*w
  C_bu=4*f2*r(0,2) - r(2,0)*w2 - 2*f*h*r(0,1) + 2*f*r(0,0)*w - 2*f*r(2,2)*w + h*r(1,2)*w;

  // - 4*t0*f^2 + 2*t2*w*f
  D_bu=- 4*t(0)*f2 + 2*t(2)*w*f;

  // 2*r20*w - 4*f*r00
  E_bu=2*r(2,0)*w - 4*f*r(0,0);

  //4*f*r01 - 2*r12*w
  F_bu=4*f*r(0,1) - 2*r(1,2)*w;

  //4*f^2*r02 - r20*w^2 - 2*f*h*r01 + 2*f*r00*w - 2*f*r22*w + h*r12*w
  G_bu=4*f2*r(0,2) - r(2,0)*w2 - 2*f*h*r(0,1) + 2*f*r(0,0)*w - 2*f*r(2,2)*w + h*r(1,2)*w;

  //- 4*t0*f^2 + 2*t2*w*f
  H_bu=- 4*t(0)*f2 + 2*t(2)*w*f;


  A_bv=4*f*r(1,0) + 2*h*r(2,0);

  B_bv=- 4*f*r(1,1) - 2*h*r(1,2);

  C_bv=h2*r(1,2) - 4*f2*r(1,2) + 2*f*h*r(1,1) - 2*f*h*r(2,2) - 2*f*r(1,0)*w - h*r(2,0)*w;

  D_bv=4*t(1)*f2 + 2*h*t(2)*f;

  E_bv=4*f*r(1,0) + 2*h*r(2,0);

  F_bv=- 4*f*r(1,1) - 2*h*r(1,2);

  G_bv=h2*r(1,2) - 4*f2*r(1,2) + 2*f*h*r(1,1) - 2*f*h*r(2,2) - 2*f*r(1,0)*w - h*r(2,0)*w;

  H_bv=4*t(1)*f2 + 2*h*t(2)*f;

}

EpipolarLine* CamCouple::getEpSegment(float u1, float v1){
  // get slope
  // void getSlope(float u1, float v1, float& slope_m);



}




// frame coupling
void Mapper::frameCouplingRandom(int& frame_idx_r, int& frame_idx_m ){
  int max_frame=dtam_->getFrameCurrent();
  frame_idx_r=rand() % max_frame;
  while(true){
    frame_idx_m=rand() % max_frame;
    if (frame_idx_m!=frame_idx_r)
      break;
  }
}

void Mapper::frameCouplingLast(int& frame_idx_r, int& frame_idx_m ){
  int max_frame=dtam_->getFrameCurrent();
  frame_idx_r=max_frame-1;
  frame_idx_m=max_frame-2;
}

void Mapper::frameCouplingOpposite(int& frame_idx_r, int& frame_idx_m ){
  int max_frame=dtam_->getFrameCurrent();
  frame_idx_r=max_frame-1;
  frame_idx_m=0;
}



bool Mapper::computeEpipolarLineCouple(const CameraForMapping* cam_1, const CameraForMapping* cam_2,
        Eigen::Vector2f& uv_1, EpipolarLine*& ep_line_1,EpipolarLine*& ep_line_2)
{

  Eigen::Vector2f cam_2_on_cam_1;
  cam_1->projectCam(cam_2, cam_2_on_cam_1);

  ep_line_1 = new EpipolarLine(cam_1, cam_2_on_cam_1,uv_1);


  if(!ep_line_1->stretchToBorders()){
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

  if(!ep_line_2->stretchToBorders()){
    sharedCoutDebug("ep2 returned false");
    return false;}

  return true;
}



void Mapper::selectCandidates(){
  int idx=dtam_->keyframe_vector_->back();
  CameraForMapping* cam_r= dtam_->camera_vector_->at(idx);
  cam_r->collectRegions(threshold_);
  cam_r->selectCandidates(num_candidates_);

}

void Mapper::propagateOldCandidates(){

}

bool Mapper::initializeCandidates(const CameraForMapping* cam_r,
            const CameraForMapping* cam_m, int& current_r_idx){

   CamCouple* cam_couple = new CamCouple(cam_r, cam_m);

  // iterate along candidates
  while(!(cam_r->candidates_->empty())){
    // check if there is a new frame
    if(current_r_idx<dtam_->frame_current_-1){
      current_r_idx=dtam_->frame_current_-1;
      cam_r = dtam_->camera_vector_->at(current_r_idx);
      cam_m = dtam_->camera_vector_->at(current_r_idx-1);
    }

    Candidate* curr_cand=cam_r->candidates_->back();
    cam_r->candidates_->pop_back();
    // get epipolar segment
    cam_couple->getEpSegment(curr_cand->first[0],curr_cand->first[1]);
    // search along epipolar line

    // compute mse with c,dd,dh,dv
    // roll -> associate dd,dh,dv

    // if minimum is clear, add as active point

  }



  return false;
}

void Mapper::doMapping(){

  // int frame_idx_r;
  // int frame_idx_m;
  //
  // while(true){
  //   // frameCouplingRandom(frame_idx_r, frame_idx_m );
  //   // frameCouplingLast(frame_idx_r, frame_idx_m );
  //   frameCouplingOpposite(frame_idx_r, frame_idx_m );
  //   // sharedCoutDebug("chosen frames: "+std::to_string(frame_idx_r)+ " " +std::to_string(frame_idx_m));
  //
  //   CameraForMapping* cam_r = dtam_->camera_vector_->at(frame_idx_r);
  //   CameraForMapping* cam_m = dtam_->camera_vector_->at(frame_idx_m);
  //   EpipolarLine* ep_line_r; EpipolarLine* ep_line_m;
  //
  //
  //
  //   // float size = 1;
  //   // float size = 1.3;
  //   // float size = 2;
  //   // locker.lock();
  //   //
  //   // cam_r->showCandidates(size);
  //   // cv::waitKey(0);
  //   // // cv::destroyAllWindows();
  //   // locker.unlock();
  //
  // }

}


void Mapper::getParametersABCD(EpipolarLine* ep_line_source, EpipolarLine* ep_line_range,
                                float depth, Eigen::Vector4f& abcd){

  // from cam_2 to cam_1
  Eigen::Isometry3f T = (*(ep_line_range->cam->frame_world_wrt_camera_))*(*(ep_line_source->cam->frame_camera_wrt_world_));
  Eigen::Matrix3f r=T.linear();
  Eigen::Vector3f t=T.translation();

  float slope1 = ep_line_source->slope;
  float slope2 = ep_line_range->slope;
  float f = ep_line_source->cam->cam_parameters_->lens;
  float f2 = f*f;
  float w = ep_line_source->cam->cam_parameters_->width;
  float h = ep_line_source->cam->cam_parameters_->height;
  float w2 = w*w;
  float h2 = h*h;
  float d1;
  bool u_or_v = ep_line_source->u_or_v;
  float c0 = ep_line_source->c0;

  // max depth
  d1=depth;
  if(u_or_v){


    abcd[0]=-d1 * (2*f*r(0,0) - r(2,0)*w - 2*f*r(0,1)*slope1 + r(1,2)* slope1*w)*2;

    abcd[1]= 2*f*t(2)*w - d1*r(2,0)*w2 - 4*f2 *t(0) + 4*d1*f2 *r(0,2) - 2*d1*f*h*r(0,1) + 4*d1*f*r(0,1)*c0
    + 2*d1*f*r(0,0)*w - 2*d1*f*r(2,2)*w + d1*h*r(1,2)*w - 2*d1*r(1,2)*c0*w;

    abcd[2]=4*d1*(r(2,0) - r(1,2)*slope1);

    abcd[3]=4*f*t(2) - 4*d1*f*r(2,2) + 2*d1*h*r(1,2) - 4*d1*r(1,2)*c0 - 2*d1*r(2,0)*w;

  }
  else{


    abcd[0]= d1*(2*f*r(1,0) + h*r(2,0) - 2*f*r(1,1)*slope1 - h*r(1,2)*slope1)*2;

    abcd[1]=  slope1*(4*f2 *t(1) + 2*f*h*t(2) - 4*d1*f2* r(1,2) + d1*h2* r(1,2) + 2*d1*f*h*r(1,1) - 2*d1*f*h*r(2,2)
    + 4*d1*f*r(1,0)*c0 - 2*d1*f*r(1,0)*w + 2*d1*h*r(2,0)*c0 - d1*h*r(2,0)*w);

    abcd[2]=4*d1*(r(2,0) - r(1,2)*slope1);

    abcd[3]=slope1*(2*f*t(2) - 2*d1*f*r(2,2) + d1*h*r(1,2) + 2*d1*r(2,0)*c0 - d1*r(2,0)*w)*2;

  }

}

float Mapper::coord2FromCoord1(float coord1, Eigen::Vector4f& abcd){
  float coord2 = (abcd[0]*coord1+abcd[1])/(abcd[2]*coord1+abcd[3]);
  return coord2;
}

void Mapper::showRangeStudy(EpipolarLine* ep_line_source, EpipolarLine* ep_line_range, int uvs_idx, float size){


    ///////////////////////////////////////
    Image<cv::Vec3b>* img_source=ep_line_source->createEpipolarImg("source");
    Image<cv::Vec3b>* img_gt=ep_line_range->createEpipolarImg("gt");
    Image<cv::Vec3b>* img_query=ep_line_range->createEpipolarImg("query");

    // 1 range with projection

    Eigen::Vector2i pixel_coords;
    Eigen::Vector2f uv_source= ep_line_source->uvs->at(uvs_idx);
    ep_line_source->cam->uv2pixelCoords(uv_source, pixel_coords);
    img_source->setPixel(pixel_coords,black);

    Eigen::Vector3f p;
    Eigen::Vector2f uv_range;

    float max_depth = ep_line_source->cam->cam_parameters_->max_depth;
    const cv::Vec3b& cl1=blue;

    float min_depth = ep_line_source->cam->cam_parameters_->min_depth;
    const cv::Vec3b& cl2=red;

    // project uv at max depth and set pxl
    ep_line_source->cam->pointAtDepth(uv_source, max_depth, p);
    ep_line_range->cam->projectPoint(p,uv_range);
    ep_line_range->cam->uv2pixelCoords(uv_range, pixel_coords);
    img_gt->setPixel(pixel_coords,cl1);
    sharedCoutDebug("pxl coords u gt: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));

    // project uv at min depth and set pxl
    ep_line_source->cam->pointAtDepth(uv_source, min_depth, p);
    ep_line_range->cam->projectPoint(p,uv_range);
    ep_line_range->cam->uv2pixelCoords(uv_range, pixel_coords);
    img_gt->setPixel(pixel_coords,cl2);
    sharedCoutDebug("pxl coords v gt: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));

    ////////////////////////////////////////

    // // 2 range with formula
    float coord_source;
    ep_line_source->UVToCoord( uv_source, coord_source);

    float coord_range;

    Eigen::Vector4f abcd_m;
    Eigen::Vector4f abcd_M;

    // min depth
    getParametersABCD( ep_line_source, ep_line_range, min_depth, abcd_m);

    // max depth
    getParametersABCD( ep_line_source, ep_line_range, max_depth, abcd_M);


    // formula for max depth
    coord_range=coord2FromCoord1( coord_source, abcd_M);
    ep_line_range->coordToUV( coord_range, uv_range);
    ep_line_range->cam->uv2pixelCoords(uv_range, pixel_coords);
    img_query->setPixel(pixel_coords,cl1);
    sharedCoutDebug("pxl coords u: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));


    coord_range = coord2FromCoord1( coord_source, abcd_m);
    ep_line_range->coordToUV( coord_range, uv_range);
    ep_line_range->cam->uv2pixelCoords(uv_range, pixel_coords);
    img_query->setPixel(pixel_coords,cl2);
    sharedCoutDebug("pxl coords v: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));




    /////////////////////////////////////////
    // hconcat
    img_source->show(size);
    img_gt->show(size);
    img_query->show(size);

}

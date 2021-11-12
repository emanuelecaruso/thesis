#include "mapper.h"
#include "epline.h"
#include "dtam.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include <cstdlib>
#include <chrono>

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


bool buildFeatureVec(EpipolarLine*& ep_line_1, EpipolarLine*& ep_line_2,
          std::vector<Feature*>*& feats_1, std::vector<Feature*>*& feats_2)
{
  // int feats_1_size = ep_line_1->uvs->size();
  // int feats_2_size = ep_line_2->uvs->size();
  // feats_1 = new std::vector<Feature*>(feats_1_size);
  // feats_2 = new std::vector<Feature*>(feats_2_size);
  //
  // float A_M, B_M, C_M, D_M, A_m, B_m, C_m, D_m;
  //
  // getParametersABCD(ep_line_1, ep_line_2,
  //                    A_M, B_M, C_M, D_M,
  //                    A_m, B_m, C_m, D_m);
  //
  // for(int i=0; i<feats_1_size; i++){
  //   cv::Vec3b gradient;
  //   cv::Vec3b color;
  //   float upperbound;
  //   float lowerbound;
  //   feats_1->at(i)= new Feature(i,color,gradient,upperbound,lowerbound);
  // }

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
    // cam_2->getCenterAsUV(uv_1);
    cam_2->sampleRandomUv(uv_1);

    if (!computeEpipolarLineCouple(cam_1, cam_2, uv_1, ep_line_1, ep_line_2)){
      continue; //take other frame couple TODO
    }

    // float size = 1;
    // float size = 1.3;
    float size = 2;
    locker.lock();
    // ep_line_1->printMembers();
    // ep_line_2->printMembers();
    showRangeStudy(ep_line_2,ep_line_1, 300,size);
    // ep_line_1->showEpipolarComparison(ep_line_2,"epipolar comparison",size);
    // dtam_->showFeatures(frame_1, size);
    cv::waitKey(0);
    // cv::destroyAllWindows();
    locker.unlock();

  }
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
    Image<cv::Vec3b>* img_gt=ep_line_range->createEpipolarImg("gt");
    Image<cv::Vec3b>* img_query=ep_line_range->createEpipolarImg("query");

    // 1 range with projection

    Eigen::Vector2f uv_source= ep_line_source->uvs->at(uvs_idx);
    Eigen::Vector3f p;
    Eigen::Vector2f uv_range;
    Eigen::Vector2i pixel_coords;

    float max_depth = ep_line_source->cam->cam_parameters_->max_depth;
    const cv::Vec3b& cl1=blue;

    float min_depth = ep_line_source->cam->cam_parameters_->min_depth;
    const cv::Vec3b& cl2=red;

    // project uv at max depth and set pxl
    ep_line_source->cam->pointAtDepth(uv_source, max_depth, p);
    ep_line_range->cam->projectPoint(p,uv_range);
    ep_line_range->cam->uv2pixelCoords(uv_range, pixel_coords);
    img_gt->setPixel(pixel_coords,cl1);
    sharedCoutDebug("pxl coords: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));

    // project uv at min depth and set pxl
    ep_line_source->cam->pointAtDepth(uv_source, min_depth, p);
    ep_line_range->cam->projectPoint(p,uv_range);
    ep_line_range->cam->uv2pixelCoords(uv_range, pixel_coords);
    img_gt->setPixel(pixel_coords,cl2);
    sharedCoutDebug("pxl coords: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));

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
    sharedCoutDebug("pxl coords: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));


    coord_range = coord2FromCoord1( coord_source, abcd_m);
    ep_line_range->coordToUV( coord_range, uv_range);
    ep_line_range->cam->uv2pixelCoords(uv_range, pixel_coords);
    img_query->setPixel(pixel_coords,cl2);
    sharedCoutDebug("pxl coords: "+std::to_string(pixel_coords.x())+", "+std::to_string(pixel_coords.y()));




    /////////////////////////////////////////
    // hconcat
    img_gt->show(size);
    img_query->show(size);

}

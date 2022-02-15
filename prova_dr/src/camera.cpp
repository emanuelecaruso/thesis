#include "camera.h"
#include <thread>
#include <vector>
#include <mutex>
#include <cstdlib>

using namespace std;
using namespace pr;

void Camera::printMembers() const {

  std::cout << "name: " << name_ << std::endl;
  cam_parameters_->printMembers();
  std::cout << "K: " << *K_ << std::endl;
  std::cout << "Kinv: " << *Kinv_ << std::endl;
  std::cout << "frame_world_wrt_camera LINEAR:\n" << (*frame_world_wrt_camera_).linear() << std::endl;
  std::cout << "frame_world_wrt_camera TRANSL:\n" << (*frame_world_wrt_camera_).translation() << std::endl;
  std::cout << "frame_camera_wrt_world LINEAR:\n" << (*frame_camera_wrt_world_).linear() << std::endl;
  std::cout << "frame_camera_wrt_world TRANSL:\n" << (*frame_camera_wrt_world_).translation() << std::endl;
  std::cout << "\n" << std::endl;

}

// sampling
void Camera::sampleRandomPixel(pxl& pixel_coords){
  pixel_coords.x() = rand() % cam_parameters_->resolution_x;
  pixel_coords.y() = rand() % cam_parameters_->resolution_y;
}
void Camera::sampleRandomUv(Eigen::Vector2f& uv){
  uv.x() = ((float)rand()/RAND_MAX) * cam_parameters_->width;
  uv.y() = ((float)rand()/RAND_MAX) * cam_parameters_->height;
}


// access
void Camera::getCenterAsUV(Eigen::Vector2f& uv) const{
  uv.x() = cam_parameters_->width/2;
  uv.y() = cam_parameters_->height/2;
}

void Camera::getCentreAsPixel(pxl& pixel_coords) const{
  pixel_coords.x() = cam_parameters_->resolution_x/2;
  pixel_coords.y() = cam_parameters_->resolution_y/2;
}

float Camera::getPixelWidth(int level) const{
  float pixel_width= cam_parameters_->pixel_width*pow(2,level+1);
  return pixel_width;
}


// assign
void Camera::assignPose(Eigen::Isometry3f& frame_camera_wrt_world){
  *frame_camera_wrt_world_=frame_camera_wrt_world;
  *frame_world_wrt_camera_=frame_camera_wrt_world.inverse();
}


void Camera::clearImgs(){
  invdepth_map_->setAllPixels(1.0);
}

Eigen::Matrix3f* Camera::compute_K(){

  float lens = cam_parameters_->lens;
  float width = cam_parameters_->width;
  float height = cam_parameters_->height;

  Eigen::Matrix3f* K = new Eigen::Matrix3f;
  *K <<
      lens ,  0   , width/2,
      0    ,  lens, height/2,
      0    ,  0   ,       1 ;

  return K;
}

void Camera::pixelCoords2uv(const pxl& pixel_coords, Eigen::Vector2f& uv, int level) const {

  int resolution_x=cam_parameters_->resolution_x/(pow(2,level+1));
  int resolution_y=cam_parameters_->resolution_y/(pow(2,level+1));

  uv.x()=((float)pixel_coords.x()/(float)resolution_x)*cam_parameters_->width;
  uv.y()=((float)pixel_coords.y()/(float)resolution_y)*cam_parameters_->height;
}

void Camera::pixelCoords2uv(const pxl& pixel_coords, Eigen::Vector2f& uv) const {
  pixelCoords2uv( pixel_coords, uv, -1);
}

void Camera::uv2pixelCoords(const Eigen::Vector2f& uv, pxl& pixel_coords, int level) const {

  int resolution_x=cam_parameters_->resolution_x/(pow(2,level+1));
  int resolution_y=cam_parameters_->resolution_y/(pow(2,level+1));

  pixel_coords.x()=((uv.x()/cam_parameters_->width)*(float)resolution_x);
  pixel_coords.y()=((uv.y()/cam_parameters_->height)*(float)resolution_y);
}

void Camera::uv2pixelCoords(const Eigen::Vector2f& uv, pxl& pixel_coords) const {
uv2pixelCoords(uv, pixel_coords, -1);
}



void Camera::pointAtDepthInCamFrame(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p_incamframe ) const {

  Eigen::Vector3f p_K_dot_incamframe;
  Eigen::Vector2f product = uv * depth;
  p_K_dot_incamframe.x() = product.x();
  p_K_dot_incamframe.y() = product.y();
  p_K_dot_incamframe.z() = depth;
  p_incamframe = (*Kinv_)*p_K_dot_incamframe;

}

void Camera::pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const {

  Eigen::Vector3f p_incamframe;
  pointAtDepthInCamFrame( uv, depth, p_incamframe);
  p = *frame_camera_wrt_world_*p_incamframe;

}

void Camera::pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p, Eigen::Vector3f& p_incamframe) const {

  pointAtDepthInCamFrame( uv, depth, p_incamframe);
  p = *frame_camera_wrt_world_*p_incamframe;

}


bool Camera::projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const {


  Eigen::Vector3f p_cam = *frame_world_wrt_camera_*p;

  // save value of z
  p_cam_z=p_cam.z();

  Eigen::Vector3f p_proj = (*K_)*p_cam;

  uv = p_proj.head<2>()*(1./p_proj.z());

  // return wether the projected point is in front or behind the camera
  if (p_proj.z()<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p_cam = *frame_world_wrt_camera_*p;

  Eigen::Vector3f p_proj = (*K_)*p_cam;

  uv = p_proj.head<2>()*(1./p_proj.z());

  // return wether the projected point is in front or behind the camera
  if (p_proj.z()<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectPointInCamFrame(const Eigen::Vector3f& p_incamframe, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p_proj = (*K_)*p_incamframe;

  uv = p_proj.head<2>()*(1./p_proj.z());

  // return wether the projected point is in front or behind the camera
  if (p_proj.z()<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectCam(Camera* cam_to_be_projected, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p = cam_to_be_projected->frame_camera_wrt_world_->translation();

  bool out = projectPoint(p, uv);
  return out;

}

bool Camera::projectCam(Camera* cam_to_be_projected, Eigen::Vector2f& uv, float& p_cam_z  ) const {

  Eigen::Vector3f p = cam_to_be_projected->frame_camera_wrt_world_->translation();

  bool out = projectPoint(p, uv, p_cam_z);
  return out;

}

void Camera::saveRGB(const std::string& path) const {
  cv::imwrite(path+ "/rgb_" +name_+".png", image_intensity_->image_);
}

void Camera::saveDepthMap(const std::string& path) const {
  cv::Mat ucharImg;
  invdepth_map_->image_.convertTo(ucharImg, CV_32FC1, 255.0);
  cv::imwrite(path+ "/depth_" +name_+".png", ucharImg);

}

Image<pixelIntensity>* Camera::returnIntensityImgFromPath(const std::string& path_rgb){

  Image<pixelIntensity>* img = new Image<pixelIntensity>(name_);
  img->image_=cv::imread(path_rgb,cv::IMREAD_GRAYSCALE);
  img->image_.convertTo(img->image_, pixelIntensity_CODE, pixelIntensity_maxval/255.0);
  return img;
}


void Camera::loadWhiteDepth(){
  invdepth_map_->initImage(cam_parameters_->resolution_y,cam_parameters_->resolution_x);
  invdepth_map_->setAllPixels(1.0); // initialize images with white color

}

void Camera::loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f){
  float resolution_x = cam_parameters_->resolution_x;
  float resolution_y = cam_parameters_->resolution_y;

  Eigen::Matrix3f R;
  R <<
    f[0], f[1], f[2],
    f[3], f[4], f[5],
    f[6], f[7], f[8];

  Eigen::Vector3f t(f[9],f[10],f[11]);
  frame_camera_wrt_world_ = new Eigen::Isometry3f;
  frame_camera_wrt_world_->linear()=R;
  frame_camera_wrt_world_->translation()=t;

  frame_world_wrt_camera_ = new Eigen::Isometry3f;
  *frame_world_wrt_camera_=frame_camera_wrt_world_->inverse();

}



void Camera::loadDepthMap(const std::string& path){
  invdepth_map_->image_=cv::imread(path, cv::IMREAD_ANYDEPTH);
  invdepth_map_->image_/=1.0908;
}

void Camera::showRGB(int image_scale) const {
  image_intensity_->show(image_scale);
}

void Camera::showDepthMap(int image_scale) const {
  invdepth_map_->show(image_scale);
}

void Candidate::marginalize() const{
  std::vector<Candidate*>* v1 = region_sampling_->cands_vec_;
  v1->erase(std::remove(v1->begin(), v1->end(), this), v1->end());

  std::vector<Candidate*>* v2 = cam_->candidates_;
  v2->erase(std::remove(v2->begin(), v2->end(), this), v2->end());

  for(RegionWithCandidates* reg : *regions_coarse_){
    std::vector<Candidate*>* v3 = reg->cands_vec_;
    v3->erase(std::remove(v3->begin(), v3->end(), this), v3->end());
  }
  delete this;
}

void Candidate::setInvdepthGroundtruth(){

  pxl pixel;
  this->cam_->uv2pixelCoords( this->uv_, pixel);
  float invdepth_val = cam_->grountruth_camera_->invdepth_map_->evalPixel(pixel);
  float invdepth_gt = invdepth_val/cam_->cam_parameters_->min_depth;
  this->invdepth_=invdepth_gt;

}

int RegionsWithCandidatesBase::getNRegionsX(CameraForMapping* cam, int level){
  return cam->cam_parameters_->resolution_x/pow(2,level);
}
int RegionsWithCandidatesBase::getNRegionsY(CameraForMapping* cam, int level){
  return cam->cam_parameters_->resolution_y/pow(2,level);
}

bool RegionWithCandidates::collectCandidates(int wavelet_levels){

  int wav_levels= wavelet_levels;

  float min_depth = cam_->cam_parameters_->min_depth;
  float max_depth = cam_->cam_parameters_->max_depth;

  auto lb_cmp = [](Candidate* const & x, float d) -> bool
    { return x->grad_magnitude_ < d; };

  // from lower to higher resolution
  for (int wav_level=wav_levels-1; wav_level>=0; wav_level--){
    int n_pxls = pow(2,reg_level_-wav_level);
    int x_ref = x_*n_pxls;
    int y_ref = y_*n_pxls;
    Wvlt_lvl* wvlt_lvl= cam_->wavelet_dec_->vector_wavelets->at(wav_level);


    for (int x_offs=0; x_offs<n_pxls; x_offs++){
      for (int y_offs=0; y_offs<n_pxls; y_offs++){
        // x_ref+x_offs

        int y_curr=y_ref+y_offs;
        int x_curr=x_ref+x_offs;

        pixelIntensity c = wvlt_lvl->c->evalPixel(y_curr,x_curr);
        pixelIntensity c_dx = wvlt_lvl->c_dx->evalPixel(y_curr,x_curr);
        pixelIntensity c_dy = wvlt_lvl->c_dy->evalPixel(y_curr,x_curr);
        float magnitude = wvlt_lvl->magn_cd->evalPixel(y_curr,x_curr);
        pixelIntensity magn_cd_dx = wvlt_lvl->magn_cd_dx->evalPixel(y_curr,x_curr);
        pixelIntensity magn_cd_dy = wvlt_lvl->magn_cd_dy->evalPixel(y_curr,x_curr);
        float phase_cd = wvlt_lvl->phase_cd->evalPixel(y_curr,x_curr);
        pixelIntensity phase_cd_dx = wvlt_lvl->phase_cd_dx->evalPixel(y_curr,x_curr);
        pixelIntensity phase_cd_dy = wvlt_lvl->phase_cd_dy->evalPixel(y_curr,x_curr);

        pxl pixel_coords{x_curr,y_curr};
        Eigen::Vector2f uv;
        cam_->pixelCoords2uv(pixel_coords, uv, wav_level);

        // check well TODO
        magnitude*=pow(0.8,wav_level);

        if(magnitude>grad_threshold_){

          bound bound_(min_depth,max_depth);
          std::vector<bound>* bounds = new std::vector<bound>{ bound_ };

          Candidate* candidate = new Candidate(wav_level,pixel_coords,uv,cam_,
                                                c,
                                                magnitude,
                                                phase_cd,
                                                // c_dx, c_dy,
                                                // magn_cd_dx, magn_cd_dy,
                                                // phase_cd_dx, phase_cd_dy,
                                                bounds, this );

          // // add children
          // for(Candidate* cand : *cands_vec_){
          //   //  if older cand is at an higher level ...
          //   int level_diff = cand->level_-wav_level;
          //   // if(level_diff>0){
          //     // ... and contains current candidate
          //     // if( (x_curr/pow(2,level_diff))==cand->pixel_.x() &&
          //     //     (y_curr/pow(2,level_diff))==cand->pixel_.y()){
          //       cand->children_->push_back(candidate);
          //       candidate->children_->push_back(cand);
          //     // }
          //   // }
          // }



          auto it = std::lower_bound(cands_vec_->begin(), cands_vec_->end(), magnitude, lb_cmp);

          cands_vec_->insert ( it , candidate );
        }
      }
    }

  }
  if (cands_vec_->empty())
    return 0;
  // std::cout << "COLLECTED CANDS: " << cands_vec_->size() << std::endl;
  return 1;

}

// void RegionWithCandidatesBase::showRegion(int size){
//
//   double alpha = 0.3;
//
//   Image<pixelIntensity>* show_img = new Image<pixelIntensity>(cam_->image_intensity_);
//
//   int wav_levels= cam_->wavelet_dec_->levels_;
//   std::vector<pixelIntensity> color_map{black,red_,green_,blue_,magenta_,cyan_};
//
//   for (int wav_level=wav_levels-1; wav_level>=0; wav_level--){
//     int n_pxls = pow(2,reg_level_-wav_level);
//     int x_ref = x_*n_pxls;
//     int y_ref = y_*n_pxls;
//
//     // compute corners
//     cv::Rect r= cv::Rect(x_ref*pow(2,wav_level+1),y_*pow(2,wav_level+1),pow(2,wav_level+1),pow(2,wav_level+1));
//
//     show_img->drawRectangle(r, color_map[wav_level], cv::FILLED, alpha);
//     // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);
//
//   }
//
//
//   show_img->show(size);
//   // selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);
//
// }

void ActivePoint::marginalize(){
  // remove point from vector of active points
  std::vector<ActivePoint*>* v1 = cam_->active_points_;
  v1->erase(std::remove(v1->begin(), v1->end(), this), v1->end());

  // add point to marginalized points vector
  std::vector<ActivePoint*>* v2 = cam_->marginalized_points_;
  v2->push_back(this);

}


void CameraForMapping::selectNewCandidates(int max_num_candidates){
  int idx=0;
  float alpha=1;
  std::vector<RegionWithCandidates*>* region_vec = regions_sampling_->region_vec_;

  while(n_candidates_<max_num_candidates){
    if(!region_vec->empty()){
      for(int i=0; i<region_vec->size(); i++){
        // if (n_candidates_>=max_num_candidates)
        //   break;

        RegionWithCandidates* region= region_vec->at(i);
        std::vector<Candidate*>* cands_vec = region->cands_vec_;

        // if there are no more candidates, remove the region
        int cand_vec_size = cands_vec->size();
        // if (cands_vec->size()<1+idx){
        if (cand_vec_size-1-idx<0){
          region_vec->erase (region_vec->begin() + i);
        }
        // otherwise
        else{

          Candidate* candidate = cands_vec->at(cand_vec_size-1-idx);
          if(idx==0){
            // for( int m=0; m<cands_vec->size(); m++){
            //   if(std::find(candidate->children_->begin(), candidate->children_->end(), cands_vec->at(m)) != candidate->children_->end()) {
            //       /* children contains candidate */
            //       cands_vec->erase (cands_vec->begin() + m);
            //       // delete child_cand;
            //   }
            //
            // }

            // for( Candidate* child_cand : *(candidate->children_)){
            //   remove(cands_vec->begin(), cands_vec->end(), child_cand);
            //   // delete child_cand;
            // }

            // push back best candidate
            candidates_->push_back(candidate);
            // remove children
            // std::cout << "Ao " << candidate->children_->size() << std::endl;

            // delete candidate->children_;

            n_candidates_++;
          }
          else{
            // if (cands_vec->size()>1)

            // Candidate* candidate_prev = cands_vec->back();
            Candidate* candidate_prev = cands_vec->at(cand_vec_size-1);
            // if (candidate_prev->pixel_==candidate->pixel_ &&
            //     candidate_prev->level_==candidate->level_)
            // Candidate* candidate = region->back();
            // region->pop_back();
            if(candidate->grad_magnitude_>candidate_prev->grad_magnitude_*alpha){
              candidates_->push_back(candidate);
              n_candidates_++;
            }
          }
        }

      }
      idx++;
      alpha*=0.95;
      // break;
    }
    else
      break;

  }
}




colorRGB CameraForMapping::invdepthToRgb(float invdepth){

  float min_depth = cam_parameters_->min_depth;
  float invdepth_normalized = min_depth*invdepth;

  float H = 230*(1-invdepth_normalized);

  float s = 1;
  float v = 0.7;
  float C = s*v;
  float X = C*(1-abs(fmod(H/60.0, 2)-1));
  float m = v-C;
  float r,g,b;
  if(H >= 0 && H < 60){
      r = C,g = X,b = 0;
  }
  else if(H >= 60 && H < 120){
      r = X,g = C,b = 0;
  }
  else if(H >= 120 && H < 180){
      r = 0,g = C,b = X;
  }
  else if(H >= 180 && H < 240){
      r = 0,g = X,b = C;
  }
  else if(H >= 240 && H < 300){
      r = X,g = 0,b = C;
  }
  else{
      r = C,g = 0,b = X;
  }
  int R = (r+m);
  int G = (g+m);
  int B = (b+m);

  colorRGB color = colorRGB(b,g,r);
  return color;
}

void CameraForMapping::showCandidates(float size){

  // std::vector<colorRGB> color_map{black,red_,green_,blue_,magenta_,cyan_};

  // double alpha = 0.75;
  double alpha = 1;

  std::string name = name_+" , "+std::to_string(candidates_->size())+" candidates";
  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg(name);
  // Image<colorRGB>* show_img = new Image<colorRGB>(image_intensity_);

  for(Candidate* candidate : *candidates_){
    // get level
    int level = candidate->level_;

    pxl pixel= candidate->pixel_;
    pixel*=pow(2,level+1);

    // compute corners
    cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level+1),pow(2,level+1));

    colorRGB color = black;
    if(candidate->one_min_)
      color = invdepthToRgb(candidate->invdepth_);

    show_img->drawRectangle(r, color, cv::FILLED, alpha);
    // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);
  }

  show_img->show(size);
  // selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);

}


void CameraForMapping::showCoarseCandidates(int level, float size){

  // std::vector<colorRGB> color_map{black,red,green,blue,magenta,cyan};

  double alpha = 1;

  int num_coarse_cands=candidates_coarse_->at(level-1)->size();

  std::string name = name_+" , "+std::to_string(num_coarse_cands)+" coarse candidates, level "+std::to_string(level);
  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg(name);
  // Image<colorRGB>* show_img = new Image<colorRGB>(image_intensity_);

  // for(std::vector<Candidate*>* v : *candidates_coarse_){
  //   int i =0;
  for(Candidate* candidate : *(candidates_coarse_->at(level-1))){

    // get level
    // int level = candidate->level_;

    pxl pixel= candidate->pixel_;
    pixel*=pow(2,level+1);

    // compute corners
    cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level+1),pow(2,level+1));

    colorRGB color = black;
    if(candidate->one_min_)
      color = invdepthToRgb(candidate->invdepth_);

    show_img->drawRectangle(r, color, cv::FILLED, alpha);
    // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);

  }
  // }

  show_img->show(size);
  // selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);

}


void CameraForMapping::showCoarseActivePoints(int level, float size){

  // std::vector<colorRGB> color_map{black,red,green,blue,magenta,cyan};

  double alpha = 1;

  int num_coarse_active_points=active_points_coarse_->at(level-1)->size();

  std::string name = name_+" , "+std::to_string(num_coarse_active_points)+" coarse active points, level "+std::to_string(level);
  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg(name);
  // Image<colorRGB>* show_img = new Image<colorRGB>(image_intensity_);

  // for(std::vector<Candidate*>* v : *candidates_coarse_){
  //   int i =0;

  for(ActivePoint* active_point : *(active_points_coarse_->at(level-1))){

    // get level
    // int level = candidate->level_;

    pxl pixel= active_point->pixel_;
    pixel*=pow(2,level+1);

    // compute corners
    cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level+1),pow(2,level+1));

    colorRGB color = black;

    color = invdepthToRgb(active_point->invdepth_);

    show_img->drawRectangle(r, color, cv::FILLED, alpha);
    // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);

  }
  // }

  show_img->show(size);
  // selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);

}


void CameraForMapping::showProjCandidates(float size){

  double alpha = 1;

  int n_proj_cands=0;

  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg("proj cand temp");

  for (RegionWithProjCandidates* reg : *(regions_projected_cands_->region_vec_)){

    for (CandidateProjected* cand_proj : *(reg->cands_proj_vec_)){
      n_proj_cands++;

      // get level
      int level = cand_proj->level_;

      pxl pixel= cand_proj->pixel_;
      pixel*=pow(2,level+1);

      std::string name = "" ;

      if (pixel.y()>=(1) && pixel.y()<(float)image_intensity_->image_.rows-(1) && pixel.x()>=(1) && pixel.x()<image_intensity_->image_.cols-(1))
      {

        // compute corners
        cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level+1),pow(2,level+1));

        colorRGB color = invdepthToRgb(cand_proj->invdepth_);

        show_img->drawRectangle(r, color, cv::FILLED, alpha);
        // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);

      }

    }
  }
  show_img->show(size, name_+", n projected cands: "+std::to_string(n_proj_cands));

}

void CameraForMapping::showActivePoints(float size){
  double alpha = 1;

  int n_proj_active_pt=0;

  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg("proj active point temp");

  for(ActivePoint* active_pt : *active_points_){
    n_proj_active_pt++;
    // get level
    int level = active_pt->level_;

    pxl pixel= active_pt->pixel_;
    pixel*=pow(2,level+1);
    std::string name = "" ;

    if (pixel.y()>=(1) && pixel.y()<(float)image_intensity_->image_.rows-(1) && pixel.x()>=(1) && pixel.x()<image_intensity_->image_.cols-(1))
    {
      // compute corners
      cv::Rect r= cv::Rect((int)pixel.x(),(int)pixel.y(),pow(2,level+1),pow(2,level+1));

      colorRGB color = invdepthToRgb(active_pt->invdepth_);

      show_img->drawRectangle(r, color, cv::FILLED, alpha);
      // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);

    }
  }
  show_img->show(size, name_+", n projected active points: "+std::to_string(n_proj_active_pt));

}

void CameraForMapping::clearProjectedActivePoints(){


  for (RegionWithProjActivePoints* reg : *(regions_projected_active_points_->region_vec_)){
    reg->active_pts_proj_vec_->clear();
  }
  for (ActivePointProjected* active_pt_proj : *(regions_projected_active_points_->active_points_proj_)){
    delete active_pt_proj;
  }
  regions_projected_active_points_->active_points_proj_->clear();
}


void CameraForMapping::showProjActivePoints(float size){
  double alpha = 1;

  int n_proj_active_pt=0;

  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg("proj active point temp");

  for (RegionWithProjActivePoints* reg : *(regions_projected_active_points_->region_vec_)){

    for (ActivePointProjected* active_pt_proj : *(reg->active_pts_proj_vec_)){

      n_proj_active_pt++;
      // get level
      int level = active_pt_proj->level_;

      pxl pixel= active_pt_proj->pixel_;
      pixel*=pow(2,level+1);
      std::string name = "" ;

      if (pixel.y()>=(1) && pixel.y()<(float)image_intensity_->image_.rows-(1) && pixel.x()>=(1) && pixel.x()<image_intensity_->image_.cols-(1))
      {
        // compute corners
        cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level+1),pow(2,level+1));

        colorRGB color = invdepthToRgb(active_pt_proj->invdepth_);

        show_img->drawRectangle(r, color, cv::FILLED, alpha);
        // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);
      }
    }
  }


  // show_img->show(size, name_+", n projected active points: "+std::to_string(n_proj_active_pt));
  show_img->show(size, name_+", projected active points");

}

// assign
void CameraForMapping::assignPose0(Eigen::Isometry3f& frame_camera_wrt_world){
  *frame_camera_wrt_world_0_=frame_camera_wrt_world;
  *frame_world_wrt_camera_0_=frame_camera_wrt_world.inverse();
}

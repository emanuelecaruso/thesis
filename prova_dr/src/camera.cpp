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
void Camera::sampleRandomPixel(Eigen::Vector2i& pixel_coords){
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

void Camera::getCentreAsPixel(Eigen::Vector2i& pixel_coords) const{
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

void Camera::pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv, int level) const {

  int resolution_x=cam_parameters_->resolution_x/(pow(2,level+1));
  int resolution_y=cam_parameters_->resolution_y/(pow(2,level+1));

  float pixel_width = cam_parameters_->width/(float)resolution_x;

  uv.x()=((float)pixel_coords.x()/(float)resolution_x)*cam_parameters_->width+(pixel_width/2);
  uv.y()=((float)pixel_coords.y()/(float)resolution_y)*cam_parameters_->height+(pixel_width/2);
}

void Camera::pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv) const {
  pixelCoords2uv( pixel_coords, uv, -1);
}

void Camera::uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords, int level) const {

  int resolution_x=cam_parameters_->resolution_x/(pow(2,level+1));
  int resolution_y=cam_parameters_->resolution_y/(pow(2,level+1));

  pixel_coords.x()=(int)((uv.x()/cam_parameters_->width)*resolution_x);
  pixel_coords.y()=(int)((uv.y()/cam_parameters_->height)*resolution_y);
}

void Camera::uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords) const {
uv2pixelCoords(uv, pixel_coords, -1);
}


void Camera::pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const {

  Eigen::Vector3f p_proj;
  Eigen::Vector2f product = uv * depth;
  p_proj.x() = product.x();
  p_proj.y() = product.y();
  p_proj.z() = depth;
  Eigen::Vector3f p_cam = (*Kinv_)*p_proj;
  p = *frame_camera_wrt_world_*p_cam;

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

bool Camera::projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p = cam_to_be_projected->frame_camera_wrt_world_->translation();

  bool out = projectPoint(p, uv);
  return out;

}

bool Camera::projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv, float& p_cam_z  ) const {

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

        float magnitude = wvlt_lvl->magnitude_img->evalPixel(y_curr,x_curr);
        pixelIntensity dh = wvlt_lvl->dh->evalPixel(y_curr,x_curr);
        pixelIntensity dv = wvlt_lvl->dv->evalPixel(y_curr,x_curr);
        pixelIntensity c = wvlt_lvl->c->evalPixel(y_curr,x_curr);

        Eigen::Vector2i pixel_coords{x_curr,y_curr};
        Eigen::Vector2f uv;
        cam_->pixelCoords2uv(pixel_coords, uv, wav_level);

        // check well TODO
        magnitude*=pow(0.8,wav_level);

        if(magnitude>grad_threshold_){

          bound bound_(min_depth,max_depth);
          std::vector<bound>* bounds = new std::vector<bound>{ bound_ };

          Candidate* candidate = new Candidate(wav_level,pixel_coords,uv,magnitude,
            dh, dv, c, bounds, this );

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


void CameraForMapping::selectNewCandidates(int max_num_candidates){
  int idx=0;
  float alpha=1;
  std::vector<RegionWithCandidates*>* region_vec = regions_->region_vec_;

  while(n_candidates_<max_num_candidates){
    if(!region_vec->empty()){
      for(int i=0; i<region_vec->size(); i++){
        if (n_candidates_>=max_num_candidates)
          break;

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


void CameraForMapping::selectActivePoints(int max_num_active_points){

}


void CameraForMapping::showCandidates_1(float size){
  // Wvlt_dec* selected = new Wvlt_dec(wavelet_dec_);
  //
  // for(Candidate* candidate : *candidates_){
  //
  //   int levels= wavelet_dec_->levels_;
  //   int level = candidate->level_;
  //   int lev_opp=levels-1-level;
  //
  //   Eigen::Vector2i pixel_coords=candidate->pixel_;
  //
  //   Wvlt_lvl* wvlt_curr_=selected->vector_wavelets->at(level);
  //   // wvlt_curr_->dh->setPixel(pixel_coords,white*8);
  //   // wvlt_curr_->dv->setPixel(pixel_coords,white*8);
  //   wvlt_curr_->c->setPixel(pixel_coords,white*8);
  // }
  // selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);

}

void CameraForMapping::showCandidates_2(float size){

  std::vector<colorRGB> color_map{black,red_,green_,blue_,magenta_,cyan_};

  double alpha = 0.5;

  std::string name = std::to_string(candidates_->size())+" candidates";
  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg(name);
  // Image<colorRGB>* show_img = new Image<colorRGB>(image_intensity_);

  for(Candidate* candidate : *candidates_){
    // get level
    int level = candidate->level_;

    Eigen::Vector2i pixel= candidate->pixel_;
    pixel*=pow(2,level+1);

    // compute corners
    cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level+1),pow(2,level+1));

    show_img->drawRectangle(r, color_map[level], cv::FILLED, alpha);
    // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);
  }

  show_img->show(size);
  // selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);

}


void CameraForMapping::showProjCandidates_2(float size){

  double alpha = 0.5;

  int n_proj_cands=0;

  Image<colorRGB>* show_img = image_intensity_->returnColoredImgFromIntensityImg("proj cand temp");

  for (RegionWithProjCandidates* reg : *(regions_projected_cands_->region_vec_)){
    for (CandidateProjected* cand : *(reg->cands_vec_)){
      n_proj_cands++;

      // get level
      int level = cand->level_;

      Eigen::Vector2i pixel= cand->pixel_;
      pixel*=pow(2,level+1);
      std::string name = "" ;

      // compute corners
      cv::Rect r= cv::Rect(pixel.x(),pixel.y(),pow(2,level+1),pow(2,level+1));

      show_img->drawRectangle(r, black, cv::FILLED, alpha);
      // show_img->drawRectangle(r, color_map[level], cv::LINE_8, alpha);

    }
  }
  show_img->show(size, "n projected cands: "+std::to_string(n_proj_cands));

}

void CameraForMapping::showActivePoints_2(float size){
  double alpha = 0.5;

  int n_proj_cands=0;
  Image<pixelIntensity>* show_img = new Image<pixelIntensity>(image_intensity_);
}

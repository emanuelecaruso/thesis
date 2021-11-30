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
void Camera::getCenterAsUV(Eigen::Vector2f& uv){
  uv.x() = cam_parameters_->width/2;
  uv.y() = cam_parameters_->height/2;
}

void Camera::getCentreAsPixel(Eigen::Vector2i& pixel_coords){
  pixel_coords.x() = cam_parameters_->resolution_x/2;
  pixel_coords.y() = cam_parameters_->resolution_y/2;
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
      lens ,   0   ,  -width/2,
      0    ,  -lens, -height/2,
      0    ,   0   ,       -1 ;

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

  // return wether the projected point is in front or behind the camera
  p_cam_z=-p_cam.z();

  Eigen::Vector3f p_proj = (*K_)*p_cam;

  uv = p_proj.head<2>()*(1./p_proj.z());

  if (p_cam_z<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p_cam = *frame_world_wrt_camera_*p;

  Eigen::Vector3f p_proj = (*K_)*p_cam;

  uv = p_proj.head<2>()*(1./p_proj.z());

  if (-p_cam.z()<cam_parameters_->lens)
    return false;

  return true;
}

bool Camera::projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv ) const {

  Eigen::Vector3f p = cam_to_be_projected->frame_camera_wrt_world_->translation();

  bool out = projectPoint(p, uv);
  return out;

}

void Camera::saveRGB(const std::string& path) const {
  cv::imwrite(path+ "/rgb_" +name_+".png", image_rgb_->image_);
}

void Camera::saveDepthMap(const std::string& path) const {
  cv::Mat ucharImg;
  invdepth_map_->image_.convertTo(ucharImg, CV_32FC1, 255.0);
  cv::imwrite(path+ "/depth_" +name_+".png", ucharImg);

}

Image<colorRGB>* Camera::returnRGBFromPath(const std::string& path_rgb){

  Image<cv::Vec3b>* img_uc = new Image<cv::Vec3b>(name_);
  Image<colorRGB>* img = new Image<colorRGB>(name_);
  img_uc->image_=cv::imread(path_rgb);
  img_uc->image_.convertTo(img->image_, colorRGB_CODE, colorRGB_maxval/255.0);
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
  image_rgb_->show(image_scale);
}

void Camera::showDepthMap(int image_scale) const {
  invdepth_map_->show(image_scale);
}




Image<colorRGB>* CameraForStudy::computeCurvature(float gain){

  Image<colorRGB>* img = new Image<colorRGB>(image_rgb_);

  Image<colorRGB>* fx = img->compute_sobel_x();
  Image<colorRGB>* fx_sqrd = fx->squared();
  Image<colorRGB>* fxx = fx->compute_sobel_x();
  Image<colorRGB>* fy = img->compute_sobel_y();
  Image<colorRGB>* fy_sqrd = fy->squared();
  Image<colorRGB>* fyy = fy->compute_sobel_y();
  Image<colorRGB>* fxy = fx->compute_sobel_y();

  // curvature
  Image<colorRGB>* k = new Image<colorRGB>("curvature_"+name_);
  // curvature -> κ = fy^2 fxx − 2*fx fy fxy + fx^2 fyy ,
  k->image_=fy_sqrd->image_.mul(fxx->image_)-2*fx->image_.mul(fy->image_.mul(fxy->image_))+fx_sqrd->image_.mul(fyy->image_);
  // k->image_/=(image_rgb_->image_.mul(image_rgb_->image_.mul(image_rgb_->image_)));
  k->image_*=gain;

  return k;
}



Image<colorRGB>* CameraForStudy::gradientX(){

  Image<colorRGB>* img = new Image<colorRGB>(image_rgb_);

  Image<colorRGB>* fx = img->compute_sobel_x();
  Image<colorRGB>* fxx = fx->compute_sobel_x();
  Image<colorRGB>* fxxx = fxx->compute_sobel_x();

  Image<colorRGB>* out = fx;

  std::vector<cv::Mat> channels_i(3);
  split(out->image_, channels_i);
  // channels_i[2]*=1.0/(4);
  // channels_i[1]*=1.0/(4);
  // channels_i[0]*=1.0/(4);
  merge(channels_i, out->image_);


  return out;
  // return fx;
}

Image<colorRGB>* CameraForStudy::gradientY(){

  Image<colorRGB>* img = new Image<colorRGB>(image_rgb_);

  Image<colorRGB>* fy = img->compute_sobel_y();
  Image<colorRGB>* fyy = fy->compute_sobel_y();
  Image<colorRGB>* fyyy = fyy->compute_sobel_y();

  Image<colorRGB>* out = fy;

  std::vector<cv::Mat> channels_i(3);
  split(out->image_, channels_i);
  // channels_i[2]*=1.0/(4);
  // channels_i[1]*=1.0/(4);
  // channels_i[0]*=1.0/(4);
  merge(channels_i, out->image_);

  return out;
  // return fy;
}

Image<colorRGB>* CameraForStudy::gradientRobustX(){

  Image<colorRGB>* img = new Image<colorRGB>(image_rgb_);

  float offset=0.1;
  img->image_+=cv::Scalar(offset,offset,offset);

  Image<colorRGB>* fx = img->compute_sobel_x();

  Image<colorRGB>* fxr = new Image<colorRGB>("grad_x_rob_"+name_);

  fxr->image_=fx->image_/(img->image_);
  // fxr->image_*=5;

  return fxr;
}

Image<float>* CameraForStudy::gradientintensity(){

  Image<colorRGB>* img = new Image<colorRGB>(image_rgb_);

  Image<colorRGB>* fx = img->compute_sobel_x();
  Image<colorRGB>* fy = img->compute_sobel_y();
  Image<colorRGB>* fx_sqrd = fx->squared();
  Image<colorRGB>* fy_sqrd = fy->squared();

  Image<colorRGB>* f = new Image<colorRGB>();
  Image<float>* out = new Image<float>("grad_intensity_"+name_);
  // curvature -> κ = fy^2 fxx − 2*fx fy fxy + fx^2 fyy ,
  f->image_=(1./8.)*(fx_sqrd->image_+fy_sqrd->image_);
  out=f->getComponentSum();

  return out;
}


void CameraForMapping::collectRegions(float grad_threshold){
  Wvlt_lvl* wvlt_last_lvl= wavelet_dec_->vector_wavelets->back();

  int levels= wavelet_dec_->levels_;
  int rows=wvlt_last_lvl->c->image_.rows;
  int cols=wvlt_last_lvl->c->image_.cols;

  float min_depth = cam_parameters_->min_depth;
  float max_depth = cam_parameters_->max_depth;

  auto lb_cmp = [](Candidate* const & x, float d) -> bool
    { return x->grad_magnitude_ < d; };

  for (int row=0; row<rows; row++){
    for (int col=0; col<cols; col++){
      Region* reg= new Region;
      for (int level=levels-1; level>=0; level--){
        int lev_opp=levels-1-level;
        for (int row_offs=0; row_offs<(pow(2,lev_opp)); row_offs++){
          for (int col_offs=0; col_offs<(pow(2,lev_opp)); col_offs++){

            Wvlt_lvl* wvlt_lvl= wavelet_dec_->vector_wavelets->at(level);
            // int row_curr=(row<<lev_opp)+row_offs;
            // int col_curr=(col<<lev_opp)+col_offs;
            int row_curr=(row*(pow(2,lev_opp)))+row_offs;
            int col_curr=(col*(pow(2,lev_opp)))+col_offs;

            float magnitude = wvlt_lvl->magnitude_img->evalPixel(row_curr,col_curr);
            colorRGB magnitude3C = wvlt_lvl->magnitude3C_img->evalPixel(row_curr,col_curr);
            colorRGB c = wvlt_lvl->c->evalPixel(row_curr,col_curr);

            Eigen::Vector2i pixel_coords{col_curr,row_curr};
            Eigen::Vector2f uv;
            pixelCoords2uv(pixel_coords, uv, level);

            if(magnitude>grad_threshold){
              std::cout << name_ << ": "<< magnitude << std::endl;
              Candidate* candidate = new Candidate(level,pixel_coords,uv,magnitude,
                                                   min_depth, max_depth,magnitude3C, c  );

              // std::vector<bound>* bounds = new std::vector<bound>{ ( min_depth, max_depth ) };
              // Candidate* candidate = new Candidate(level,pixel_coords,uv,magnitude,
              //                                      ,magnitude3C, c, bounds );

              auto it = std::lower_bound(reg->begin(), reg->end(), magnitude, lb_cmp);

              reg->insert ( it , candidate );
            }

          }
        }
      }
      if(!reg->empty()){
        regions_->push_back(reg);
      }
    }
  }

}


void CameraForMapping::selectNewCandidates(int max_num_candidates){
  int idx=0;
  float alpha=1;
  while(n_candidates_<max_num_candidates){
    if(!regions_->empty()){
      for(int i=0; i<regions_->size(); i++){
        if (n_candidates_>=max_num_candidates)
          break;

        Region* region= regions_->at(i);

        if (1+idx>region->size()){
          regions_->erase (regions_->begin() + i);
        }
        else{
          Candidate* candidate = region->at(region->size()-1-idx);
          if(idx==0){
            candidates_->push_back(candidate);
            n_candidates_++;
          }else{
            Candidate* candidate_prev = region->back();
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
      alpha*=0.9;
      // break;
    }
    else
      break;

  }
}

void CameraForMapping::showCandidates_1(float size){
  Wvlt_dec* selected = new Wvlt_dec(wavelet_dec_);

  for(Candidate* candidate : *candidates_){

    int levels= wavelet_dec_->levels_;
    int level = candidate->level_;
    int lev_opp=levels-1-level;

    Eigen::Vector2i pixel_coords=candidate->pixel_;

    Wvlt_lvl* wvlt_curr_=selected->vector_wavelets->at(level);
    // wvlt_curr_->dh->setPixel(pixel_coords,white*8);
    // wvlt_curr_->dv->setPixel(pixel_coords,white*8);
    wvlt_curr_->magnitude3C_img->setPixel(pixel_coords,white*8);
  }
  selected->showWaveletDec(std::to_string(n_candidates_)+" candidates",size);

}

void CameraForMapping::showCandidates_2(float size){

  std::vector<colorRGB> color_map{black,red_,green_,blue_,magenta_,cyan_};

  double alpha = 0.5;

  Image<colorRGB>* show_img = new Image<colorRGB>(wavelet_dec_->image_);
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

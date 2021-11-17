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
  // image_rgb_->setAllPixels(cv::Vec3b(255,255,255));
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

void Camera::pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv) const {
  float pixel_width = cam_parameters_->width/cam_parameters_->resolution_x;

  uv.x()=((float)pixel_coords.x()/(cam_parameters_->resolution_x))*cam_parameters_->width+(pixel_width/2);
  uv.y()=(((float)pixel_coords.y())/(float)cam_parameters_->resolution_y)*(float)(cam_parameters_->height)+(pixel_width/2);
}

void Camera::uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords) const {

  pixel_coords.x()=(int)((uv.x()/cam_parameters_->width)*cam_parameters_->resolution_x);
  pixel_coords.y()=(int)((uv.y()/cam_parameters_->height)*cam_parameters_->resolution_y);
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

Image<cv::Vec3b>* Camera::returnRGBFromPath(const std::string& path_rgb){

  Image<cv::Vec3b>* img = new Image<cv::Vec3b>(name_);
  img->image_=cv::imread(path_rgb);
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

Image<cv::Vec3b>* Camera::setPixelAndReturnImg(const Eigen::Vector2i& pixel_coords,
                                               const std::string& name){

  Image<cv::Vec3b>* img = image_rgb_->clone(image_rgb_->name_);
  return img;
}




Image<cv::Vec3f>* CameraForStudy::computeCurvature(float gain){

  Image<cv::Vec3f>* img = new Image<cv::Vec3f>();
  image_rgb_->image_.convertTo(img->image_, CV_32FC3, 1/255.0);

  Image<cv::Vec3f>* fx = img->compute_sobel_x();
  Image<cv::Vec3f>* fx_sqrd = fx->squared();
  Image<cv::Vec3f>* fxx = fx->compute_sobel_x();
  Image<cv::Vec3f>* fy = img->compute_sobel_y();
  Image<cv::Vec3f>* fy_sqrd = fy->squared();
  Image<cv::Vec3f>* fyy = fy->compute_sobel_y();
  Image<cv::Vec3f>* fxy = fx->compute_sobel_y();

  // curvature
  Image<cv::Vec3f>* k = new Image<cv::Vec3f>("curvature_"+name_);
  // curvature -> κ = fy^2 fxx − 2*fx fy fxy + fx^2 fyy ,
  k->image_=fy_sqrd->image_.mul(fxx->image_)-2*fx->image_.mul(fy->image_.mul(fxy->image_))+fx_sqrd->image_.mul(fyy->image_);
  // k->image_/=(image_rgb_->image_.mul(image_rgb_->image_.mul(image_rgb_->image_)));
  k->image_*=gain;

  return k;
}



Image<cv::Vec3f>* CameraForStudy::gradientX(){

  Image<cv::Vec3f>* img = new Image<cv::Vec3f>("grad_x_"+name_);
  image_rgb_->image_.convertTo(img->image_, CV_32FC3, 1/255.0);

  Image<cv::Vec3f>* fx = img->compute_sobel_x();
  Image<cv::Vec3f>* fxx = fx->compute_sobel_x();
  Image<cv::Vec3f>* fxxx = fxx->compute_sobel_x();

  Image<cv::Vec3f>* out = fx;

  std::vector<cv::Mat> channels_i(3);
  split(out->image_, channels_i);
  // channels_i[2]*=1.0/(4);
  // channels_i[1]*=1.0/(4);
  // channels_i[0]*=1.0/(4);
  merge(channels_i, out->image_);


  return out;
  // return fx;
}

Image<cv::Vec3f>* CameraForStudy::gradientY(){

  Image<cv::Vec3f>* img = new Image<cv::Vec3f>("grad_y_"+name_);
  image_rgb_->image_.convertTo(img->image_, CV_32FC3, 1/255.0);

  Image<cv::Vec3f>* fy = img->compute_sobel_y();
  Image<cv::Vec3f>* fyy = fy->compute_sobel_y();
  Image<cv::Vec3f>* fyyy = fyy->compute_sobel_y();

  Image<cv::Vec3f>* out = fy;

  std::vector<cv::Mat> channels_i(3);
  split(out->image_, channels_i);
  // channels_i[2]*=1.0/(4);
  // channels_i[1]*=1.0/(4);
  // channels_i[0]*=1.0/(4);
  merge(channels_i, out->image_);

  return out;
  // return fy;
}

Image<cv::Vec3f>* CameraForStudy::gradientRobustX(){

  Image<cv::Vec3f>* img = new Image<cv::Vec3f>();
  image_rgb_->image_.convertTo(img->image_, CV_32FC3, 1/255.0);

  float offset=0.1;
  img->image_+=cv::Scalar(offset,offset,offset);

  Image<cv::Vec3f>* fx = img->compute_sobel_x();

  Image<cv::Vec3f>* fxr = new Image<cv::Vec3f>("grad_x_rob_"+name_);

  fxr->image_=fx->image_/(img->image_);
  // fxr->image_*=5;

  return fxr;
}

Image<float>* CameraForStudy::gradientintensity(){

  Image<cv::Vec3f>* img = new Image<cv::Vec3f>();
  image_rgb_->image_.convertTo(img->image_, CV_32FC3, 1/255.0);

  Image<cv::Vec3f>* fx = img->compute_sobel_x();
  Image<cv::Vec3f>* fy = img->compute_sobel_y();
  Image<cv::Vec3f>* fx_sqrd = fx->squared();
  Image<cv::Vec3f>* fy_sqrd = fy->squared();

  Image<cv::Vec3f>* f = new Image<cv::Vec3f>();
  Image<float>* out = new Image<float>("grad_intensity_"+name_);
  // curvature -> κ = fy^2 fxx − 2*fx fy fxy + fx^2 fyy ,
  f->image_=(1./8.)*(fx_sqrd->image_+fy_sqrd->image_);
  out=f->getComponentSum();

  return out;
}


void CameraForMapping::collectRegions(float threshold){
  Wvlt_lvl* wvlt_last_lvl= wavelet_dec_->vector_wavelets->back();

  int levels= wavelet_dec_->levels_;
  int rows=wvlt_last_lvl->c->image_.rows;
  int cols=wvlt_last_lvl->c->image_.cols;

  auto lb_cmp = [](Candidate* const & x, float d) -> bool
    { return (*x).second < d; };

  for (int row=0; row<rows; row++){
    for (int col=0; col<cols; col++){
      Region* reg= new Region;
      for (int level=levels-1; level>=0; level--){
        int lev_opp=levels-1-level;
        for (int row_offs=0; row_offs<(2<<lev_opp); row_offs++){
          for (int col_offs=0; col_offs<(2<<lev_opp); col_offs++){

            Wvlt_lvl* wvlt_lvl= wavelet_dec_->vector_wavelets->at(level);
            int row_curr=(row<<lev_opp)+row_offs;
            int col_curr=(col<<lev_opp)+col_offs;
            float norm = wvlt_lvl->norm_img->evalPixel(row_curr,col_curr);
            Eigen::Vector3i idx(row_curr,col_curr,level);
            if(norm>threshold){
              Candidate* candidate = new Candidate{idx,norm};

              auto it = std::lower_bound(reg->begin(), reg->end(), norm, lb_cmp);

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


void CameraForMapping::selectCandidates(int max_num_candidates){
  int num_candidates=0;

  while(num_candidates<max_num_candidates){
    if(!regions_->empty()){
      for(int i=regions_->size()-1; i>=0; i--){
        if (num_candidates>=max_num_candidates)
          break;

        Region* region= regions_->at(i);

        Candidate* candidate = region->back();
        region->pop_back();

        candidates_->push_back(candidate);
        if (region->empty()){
          regions_->erase (regions_->begin() + i);
        }


        num_candidates++;
      }
    }

  }
}

void CameraForMapping::showCandidates(float size){
  Wvlt_dec* selected = new Wvlt_dec(wavelet_dec_);

  int n_candidates=0;
  for(Candidate* candidate : *candidates_){
    Eigen::Vector3i idx = candidate->first;
    Wvlt_lvl* wvlt_curr_=selected->vector_wavelets->at(idx[2]);
    wvlt_curr_->dh->setPixel(idx[0],idx[1],white);
    wvlt_curr_->dv->setPixel(idx[0],idx[1],white);
    wvlt_curr_->dd->setPixel(idx[0],idx[1],white);
    n_candidates++;
  }
  selected->showWaveletDec(std::to_string(n_candidates)+" candidates",size);

}

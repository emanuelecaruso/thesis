#include "mapper.h"
#include "epline.h"
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



void EpipolarLine::lineTraverse(int level)
{

    float distance = abs(end-start);

    // float pixel_width=cam->cam_parameters_->width/(float)cam->cam_parameters_->resolution_x;
    float pixel_width=cam->cam_parameters_->width/((float)cam->cam_parameters_->resolution_x/pow(2,level+1));

    int n_uvs= (distance/pixel_width)+2;
    // int n_uvs= ((distance/pixel_width)+1)/(level+2);
    // std::cout << n_uvs << std::endl;

    uvs = new std::vector<Eigen::Vector2f>(n_uvs);

    for (int i=0; i<n_uvs; i++){
      float ratio = (float)i/(n_uvs-1);
      float coord = (1-ratio)*start+(ratio)*end;

      Eigen::Vector2f uv;
      coordToUV(coord,uv);
      uvs->at(i)= uv;
    }
}



float EpipolarLine::getCostMagn(const pixelIntensity intensity_r, const pixelIntensity intensity_m,
                                const pixelIntensity magnitude_r, const pixelIntensity magnitude_m) {
  float cost_magn = abs(magnitude_r-magnitude_m);
  float cost_intensity = abs(intensity_r-intensity_m);
  // return cost_magn;
  return cost_magn+cost_intensity;
}


std::vector<Eigen::Vector2f>* EpipolarLine::collectUvsROfDSOPattern(Candidate* cand){
  std::vector<Eigen::Vector2f>* uvs = new std::vector<Eigen::Vector2f>;
  float pixel_width = cand->cam_->cam_parameters_->width/(float)(cand->cam_->cam_parameters_->resolution_x/(pow(2,cand->level_+1)));

  // Eigen::Vector2f uv_central = cand->uv_;
  // uvs->push_back(uv_central);

  Eigen::Vector2f uv_new;
  uv_new.x() = cand->uv_.x()+pixel_width*2;
  uv_new.y() = cand->uv_.y();
  uvs->push_back(uv_new);
  uv_new.x() = cand->uv_.x()-pixel_width*2;
  uv_new.y() = cand->uv_.y();
  uvs->push_back(uv_new);
  uv_new.x() = cand->uv_.x();
  uv_new.y() = cand->uv_.y()+pixel_width*2;
  uvs->push_back(uv_new);
  uv_new.x() = cand->uv_.x();
  uv_new.y() = cand->uv_.y()-pixel_width*2;
  uvs->push_back(uv_new);
  uv_new.x() = cand->uv_.x()+pixel_width;
  uv_new.y() = cand->uv_.y()+pixel_width;
  uvs->push_back(uv_new);
  uv_new.x() = cand->uv_.x()-pixel_width;
  uv_new.y() = cand->uv_.y()+pixel_width;
  uvs->push_back(uv_new);
  uv_new.x() = cand->uv_.x()-pixel_width;
  uv_new.y() = cand->uv_.y()-pixel_width;
  uvs->push_back(uv_new);

  return uvs;
}

std::vector<pixelIntensity>* EpipolarLine::collectIntensitiesMOfDSOPattern(Candidate* cand, Eigen::Vector2f& uv_m, CamCouple* cam_couple, std::vector<pixelIntensity>* intensities_r, std::vector<pixelIntensity>* intensities_m ){

  Image<pixelIntensity>* c_r = cam_couple->cam_r_->wavelet_dec_->vector_wavelets->at(cand->level_)->c;
  Image<pixelIntensity>* c_m = cam_couple->cam_m_->wavelet_dec_->vector_wavelets->at(cand->level_)->c;

  // collect uvs r
  std::vector<Eigen::Vector2f>* uvs_r = collectUvsROfDSOPattern(cand);

  // get d1
  float d1;
  float coord;
  if(u_or_v){
    coord=uv_m.x();
  }else{
    coord=uv_m.y();
  }
  cam_couple->getD1(cand->uv_.x(),cand->uv_.y(),d1,coord,u_or_v);

  // push central pixel
  pxl pixel_central_m;
  cam_couple->cam_m_->uv2pixelCoords(uv_m,pixel_central_m,cand->level_);
  pixelIntensity intensity_central_r = c_r->evalPixelBilinear(cand->pixel_);
  pixelIntensity intensity_central_m = c_m->evalPixelBilinear(pixel_central_m);
  intensities_r->push_back(intensity_central_r);
  intensities_m->push_back(intensity_central_m);

  //project each point inside r on cam m
  for(Eigen::Vector2f uv_r : *uvs_r){
    Eigen::Vector2f uv_m;
    pxl pixel_r, pixel_m;
    cam_couple->getUv(uv_r.x(),uv_r.y(),d1,uv_m.x(),uv_m.y());
    cam_couple->cam_r_->uv2pixelCoords(uv_r,pixel_r,cand->level_);
    cam_couple->cam_m_->uv2pixelCoords(uv_m,pixel_m,cand->level_);
    pixelIntensity intensity_r = c_r->evalPixelBilinear(pixel_r);
    pixelIntensity intensity_m = c_m->evalPixelBilinear(pixel_m);
    intensities_r->push_back(intensity_r);
    intensities_m->push_back(intensity_m);
  }

}

float EpipolarLine::getCostSSD(std::vector<pixelIntensity>* intensities_r, std::vector<pixelIntensity>* intensities_m){
  float cost = 0;
  for(int i=0; i<intensities_r->size(); i++){
    cost+=pow(intensities_r->at(i)-intensities_m->at(i),2);
  }
  return cost;
}



float EpipolarLine::getCostPhase( Candidate* cand, Eigen::Vector2f& uv_m, float phase_m, CamCouple* cam_couple){

  float pixel_width=cand->cam_->cam_parameters_->width/((float)cam->cam_parameters_->resolution_x/pow(2,cand->level_+1));
  float phase_r = cand->grad_phase_;
  Eigen::Vector2f grad_direction_r {cos(phase_r)*pixel_width,sin(phase_r)*pixel_width};

  // std::cout << "\nphase r: " << phase_r << ", grad dir r:\n" << grad_direction_r << std::endl;
  Eigen::Vector2f tip_to_project = cand->uv_+grad_direction_r;
  float d1, coord;
  if(u_or_v)
    coord = uv_m.x();
  else
    coord = uv_m.y();

  cam_couple->getD1(cand->uv_.x(), cand->uv_.y(), d1, coord, u_or_v);

  Eigen::Vector2f tip_m, direction_m;

  cam_couple->getUv(tip_to_project.x(), tip_to_project.y(), d1, tip_m.x(), tip_m.y() );

  direction_m=tip_m-uv_m;

  // std::cout << "\n1:\n" << direction_m << " 2:\n " << grad_direction_r << std::endl;

  float phase_m_ = std::atan2(direction_m.y(),direction_m.x());

  if (phase_m_<0)
    phase_m_+=2*PI;
  if (phase_m_<0)
    phase_m_+=2*PI;

  // if( (phase_far < phase_m && phase_m < phase_close) || (phase_far > phase_m && phase_m > phase_close) ){
  if(abs(phase_m_)>10){
    std::cout << "MA COMEEEEEE " << phase_m_ << " " << phase_m << std::endl;
  }
  return abs(radiansSub(phase_m_,phase_m)/(2*PI));

}

bool EpipolarLine::checkPhaseInRange( Candidate* cand, Eigen::Vector2f& uv_m, float phase_m, CamCouple* cam_couple){

  float pixel_width=cand->cam_->cam_parameters_->width/((float)cam->cam_parameters_->resolution_x/pow(2,cand->level_+1));
  float phase_r = cand->grad_phase_;
  Eigen::Vector2f grad_direction_r {cos(phase_r)*pixel_width,sin(phase_r)*pixel_width};

  // std::cout << "\nphase r: " << phase_r << ", grad dir r:\n" << grad_direction_r << std::endl;
  Eigen::Vector2f tip_to_project = cand->uv_+grad_direction_r;
  float d1, coord;
  if(u_or_v)
    coord = uv_m.x();
  else
    coord = uv_m.y();

  cam_couple->getD1(cand->uv_.x(), cand->uv_.y(), d1, coord, u_or_v);

  Eigen::Vector2f tip_m, direction_m;

  cam_couple->getUv(tip_to_project.x(), tip_to_project.y(), d1, tip_m.x(), tip_m.y() );

  direction_m=tip_m-uv_m;

  // std::cout << "\n1:\n" << direction_m << " 2:\n " << grad_direction_r << std::endl;

  float phase_m_ = std::atan2(direction_m.y(),direction_m.x());

  if (phase_m_<0)
    phase_m_+=2*PI;
  if (phase_m_<0)
    phase_m_+=2*PI;

  // if( (phase_far < phase_m && phase_m < phase_close) || (phase_far > phase_m && phase_m > phase_close) ){
  if( (abs(radiansSub(phase_m_,phase_m))) < 0.1 ){
    return true;
  }
  return false;
}

bool EpipolarLine::searchMinDSO(Candidate* candidate, Params* parameters, CamCouple* cam_couple ){
  // iterate through uvs
  float prev_cost = FLT_MAX;
  int min_uv_idx;

  pixelIntensity intensity_r = candidate->intensity_;
  float magnitude_r = candidate->grad_magnitude_;

  bool sign=true;

  for(int i=0; i<uvs->size(); i++){
    Eigen::Vector2f uv = uvs->at(i);
    pxl pixel;
    cam->uv2pixelCoords(uv,pixel,candidate->level_);

    float magnitude_m;

    if(! cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magn_cd->evalPixelBilinear(pixel, magnitude_m)){
      continue;
    }

    // hard thresholding on magnitude
    // bool magnitude_under_threshold=magnitude_m<magnitude_r*parameters->grad_perc_threshold;
    bool magnitude_under_threshold=abs(magnitude_m-magnitude_r)>parameters->cost_grad_threshold_DSO;
    // std::cout << abs(magnitude_m-magnitude_r) << "\n";

    if(magnitude_under_threshold){
      // check if previous cost is a local minimum
      if(sign && prev_cost<(parameters->cost_threshold )){
        // add previous cost inside minimums
        uv_idxs_mins->push_back(i-1);
      }
      // restart search
      sign=false;
      prev_cost = FLT_MAX;
      continue;
    }

    std::vector<pixelIntensity>* intensities_r = new std::vector<pixelIntensity>;
    std::vector<pixelIntensity>* intensities_m = new std::vector<pixelIntensity>;
    collectIntensitiesMOfDSOPattern( candidate, uv, cam_couple, intensities_r, intensities_m);

    // get cost
    float cost = getCostSSD(intensities_r, intensities_m);

    if (cost<prev_cost){
      sign=true;
    }
    else{
      if(sign && prev_cost<(parameters->cost_threshold)){
        uv_idxs_mins->push_back(i-1);
      }
      sign=false;
    }
    prev_cost = cost;
  }
  // std::cout << prev_cost << std::endl;
  if(uv_idxs_mins->empty())
    return 0;
  return 1;

}



bool EpipolarLine::searchMin(Candidate* candidate, Params* parameters, CamCouple* cam_couple ){
  // iterate through uvs
  float prev_cost = FLT_MAX;
  int min_uv_idx;

  pixelIntensity intensity_r = candidate->intensity_;
  float magnitude_r = candidate->grad_magnitude_;
  float phase_r = candidate->grad_phase_;


  bool sign=true;

  for(int i=0; i<uvs->size(); i++){
    Eigen::Vector2f uv = uvs->at(i);
    pxl pixel;
    cam->uv2pixelCoords(uv,pixel,candidate->level_);

    pixelIntensity intensity_m;
    float magnitude_m;
    float phase_m;


    if(!cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->c->evalPixelBilinear(pixel, intensity_m))
      continue;

    cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magn_cd->evalPixelBilinear(pixel, magnitude_m);
    cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->phase_cd->evalPixelBilinear(pixel, phase_m);

    float cost = getCostMagn(intensity_r, intensity_m, magnitude_r, magnitude_m);


    // hard thresholding on cost
    bool rough_cost_too_high=cost>parameters->cost_threshold;

    if(rough_cost_too_high){
      // check if previous cost is good enough for a local minimum
      if(sign && prev_cost<(parameters->cost_threshold  )){
        // add previous cost inside minimums
        uv_idxs_mins->push_back(i-1);
      }
      // restart search
      sign=false;
      prev_cost = FLT_MAX;
      continue;
    }


    // get cost
    float cost_phase = getCostPhase(candidate, uv, phase_m, cam_couple);
    cost += cost_phase;
    // bool phase_in_range = true;


    if (cost<prev_cost){
      sign=true;
      prev_cost = cost;
    }
    else{
      // check if previous cost is good enough for a local minimum
      if(sign && prev_cost<(parameters->cost_threshold)){
        // add previous cost inside minimums
        uv_idxs_mins->push_back(i-1);
      }
      sign=false;
      prev_cost = FLT_MAX;
    }

  }
  if(sign && prev_cost<(parameters->cost_threshold)){
    // add previous cost inside minimums
    uv_idxs_mins->push_back(uvs->size()-1);
  }

  // std::cout << prev_cost << std::endl;
  if(uv_idxs_mins->empty())
    return 0;
  return 1;

}

float EpipolarLine::slope2angle(){
  return std::atan2(slope,1);
}

// show
void EpipolarLine::showEpipolar(int level,float size){
    Image<colorRGB>* image_intensity_new = createEpipolarImg(level);
    image_intensity_new->show(size*(pow(2,level+1)));
}

void EpipolarLine::showEpipolarWithMin(int level,float size){
    Image<colorRGB>* image_intensity_new = createEpipolarImg(level);

    if(!uv_idxs_mins->empty()){
      for (int idx : *uv_idxs_mins){
        pxl pixel;
        cam->uv2pixelCoords(uvs->at(idx),pixel,level);
        image_intensity_new->setPixel(pixel,blue);
      }
    }
    image_intensity_new->show(size*(pow(2,level+1)), cam->name_);
}

void EpipolarLine::showEpipolarComparison(EpipolarLine* ep_line_2, bool print=false, float size=1){
    Image<colorRGB>* image_intensity_new = createEpipolarImg();
    Image<colorRGB>* image_intensity_new_2 = ep_line_2->createEpipolarImg();
    image_intensity_new->showWithOtherImage(image_intensity_new_2,size);

}

void EpipolarLine::showEpipolarComparison(EpipolarLine* ep_line_2,
                          const std::string& name, bool print=false, float size=1)
{
    Image<colorRGB>* image_intensity_new = createEpipolarImg();
    Image<colorRGB>* image_intensity_new_2 = ep_line_2->createEpipolarImg();
    image_intensity_new->showWithOtherImage(image_intensity_new_2,name,size);
}

void EpipolarLine::drawEpipolar(Image<colorRGB>* img, const colorRGB& color, int level){

    if (!uvs->empty())
      for( int i=0; i<uvs->size(); i++){
        pxl pixel;

        cam->uv2pixelCoords(uvs->at(i),pixel,level);

        // if(i==0)
        //   image_intensity_new->setPixel(pixel, blue);
        // else if (i==uvs->size()-1)
        //   image_intensity_new->setPixel(pixel, red);
        // else
        img->setPixel(pixel, color);
      }

}


// create imgs to show
Image<colorRGB>* EpipolarLine::createEpipolarImg(const std::string& name, int level){

    Image<colorRGB>* image_intensity_new = new Image<colorRGB> (cam->name_) ;

    if (level==-1)
      image_intensity_new = cam->image_intensity_->returnColoredImgFromIntensityImg("epipolar") ;
    else{
      image_intensity_new =cam->wavelet_dec_->vector_wavelets->at(level)->magn_cd->returnColoredImgFromIntensityImg("epipolar") ;
      // image_intensity_new->image_/=2*PI;
    }

    drawEpipolar(image_intensity_new, green, level);

    return image_intensity_new;
}

Image<colorRGB>* EpipolarLine::createEpipolarImg(int level){
  return createEpipolarImg("epipolar_"+cam->name_,level);
}

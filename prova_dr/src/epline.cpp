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



float EpipolarLine::getCostMagn(const pixelIntensity magnitude_r, const pixelIntensity magnitude_m,
                            const pixelIntensity intensity_r, const pixelIntensity intensity_m) {
  float cost_magn = abs(magnitude_r-magnitude_m);
  float cost_col = abs(intensity_r-intensity_m);
  // return cost_magn;
  return cost_magn+cost_col;
}


float EpipolarLine::getCostNew(const pixelIntensity dh_r, const pixelIntensity dh_m,
                            const pixelIntensity dv_r, const pixelIntensity dv_m,
                            const pixelIntensity intensity_r, const pixelIntensity intensity_m ) {
  float cost_dh = abs(dh_r-dh_m);
  float cost_dv = abs(dv_r-dv_m);
  float cost_col = abs(intensity_r-intensity_m);

  return cost_dh+cost_dv+2*cost_col;
  // return cost_dh+cost_dv;
}



bool EpipolarLine::searchMin(Candidate* candidate, Params* parameters ){
  // iterate through uvs
  float prev_cost = FLT_MAX;
  int min_uv_idx;

  pixelIntensity intensity_r = candidate->intensity_;
  float magnitude_r = candidate->grad_magnitude_;

  // cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magn_cd->evalPixel(candidate->pixel_, magnitude_m);

  // for(Eigen::Vector2f uv : *uvs){

  bool sign=true;

  for(int i=0; i<uvs->size(); i++){
    Eigen::Vector2f uv = uvs->at(i);
    Eigen::Vector2i pixel;
    cam->uv2pixelCoords(uv,pixel,candidate->level_);

    pixelIntensity intensity_m;
    float magnitude_m;


    // check if pixel is within the image
    if(cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magn_cd->evalPixel(pixel, magnitude_m))
    {

      // hard thresholding on magnitude
      bool magnitude_under_threshold=magnitude_m<magnitude_r*parameters->grad_perc_threshold;

      if(magnitude_under_threshold){
        // check if previous cost is a local minimum
        // if(sign && prev_cost<(magnitude_r*parameters->cost_threshold )){
        if(sign && prev_cost<(magnitude_r*parameters->cost_threshold*pow(2,candidate->level_) )){
        // if(sign && prev_cost<(parameters->cost_threshold )){
          // add previous cost inside minimums
          uv_idxs_mins->push_back(i-1);
        }
        // restart search
        sign=false;
        prev_cost = FLT_MAX;
        continue;
      }


      // get cost
      cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->c->evalPixel(pixel, intensity_m);

      // float cost = getCostNew(dh_r,dh_m, dv_r, dv_m, intensity_r,intensity_m );
      float cost = getCostMagn(magnitude_r, magnitude_m, intensity_r, intensity_m);
      if (cost<prev_cost){
        sign=true;
      }
      else{
        // if(sign && prev_cost<(magnitude_r*parameters->cost_threshold*pow(2,candidate->level_))){
        if(sign && prev_cost<(magnitude_r*parameters->cost_threshold)){
        // if(sign && prev_cost<(parameters->cost_threshold)){
          uv_idxs_mins->push_back(i-1);
        }
        sign=false;
      }
      prev_cost = cost;

    }





  }
  // std::cout << prev_cost << std::endl;
  if(uv_idxs_mins->empty())
    return 0;
  return 1;

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
        Eigen::Vector2i pixel;
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
        Eigen::Vector2i pixel;

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
    else
      image_intensity_new =cam->wavelet_dec_->vector_wavelets->at(level)->magn_cd->returnColoredImgFromIntensityImg("epipolar") ;

    drawEpipolar(image_intensity_new, green, level);

    return image_intensity_new;
}

Image<colorRGB>* EpipolarLine::createEpipolarImg(int level){
  return createEpipolarImg("epipolar_"+cam->name_,level);
}

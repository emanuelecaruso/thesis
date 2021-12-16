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

float EpipolarLine::getCostPhase( const colorRGB& phase3C_r, const colorRGB& phase3C_m) {
  float cost_phase = abs(radiansSub(phase3C_r[0],phase3C_m[0],true))+abs(radiansSub(phase3C_r[1],phase3C_m[1],true))+abs(radiansSub(phase3C_r[2],phase3C_m[2],true));
  return cost_phase;
}

float EpipolarLine::getCostMagn(const colorRGB& magnitude3C_r, const colorRGB& magnitude3C_m,
                            const colorRGB& color_r, const colorRGB& color_m) {
  float cost_magn = abs(magnitude3C_r[0]-magnitude3C_m[0])+abs(magnitude3C_r[1]-magnitude3C_m[1])+abs(magnitude3C_r[2]-magnitude3C_m[2]);
  float cost_col = abs(color_r[0]-color_m[0])+abs(color_r[1]-color_m[1])+abs(color_r[2]-color_m[2]);
  // return cost_magn;
  return cost_magn+cost_col;
}

// float EpipolarLine::getCost(const colorRGB& magnitude3C_r, const colorRGB& magnitude3C_m,
//                             const colorRGB& phase3C_r, const colorRGB& phase3C_m,
//                             const colorRGB& color_r, const colorRGB& color_m) {
//   float x0r = magnitude3C_r[0]*cos(phase3C_r[0]);
//   float y0r = magnitude3C_r[0]*sin(phase3C_r[0]);
//   float x1r = magnitude3C_r[1]*cos(phase3C_r[1]);
//   float y1r = magnitude3C_r[1]*sin(phase3C_r[1]);
//   float x2r = magnitude3C_r[2]*cos(phase3C_r[2]);
//   float y2r = magnitude3C_r[2]*sin(phase3C_r[2]);
//
//   float x0m = magnitude3C_m[0]*cos(phase3C_m[0]);
//   float y0m = magnitude3C_m[0]*sin(phase3C_m[0]);
//   float x1m = magnitude3C_m[1]*cos(phase3C_m[1]);
//   float y1m = magnitude3C_m[1]*sin(phase3C_m[1]);
//   float x2m = magnitude3C_m[2]*cos(phase3C_m[2]);
//   float y2m = magnitude3C_m[2]*sin(phase3C_m[2]);
//
//   float cost_0 = sqrt(pow(x0r-x0m,2)+pow(y0r-y0m,2));
//   float cost_1 = sqrt(pow(x1r-x1m,2)+pow(y1r-y1m,2));
//   float cost_2 = sqrt(pow(x2r-x2m,2)+pow(y2r-y2m,2));
//   // float cost_0 = ( ((x0r-x0m)*(x0r-x0m))+ ((y0r-y0m)*(y0r-y0m)) );
//   // float cost_1 = ( ((x1r-x1m)*(x1r-x1m))+ ((y1r-y1m)*(y1r-y1m)) );
//   // float cost_2 = ( ((x2r-x2m)*(x2r-x2m))+ ((y2r-y2m)*(y2r-y2m)) );
//   // float cost_0 = pow(y0r-y0m,2);
//   // float cost_1 = pow(y1r-y1m,2);
//   // float cost_2 = pow(y2r-y2m,2);
//
//   float cost_magn = cost_0+cost_1+cost_2;
//   // std::cout << x0r << " " << x0m << " " << y0r << " " << y0m << " " << x1r << " " << x1m << " " << y1r << " " << y1m << " " << x2r << " " << x2m << " " << y2r << " " << y2m  << " CHG " << cost_0 << " " << cost_1 << " " << cost_2 << std::endl;
//   float cost_col = abs(color_r[0]-color_m[0])+abs(color_r[1]-color_m[1])+abs(color_r[2]-color_m[2]);
//
//   std::cout << cost_magn << " " << cost_col << std::endl;
//
//   return cost_magn;
//   // return cost_magn+cost_col;
// }


// float EpipolarLine::getCost(const colorRGB& magnitude3C_r, const colorRGB& magnitude3C_m,
//                             const colorRGB& phase3C_r, const colorRGB& phase3C_m,
//                             const colorRGB& color_r, const colorRGB& color_m) {
//
//   float cost_magn = abs(radiansSub(phase3C_r[0],phase3C_m[0],true))*abs(magnitude3C_r[0]-magnitude3C_m[0])+
//                     abs(radiansSub(phase3C_r[1],phase3C_m[1],true))*abs(magnitude3C_r[1]-magnitude3C_m[1])+
//                     abs(radiansSub(phase3C_r[2],phase3C_m[2],true))*abs(magnitude3C_r[2]-magnitude3C_m[2]);
//
//   // float cost_magn = abs(magnitude3C_r[0]-magnitude3C_m[0])+
//   //                   abs(magnitude3C_r[1]-magnitude3C_m[1])+
//   //                   abs(magnitude3C_r[2]-magnitude3C_m[2]);
//   float cost_col = abs(color_r[0]-color_m[0])+abs(color_r[1]-color_m[1])+abs(color_r[2]-color_m[2]);
//
//   return cost_magn+cost_col;
// }

float EpipolarLine::getCostNew(const colorRGB& dh_r, const colorRGB& dh_m,
                            const colorRGB& dv_r, const colorRGB& dv_m,
                            const colorRGB& color_r, const colorRGB& color_m ) {
  float cost_dh = abs(dh_r[0]-dh_m[0])+abs(dh_r[1]-dh_m[1])+abs(dh_r[2]-dh_m[2]);
  float cost_dv = abs(dv_r[0]-dv_m[0])+abs(dv_r[1]-dv_m[1])+abs(dv_r[2]-dv_m[2]);
  float cost_col = abs(color_r[0]-color_m[0])+abs(color_r[1]-color_m[1])+abs(color_r[2]-color_m[2]);

  return cost_dh+cost_dv+cost_col;
  // return cost_magn+cost_col;
}

float EpipolarLine::getCost(const colorRGB& magnitude3C_r, const colorRGB& magnitude3C_m,
                            const colorRGB& phase3C_r, const colorRGB& phase3C_m,
                            const colorRGB& color_r, const colorRGB& color_m) {
  float cost_magn = abs(magnitude3C_r[0]-magnitude3C_m[0])+abs(magnitude3C_r[1]-magnitude3C_m[1])+abs(magnitude3C_r[2]-magnitude3C_m[2]);
  float cost_col = abs(color_r[0]-color_m[0])+abs(color_r[1]-color_m[1])+abs(color_r[2]-color_m[2]);
  return cost_magn;
  // return cost_magn+cost_col;
}


bool EpipolarLine::searchMin(Candidate* candidate, Params* parameters ){
  // iterate through uvs
  float prev_cost = FLT_MAX;
  int min_uv_idx;

  colorRGB color_r = candidate->color_;
  float magnitude_r = candidate->grad_magnitude_;
  colorRGB magnitude3C_r = candidate->grad3C_magnitude_;
  colorRGB phase3C_r = candidate->grad3C_phase_;
  colorRGB dh_r = candidate->dh_;
  colorRGB dv_r = candidate->dv_;
  // cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magnitude_img->evalPixel(candidate->pixel_, magnitude_m);

  // for(Eigen::Vector2f uv : *uvs){

  bool sign=true;

  for(int i=0; i<uvs->size(); i++){
    Eigen::Vector2f uv = uvs->at(i);
    Eigen::Vector2i pixel;
    cam->uv2pixelCoords(uv,pixel,candidate->level_);

    colorRGB color_m;
    float magnitude_m;
    colorRGB magnitude3C_m;
    colorRGB phase3C_m;
    colorRGB dh_m;
    colorRGB dv_m;

    // check if pixel is within the image
    if(cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magnitude_img->evalPixel(pixel, magnitude_m))
    {

      // hard thresholding on magnitude
      bool magnitude_under_threshold=magnitude_m<magnitude_r*parameters->grad_perc_threshold;

      // // hard thresholding on phase
      // cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->phase3C_img->evalPixel(pixel, phase3C_m);
      // float cost_phase = getCostPhase( phase3C_r, phase3C_m );
      // // std::cout << "cost_phase " << cost_phase << std::endl;
      // bool phase_cost_over_threshold=cost_phase>0.3;

      // if(magnitude_under_threshold || phase_cost_over_threshold){
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
      cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magnitude3C_img->evalPixel(pixel, magnitude3C_m);
      cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->dh->evalPixel(pixel, dh_m);
      cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->dv->evalPixel(pixel, dv_m);
      // cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magnitude3C_img->evalPixel(pixel, phase3C_m);
      cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->c->evalPixel(pixel, color_m);
      // float cost = getCostMagn(magnitude3C_r,magnitude3C_m, color_r,color_m );
      // float cost = getCost(magnitude3C_r,magnitude3C_m,phase3C_r,phase3C_m, color_r,color_m );
      float cost = getCostNew(dh_r,dh_m, dv_r, dv_m, color_r,color_m );
      // float cost = getCostPhase( phase3C_r,phase3C_m );
      if (cost<prev_cost){
        sign=true;
      }
      else{
        if(sign && prev_cost<(magnitude_r*parameters->cost_threshold*pow(2,candidate->level_))){
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
    Image<colorRGB>* image_rgb_new = createEpipolarImg(level);
    image_rgb_new->show(size*(pow(2,level+1)));
}

void EpipolarLine::showEpipolarWithMin(int level,float size){
    Image<colorRGB>* image_rgb_new = createEpipolarImg(level);

    if(!uv_idxs_mins->empty()){
      for (int idx : *uv_idxs_mins){
        Eigen::Vector2i pixel;
        cam->uv2pixelCoords(uvs->at(idx),pixel,level);
        image_rgb_new->setPixel(pixel,blue);
      }
    }
    image_rgb_new->show(size*(pow(2,level+1)), cam->name_);
}

void EpipolarLine::showEpipolarComparison(EpipolarLine* ep_line_2, bool print=false, float size=1){
    Image<colorRGB>* image_rgb_new = createEpipolarImg();
    Image<colorRGB>* image_rgb_new_2 = ep_line_2->createEpipolarImg();
    image_rgb_new->showWithOtherImage(image_rgb_new_2,size);

}

void EpipolarLine::showEpipolarComparison(EpipolarLine* ep_line_2,
                          const std::string& name, bool print=false, float size=1)
{
    Image<colorRGB>* image_rgb_new = createEpipolarImg();
    Image<colorRGB>* image_rgb_new_2 = ep_line_2->createEpipolarImg();
    image_rgb_new->showWithOtherImage(image_rgb_new_2,name,size);
}



// create imgs to show
Image<colorRGB>* EpipolarLine::createEpipolarImg(const std::string& name, int level){

    Image<colorRGB>* image_rgb_new;
    if (level==-1)
      image_rgb_new = cam->image_rgb_->clone(name);
    else
      image_rgb_new = cam->wavelet_dec_->vector_wavelets->at(level)->c->clone(name);


    if (!uvs->empty())
      for( int i=0; i<uvs->size(); i++){
        Eigen::Vector2i pixel;

        cam->uv2pixelCoords(uvs->at(i),pixel,level);

        // if(i==0)
        //   image_rgb_new->setPixel(pixel, blue);
        // else if (i==uvs->size()-1)
        //   image_rgb_new->setPixel(pixel, red);
        // else
        image_rgb_new->setPixel(pixel, green);
      }
    return image_rgb_new;
}

Image<colorRGB>* EpipolarLine::createEpipolarImg(int level){
  return createEpipolarImg("epipolar_"+cam->name_,level);
}

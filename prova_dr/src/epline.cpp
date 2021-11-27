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

float EpipolarLine::getCost(colorRGB magnitude3C_r, colorRGB magnitude3C_m,colorRGB color_r, colorRGB color_m){
  float cost_magn = abs(magnitude3C_r[0]-magnitude3C_m[0])+abs(magnitude3C_r[1]-magnitude3C_m[1])+abs(magnitude3C_r[2]-magnitude3C_m[2]);
  float cost_col = abs(color_r[0]-color_m[0])+abs(color_r[1]-color_m[1])+abs(color_r[2]-color_m[2]);
  return cost_magn;
  // return cost_magn+cost_col;
}

void EpipolarLine::updateBounds(colorRGB magnitude3C ){

}

void EpipolarLine::searchMin(Candidate* candidate ){
  // iterate through uvs
  float min_cost = FLT_MAX;
  int min_uv_idx;
  // for(Eigen::Vector2f uv : *uvs){
  for(int i=0; i<uvs->size(); i++){
    Eigen::Vector2f uv = uvs->at(i);
    Eigen::Vector2i pixel;
    cam->uv2pixelCoords(uv,pixel,candidate->level_);
    colorRGB magnitude3C_m;
    colorRGB color;
    if(cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->magnitude3C_img->evalPixel(pixel, magnitude3C_m))
    {
      cam->wavelet_dec_->vector_wavelets->at(candidate->level_)->c->evalPixel(pixel, color);
      float cost = getCost(candidate->grad3C_magnitude_,magnitude3C_m, candidate->color_,color );
      if (cost<min_cost){
        min_cost=cost;
        uv_idx_colored=i;
      }
    }
  }


}


// show
void EpipolarLine::showEpipolar(int level,float size){
    Image<colorRGB>* image_rgb_new = createEpipolarImg(level);
    image_rgb_new->show(size*(pow(2,level+1)));
}

void EpipolarLine::showEpipolarWithMin(int level,float size){
    Image<colorRGB>* image_rgb_new = createEpipolarImg(level);
    Eigen::Vector2i pixel;
    cam->uv2pixelCoords(uvs->at(uv_idx_colored),pixel,level);

    image_rgb_new->setPixel(pixel,blue);
    image_rgb_new->show(size*(pow(2,level+1)));
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

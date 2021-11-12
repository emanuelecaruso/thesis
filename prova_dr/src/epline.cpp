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


bool EpipolarLine::resizeLine(){

    float width = cam->cam_parameters_->width;
    float height = cam->cam_parameters_->height;
    float pixel_width = width/((float)cam->cam_parameters_->resolution_x);

    std::vector<float> axes_v {0+(pixel_width/2),width-(pixel_width/2)};
    std::vector<float> axes_h {0+(pixel_width/2),height-(pixel_width/2)};

    bool axis_v_idx = 0;
    bool axis_h_idx;
    if (slope>0)  // 2 and 4 quadrant (y points down)
      axis_h_idx=0;
    else  // 1 and 3 quadrant (y points down)
      axis_h_idx=1;


    Eigen::Vector2f start_uv, end_uv;
    coordToUV(start,start_uv);
    coordToUV(end,end_uv);


    // bring to axis v
    float axis_v= axes_v[axis_v_idx];
    float delta = axis_v-start_uv.x();
    start_uv.y()=start_uv.y()+slope*delta;
    start_uv.x()=axis_v;
    // if out of the square
    if ( (start_uv.y()<axes_h[0]) || (start_uv.y()>axes_h[1]) ){
      // bring to axis h

      float axis_h= axes_h[axis_h_idx];
      float delta = axis_h-start_uv.y();
      start_uv.x()=start_uv.x()+delta/slope;
      start_uv.y()=axis_h;
      // if out of the square again, line does not intersect the rectangle
      if ( (start_uv.x()<axes_v[0]) || (start_uv.x()>axes_v[1]) ){
        sharedCoutDebug("error on start of segment");
        return false;
      }
    }
    UVToCoord(start_uv,start);
    // sharedCoutDebug("slope: "+std::to_string(slope) );
    // sharedCoutDebug("START c0: "+std::to_string(c0) );
    // sharedCoutDebug("vx for end is: "+std::to_string(start_uv.x()));
    // sharedCoutDebug("vy for end is: "+std::to_string(start_uv.y()));
    // sharedCoutDebug("vc for end is: "+std::to_string(start));
    //

    // it may be simplified?? TODO
    // bring to axis v
    axis_v= axes_v[!axis_v_idx];
    delta = axis_v-end_uv.x();
    end_uv.y()=end_uv.y()+slope*delta;
    end_uv.x()=axis_v;
    // if out of the square
    // sharedCoutDebug("\nEND" );
    // sharedCoutDebug("axis_h: "+std::to_string(axis_h));
    // sharedCoutDebug("vx for end is: "+std::to_string(end_uv.x()));
    // sharedCoutDebug("vy for end is: "+std::to_string(end_uv.y()));
    // sharedCoutDebug("vc for end is: "+std::to_string(end));
    if ( (end_uv.y()<axes_h[0]) || (end_uv.y()>axes_h[1]) ){
      // bring to axis h
      float axis_h= axes_h[!axis_h_idx];
      float delta = axis_h-end_uv.y();
      end_uv.x()=end_uv.x()+delta/slope;
      end_uv.y()=axis_h;
      // if out of the square again, line does not intersect the rectangle

      if ( (end_uv.x()<axes_v[0]) || (end_uv.x()>axes_v[1]) ){
        sharedCoutDebug("error on end of segment");
        return false;
      }
    }
    UVToCoord(end_uv,end);




    // pay attention on lineTraverse! TODO
    lineTraverse();
    return true;
}

void EpipolarLine::lineTraverse()
{

    float distance = abs(end-start);

    float pixel_width=cam->cam_parameters_->width/(float)cam->cam_parameters_->resolution_x;

    // alternative solutions? TODO
    int n_uvs= (distance/pixel_width)+1;

    uvs = new std::vector<Eigen::Vector2f>(n_uvs);

    for (int i=0; i<n_uvs; i++){
      float ratio = (float)i/(n_uvs-1);
      float coord = (1-ratio)*start+(ratio)*end;

      Eigen::Vector2f uv;
      coordToUV(coord,uv);
      uvs->at(i)= uv;
    }
}


// show
void EpipolarLine::showEpipolar(float size){
    Image<cv::Vec3b>* image_rgb_new = createEpipolarImg();
    image_rgb_new->show(size);
}

void EpipolarLine::showEpipolarComparison(EpipolarLine* ep_line_2, float size=1){
    Image<cv::Vec3b>* image_rgb_new = createEpipolarImg();
    Image<cv::Vec3b>* image_rgb_new_2 = ep_line_2->createEpipolarImg();
    image_rgb_new->showWithOtherImage(image_rgb_new_2,size);
}

void EpipolarLine::showEpipolarComparison(EpipolarLine* ep_line_2,
                          const std::string& name, float size=1)
{
    Image<cv::Vec3b>* image_rgb_new = createEpipolarImg();
    Image<cv::Vec3b>* image_rgb_new_2 = ep_line_2->createEpipolarImg();
    image_rgb_new->showWithOtherImage(image_rgb_new_2,name,size);
}



// create imgs to show
Image<cv::Vec3b>* EpipolarLine::createEpipolarImg(const std::string& name){
    Image<cv::Vec3b>* image_rgb_new = cam->image_rgb_->clone(name);

    for( int i=0; i<uvs->size(); i++){
      Eigen::Vector2i pixel;

      cam->uv2pixelCoords(uvs->at(i),pixel);

      if(i==0)
        image_rgb_new->setPixel(pixel, blue);
      else if (i==uvs->size()-1)
        image_rgb_new->setPixel(pixel, red);
      else
        image_rgb_new->setPixel(pixel, yellow);
    }
    return image_rgb_new;
}

Image<cv::Vec3b>* EpipolarLine::createEpipolarImg(){
  return createEpipolarImg("epipolar_"+cam->name_);
}

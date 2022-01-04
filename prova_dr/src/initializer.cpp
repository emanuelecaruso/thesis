#include "dtam.h"
#include "initializer.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"

const Image<float>* Initializer::getReferenceImage(){
  return dtam_->camera_vector_->at(ref_frame_idx_)->image_intensity_;
}

const Image<float>* Initializer::getCurrentImage(){
  return dtam_->camera_vector_->at(dtam_->frame_current_)->image_intensity_;
}

const Image<float>* Initializer::getPrevImage(){
  return dtam_->camera_vector_->at(dtam_->frame_current_-1)->image_intensity_;
}

void Initializer::extractCorners(){

  corners_vec_->clear();
  ref_frame_idx_=dtam_->frame_current_;
  const Image<float>* img_r = getReferenceImage();
  corners_vec_->push_back(new std::vector<cv::Point2f>);
  errors_vec_->push_back(new std::vector<float>);
  status_vec_->push_back(new std::vector<uchar>);
  cv::goodFeaturesToTrack(img_r->image_,*(corners_vec_->at(0)),parameters_->n_corners,parameters_->quality_level,parameters_->min_distance);

}

void Initializer::showCornersRef(){
  const Image<float>* img_r = getReferenceImage();
  Image<colorRGB>* show_image= img_r->returnColoredImgFromIntensityImg("corners: "+std::to_string(corners_vec_->at(0)->size()));

  for (auto corner : *(corners_vec_->at(0))){
    show_image->drawCircle(red, corner);
  }
  show_image->show(2);
  // cv::waitKey(0);
}

void Initializer::trackCornersLK(){
  const Image<float>* img_prev = getPrevImage();
  const Image<float>* img_next = getCurrentImage();
  cv::Mat_<uchar> img_prev_uchar;
  cv::Mat_<uchar> img_next_uchar;

  int n = dtam_->frame_current_-ref_frame_idx_;
  // calculate optical flow
  std::vector<uchar> status;
  std::vector<float> err;
  cv::Size size_win = cv::Size(30,30);

  corners_vec_->push_back(new std::vector<cv::Point2f>);
  errors_vec_->push_back(new std::vector<float>);
  status_vec_->push_back(new std::vector<uchar>);

  img_prev->image_.convertTo(img_prev_uchar, CV_8UC1, 255);
  img_next->image_.convertTo(img_next_uchar, CV_8UC1, 255);


  // TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
  calcOpticalFlowPyrLK(img_prev_uchar, img_next_uchar, *(corners_vec_->at(n-1)), *(corners_vec_->at(n)), *(status_vec_->at(n)), *(errors_vec_->at(n)), size_win);
}

void Initializer::showCornersTrack(){
  std::vector<colorRGB> colors;
  for (int i=0; i<corners_vec_->at(0)->size(); i++){
    float r = ((float)rand()/RAND_MAX);
    float g = ((float)rand()/RAND_MAX);
    float b = ((float)rand()/RAND_MAX);
    colorRGB color = colorRGB {b,g,r};
    colors.push_back(color);
  }
  while(true){
    for(int i=0; i<corners_vec_->size(); i++){
      const Image<float>* img = dtam_->camera_vector_->at(ref_frame_idx_+i)->image_intensity_;
      Image<colorRGB>* show_image= img->returnColoredImgFromIntensityImg("corners tracking");
      // for (auto corner : *(corners_vec_->at(0))){
      for (int j=0; j<corners_vec_->at(i)->size(); j++){
        if ( i==0 || ( (status_vec_->at(i)->at(j)) && (errors_vec_->at(i)->at(j))<10 ) ){
          show_image->drawCircle(colors[j], corners_vec_->at(i)->at(j));
        }
      }
      show_image->show(2);
      cv::waitKey(1000);
      // std::cout << "size: " << i << " "<< corners_vec_->at(i)->size() << std::endl;
      // std::cout << "size: " << i << " "<< status_vec_->at(0)->size() << std::endl;
    }


    // show_image->show(2);
  }
}

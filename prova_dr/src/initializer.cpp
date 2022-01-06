#include "dtam.h"
#include "initializer.h"
#include <math.h>
#include "utils.h"
#include <stdlib.h>
#include "defs.h"
#include <opencv2/core/eigen.hpp>

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
  cv::Size size_win = cv::Size(parameters_->size_window,parameters_->size_window);

  corners_vec_->push_back(new std::vector<cv::Point2f>);
  errors_vec_->push_back(new std::vector<float>);
  status_vec_->push_back(new std::vector<uchar>);

  img_prev->image_.convertTo(img_prev_uchar, CV_8UC1, 255);
  img_next->image_.convertTo(img_next_uchar, CV_8UC1, 255);


  // TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
  calcOpticalFlowPyrLK(img_prev_uchar, img_next_uchar, *(corners_vec_->at(n-1)), *(corners_vec_->at(n)), *(status_vec_->at(n)), *(errors_vec_->at(n)), size_win);

  // filter corners
  for (int i=status_vec_->at(n)->size()-1; i>=0; i--){
    // if(errors_vec_->at(n)->at(i)<10 || !(status_vec_->at(n)->at(i)) ){
    if(errors_vec_->at(n)->at(i)>parameters_->err_threshold ){
      // std::cout << i << std::endl;
      for (int j=0; j<=n; j++){
        // std::cout << status_vec_->at(j)->size() << " "<< errors_vec_->at(j)->size() << " "<< corners_vec_->at(j)->size() << std::endl;
        corners_vec_->at(j)->erase (corners_vec_->at(j)->begin()+i);
        if(j>0){
          status_vec_->at(j)->erase (status_vec_->at(j)->begin()+i);
          errors_vec_->at(j)->erase (errors_vec_->at(j)->begin()+i);
        }
      }
    }
  }




  // for (int i=0; i<status_vec_->at(n)->size(); i++){
  //   status_vec_->at(n)->at(i)=errors_vec_->at(n)->at(i)<10;
  // }
}

bool Initializer::findPose(){
  // estimate homography
  // cv::Mat H = findHomography();

  // estimate essential matrix
  // cv::Mat E = findEssentialMatrix();
  cv::Mat F = findFundamentalMatrix();

  // eval models

  // find pose
  cv::Mat E = fundamental2Essential(F);
  Eigen::Isometry3f T = essential2pose( E );
  // Eigen::Isometry3f T = homography2pose( H );


}

Eigen::Isometry3f Initializer::computeRelativePoseGt(){
  Eigen::Isometry3f w_T_m = *(dtam_->environment_->camera_vector_->at(dtam_->frame_current_)->frame_camera_wrt_world_);
  Eigen::Isometry3f r_T_w = *(dtam_->environment_->camera_vector_->at(ref_frame_idx_)->frame_world_wrt_camera_);
  Eigen::Isometry3f r_T_m = r_T_w*w_T_m;

  return r_T_m;
}


cv::Mat Initializer::fundamental2Essential(cv::Mat& F_){

  Eigen::Matrix3f K = *(dtam_->camera_vector_->at(dtam_->frame_current_)->K_);
  Eigen::Matrix3f Kt = K.transpose();
  Eigen::Matrix3f F;
  cv2eigen(  F_, F );

  Eigen::Matrix3f E = Kt*F*K;

  cv::Mat E_;
  eigen2cv(  E, E_ );

  return E_;

}

Eigen::Isometry3f Initializer::homography2pose(cv::Mat& H){
  // get grountruth of the pose to predict
  Eigen::Isometry3f T_gt = computeRelativePoseGt();
   // get groundtruth of scale
  float t_magnitude = T_gt.translation().norm();

  Eigen::Matrix3f K_ = *(dtam_->camera_vector_->at(dtam_->frame_current_)->K_);
  cv::Mat K;
  eigen2cv(K_, K);
  std::vector<cv::Mat> Rs, Ts;

  cv::decomposeHomographyMat(H, K, Rs, Ts, cv::noArray());

  std::cout << "\nCOMPARISON, frame " << dtam_->frame_current_ << std::endl;
  std::cout << "gt: "<< T_gt.translation() << std::endl;
  std::cout << "pred: " << Ts[0] << std::endl;


  Eigen::Isometry3f T;
  // T.linear()=R;
  return T;
}


Eigen::Isometry3f Initializer::essential2pose(cv::Mat& E){

  // get grountruth of the pose to predict
  Eigen::Isometry3f T_gt = computeRelativePoseGt();
   // get groundtruth of scale
  float t_magnitude = T_gt.translation().norm();

  cv::Mat R1, R2, t;
  decomposeEssentialMat( E, R1, R2, t );
  t*=t_magnitude; // adjust scale for t vector

  // compute eigenvalues
  cv::Mat eigenvalues;
  cv::eigen(E,eigenvalues);

  cv::Mat w;
  cv::SVD::compute(	E, w);

  std::cout << "\nCOMPARISON, frame " << dtam_->frame_current_ << std::endl;
  std::cout << "eigenvalues: " << eigenvalues << std::endl;
  std::cout << "singularvalues: " << w << std::endl;
  std::cout << "gt: "<< T_gt.translation() << std::endl;
  std::cout << "pred: " << t << std::endl;

  // Eigen::Matrix3f R;
  // eigen2cv(  R1, R);
  //
  Eigen::Isometry3f T;
  // T.linear()=R;
  return T;
}

cv::Mat Initializer::findEssentialMatrix(){
  int method = cv::RANSAC;
  double prob = 0.995;
  double threshold = 1.0;
  Eigen::Matrix3f K_ = *(dtam_->camera_vector_->at(dtam_->frame_current_)->K_);
  cv::Mat K;
  eigen2cv(K_, K);


  cv::Mat E = cv::findEssentialMat ( *(corners_vec_->at(0)), *(corners_vec_->back()),
                                      K, method, prob, threshold );

  return E;
}

cv::Mat Initializer::findFundamentalMatrix(){
  int method = cv::FM_RANSAC;
  double ransacReprojThreshold = parameters_->ransacReprojThreshold;
  double 	confidence = parameters_->confidence;

  cv::Mat F = cv::findFundamentalMat	(	*(corners_vec_->at(0)), *(corners_vec_->back()),
                                        method, ransacReprojThreshold, confidence );
  return F;
}

cv::Mat Initializer::findHomography(){
  int method = cv::RANSAC;
  double ransacReprojThreshold = parameters_->ransacReprojThreshold;
  const int maxIters = 2000;
  double 	confidence = parameters_->confidence;


  cv::Mat H = cv::findHomography	( *(corners_vec_->at(0)), *(corners_vec_->back()),
                          method, ransacReprojThreshold, cv::noArray(), maxIters, confidence );


  return H;
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
        show_image->drawCircle(colors[j], corners_vec_->at(i)->at(j));
      }
      show_image->show(2);
      cv::waitKey(1000);
    }

  }
}

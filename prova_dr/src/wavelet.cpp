#include "wavelet.h"
#include "camera.h"

Image<float>* Wvlt_lvl::getMagnitude()
{
  Image<pixelIntensity>* magn = new Image<pixelIntensity>("magnitude");

  int width=dh->image_.cols;
  int height=dh->image_.rows;

  magn->initImage(height/2, width/2);

  cv::magnitude(dh->image_, dv->image_, magn->image_);

  return magn;
}



Image<float>* Wvlt_lvl::compute_robust_sobels(){
  Eigen::Matrix3f R = wvlt_dec->cam_->frame_camera_wrt_world_->linear();

  float rollAngle= -atan2(R(1,0),R(1,1));

  float c=cos(rollAngle);
  float s=sin(rollAngle);

  cv::Mat_<pixelIntensity> dv_ = dv->image_.clone();
  cv::Mat_<pixelIntensity> dh_ = dh->image_.clone();
  cv::Mat_<pixelIntensity> dvs;
  cv::Mat_<pixelIntensity> dvc;
  cv::Mat_<pixelIntensity> dhs;
  cv::Mat_<pixelIntensity> dhc;
  cv::multiply(dv_, cv::Scalar(s,s,s), dvs);
  cv::multiply(dv_, cv::Scalar(c,c,c), dvc);
  cv::multiply(dh_, cv::Scalar(s,s,s), dhs);
  cv::multiply(dh_, cv::Scalar(c,c,c), dhc);
  cv::add(dhc,-dvs,dh_robust->image_);
  cv::add(dvc,dhs,dv_robust->image_);
  // dh_robust->image_+= cv::Scalar(1,1,1);
  // dv_robust->image_+= cv::Scalar(1,1,1);
}


void Wvlt_lvl::WaveletDecHaar(const Image<pixelIntensity>* img){
  c=new Image<pixelIntensity>("c");
  dh=new Image<pixelIntensity>("dh");
  dv=new Image<pixelIntensity>("dv");
  dh_robust=new Image<pixelIntensity>("dh_robust");
  dv_robust=new Image<pixelIntensity>("dv_robust");
  magnitude_img=new Image<float>("magnitude");

  int width=img->image_.cols;
  int height=img->image_.rows;

  c->initImage(height/2, width/2);

  magnitude_img->initImage(height/2, width/2);
  dh->initImage(height/2, width/2);
  dv->initImage(height/2, width/2);
  dh_robust->initImage(height/2, width/2);
  dv_robust->initImage(height/2, width/2);

  cv::resize(img->image_, c->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
  // c->image_=img->image_;
  // for (int y=0;y<(height/2);y++)
  // {
  //     for (int x=0; x<(width/2);x++)
  //     {
  //         pixelIntensity c_=(img->image_.at<pixelIntensity>(2*y,2*x)+img->image_.at<pixelIntensity>(2*y,2*x+1)+img->image_.at<pixelIntensity>(2*y+1,2*x)+img->image_.at<pixelIntensity>(2*y+1,2*x+1))*(0.25);
  //         c->image_.at<pixelIntensity>(y,x)=c_;
  //     }
  // }

  dh=c->compute_sobel_x();
  dv=c->compute_sobel_y();

  magnitude_img=getMagnitude();


}


void Wvlt_dec::showWaveletDec(float size){
  showWaveletDec("wavelet decomposition", size);
}

void Wvlt_dec::showWaveletDec(const std::string& name, float size){
  Image<pixelIntensity>* out = new Image<pixelIntensity>(name);
  int cols=image_->image_.cols;
  int rows=image_->image_.rows;


  out->image_= vector_wavelets->at(levels_-1)->c->image_;

  float offset=0.5;

  for (int i=levels_-1; i>=0; i--){
    int cur_rows=rows>>i;
    int cur_cols=cols>>i;
    Wvlt_lvl* wvlt_curr=vector_wavelets->at(i);

    cv::Mat_<pixelIntensity> tmp;
    cv::hconcat(((wvlt_curr->dh->image_/8)+offset),(wvlt_curr->c->image_),tmp);

    if(i==levels_-1)
      out->setAllPixels(0); // black image

    Image<pixelIntensity>* magn = new Image<pixelIntensity>(wvlt_curr->dv);

    cv::hconcat(out->image_,(magn->image_/8)+offset,out->image_);
    // cv::hconcat(out->image_,(wvlt_curr->dh->image_/8)+offset,out->image_);

    cv::vconcat(out->image_,tmp,out->image_);
  }
  out->show(size);
}

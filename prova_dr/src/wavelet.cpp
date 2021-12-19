#include "wavelet.h"
#include "camera.h"

Image<float>* Wvlt_lvl::getMagnitude(const Image<colorRGB>* magnitude_img_)
{
  Image<float>* out = new Image<float>("magnitude");
  Image<colorRGB>* magn = new Image<colorRGB>();

  int width=dh->image_.cols;
  int height=dh->image_.rows;

  out->initImage(height/2, width/2);

  std::vector<cv::Mat> channels_iMagn(3);
  split(magnitude_img_->image_, channels_iMagn);

  out->image_=channels_iMagn[0]+channels_iMagn[1]+channels_iMagn[2];

  return out;
}


Image<colorRGB>* Wvlt_lvl::getMagnitude3C(const Image<colorRGB>* dh,
                                        const Image<colorRGB>* dv)
{
  Image<colorRGB>* magn = new Image<colorRGB>("magnitude3C");

  int width=dh->image_.cols;
  int height=dh->image_.rows;

  magn->initImage(height/2, width/2);

  std::vector<cv::Mat> channels_iMagn(3);
  split(magn->image_, channels_iMagn);

  std::vector<cv::Mat> channels_dh(3);
  split(dh->image_, channels_dh);

  std::vector<cv::Mat> channels_dv(3);
  split(dv->image_, channels_dv);

  for(int i=0; i<3; i++){
    // cv::sqrt(channels_dh[i].mul(channels_dh[i])+channels_dv[i].mul(channels_dv[i]),channels_iMagn[i]);
    cv::magnitude(channels_dh[i], channels_dv[i], channels_iMagn[i]);

  }

  merge(channels_iMagn, magn->image_);

  return magn;
}

Image<float>* Wvlt_lvl::compute_robust_sobels(){
  Eigen::Matrix3f R = wvlt_dec->cam_->frame_camera_wrt_world_->linear();

  float rollAngle= -atan2(R(1,0),R(1,1));

  float c=cos(rollAngle);
  float s=sin(rollAngle);

  cv::Mat_<colorRGB> dv_ = dv->image_.clone();
  cv::Mat_<colorRGB> dh_ = dh->image_.clone();
  cv::Mat_<colorRGB> dvs;
  cv::Mat_<colorRGB> dvc;
  cv::Mat_<colorRGB> dhs;
  cv::Mat_<colorRGB> dhc;
  cv::multiply(dv_, cv::Scalar(s,s,s), dvs);
  cv::multiply(dv_, cv::Scalar(c,c,c), dvc);
  cv::multiply(dh_, cv::Scalar(s,s,s), dhs);
  cv::multiply(dh_, cv::Scalar(c,c,c), dhc);
  cv::add(dhc,-dvs,dh_robust->image_);
  cv::add(dvc,dhs,dv_robust->image_);
  // dh_robust->image_+= cv::Scalar(1,1,1);
  // dv_robust->image_+= cv::Scalar(1,1,1);
}


void Wvlt_lvl::WaveletDecHaar(const Image<colorRGB>* img){
  c=new Image<colorRGB>("c");

  dh=new Image<colorRGB>("dh");
  dv=new Image<colorRGB>("dv");
  dh_robust=new Image<colorRGB>("dh_robust");
  dv_robust=new Image<colorRGB>("dv_robust");
  magnitude3C_img=new Image<colorRGB>("magnitude3C");
  magnitude_img=new Image<float>("magnitude");

  int width=img->image_.cols;
  int height=img->image_.rows;

  c->initImage(height/2, width/2);

  magnitude3C_img->initImage(height/2, width/2);
  magnitude_img->initImage(height/2, width/2);
  dh->initImage(height/2, width/2);
  dv->initImage(height/2, width/2);
  dh_robust->initImage(height/2, width/2);
  dv_robust->initImage(height/2, width/2);


  for (int y=0;y<(height/2);y++)
  {
      for (int x=0; x<(width/2);x++)
      {
          colorRGB c_=(img->image_.at<colorRGB>(2*y,2*x)+img->image_.at<colorRGB>(2*y,2*x+1)+img->image_.at<colorRGB>(2*y+1,2*x)+img->image_.at<colorRGB>(2*y+1,2*x+1))*(0.25);
          c->image_.at<colorRGB>(y,x)=c_;
      }
  }

  dh=c->compute_sobel_x();
  dv=c->compute_sobel_y();

  magnitude3C_img=getMagnitude3C(dh,dv);
  magnitude_img=getMagnitude(magnitude3C_img);


}


void Wvlt_dec::showWaveletDec(float size){
  showWaveletDec("wavelet decomposition", size);
}

void Wvlt_dec::showWaveletDec(const std::string& name, float size){
  Image<colorRGB>* out = new Image<colorRGB>(name);
  int cols=image_->image_.cols;
  int rows=image_->image_.rows;


  out->image_= vector_wavelets->at(levels_-1)->c->image_;

  float offset=0.5;

  for (int i=levels_-1; i>=0; i--){
    int cur_rows=rows>>i;
    int cur_cols=cols>>i;
    Wvlt_lvl* wvlt_curr=vector_wavelets->at(i);

    cv::Mat_<colorRGB> tmp;
    cv::hconcat(((wvlt_curr->dh->image_/8)+offset),(wvlt_curr->c->image_),tmp);

    if(i==levels_-1)
      out->setAllPixels(black);

    Image<colorRGB>* magn = new Image<colorRGB>(wvlt_curr->dv);

    cv::hconcat(out->image_,(magn->image_/8)+offset,out->image_);
    // cv::hconcat(out->image_,(wvlt_curr->dh->image_/8)+offset,out->image_);

    cv::vconcat(out->image_,tmp,out->image_);
  }
  out->show(size);
}

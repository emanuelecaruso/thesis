#include "wavelet.h"

Image<float>* Wvlt_lvl::getMagnitude(const Image<cv::Vec3f>* dh,
                                        const Image<cv::Vec3f>* dv)
{
  Image<float>* out = new Image<float>("magnitude");
  Image<cv::Vec3f>* magn = new Image<cv::Vec3f>();

  int width=dh->image_.cols;
  int height=dh->image_.rows;

  out->initImage(height/2, width/2);
  magn->initImage(height/2, width/2);

  std::vector<cv::Mat> channels_iMagn(3);
  split(magn->image_, channels_iMagn);

  std::vector<cv::Mat> channels_i1(3);
  split(dh->image_, channels_i1);

  std::vector<cv::Mat> channels_i2(3);
  split(dv->image_, channels_i2);

  for(int i=0; i<3; i++){
    cv::sqrt(channels_i1[i].mul(channels_i1[i])+channels_i2[i].mul(channels_i2[i]),channels_iMagn[i]);
  }

  out->image_=channels_iMagn[0]+channels_iMagn[1]+channels_iMagn[2];

  // cv::sqrt(channels_iMagn[0].mul(channels_iMagn[0])+channels_iMagn[1].mul(channels_iMagn[1])+channels_iMagn[2].mul(channels_iMagn[2]),out->image_);
  //
  // out->image_=channels_i1[0];
  delete magn;
  return out;
}



void Wvlt_lvl::WaveletDecHaar(const Image<cv::Vec3f>* img){
  c=new Image<cv::Vec3f>("c");

  dd=new Image<cv::Vec3f>("dd");
  dh=new Image<cv::Vec3f>("dh");
  dv=new Image<cv::Vec3f>("dv");
  magnitude_img=new Image<float>("magnitude");

  int width=img->image_.cols;
  int height=img->image_.rows;

  c->initImage(height/2, width/2);

  magnitude_img->initImage(height/2, width/2);
  dh->initImage(height/2, width/2);
  dv->initImage(height/2, width/2);

  dd->initImage(height/2, width/2);
  dd->setAllPixels(black);


  for (int y=0;y<(height/2);y++)
  {
      for (int x=0; x<(width/2);x++)
      {
          cv::Vec3f c_=(img->image_.at<cv::Vec3f>(2*y,2*x)+img->image_.at<cv::Vec3f>(2*y,2*x+1)+img->image_.at<cv::Vec3f>(2*y+1,2*x)+img->image_.at<cv::Vec3f>(2*y+1,2*x+1))*(0.25);
          c->image_.at<cv::Vec3f>(y,x)=c_;
      }
  }

  dh=c->compute_sobel_x();
  dv=c->compute_sobel_y();
  magnitude_img=getMagnitude(dh,dv);
}


void Wvlt_dec::showWaveletDec(float size){
  showWaveletDec("wavelet decomposition", size);
}

void Wvlt_dec::showWaveletDec(const std::string& name, float size){
  Image<cv::Vec3f>* out = new Image<cv::Vec3f>(name);
  int cols=image_->image_.cols;
  int rows=image_->image_.rows;
  // initImage(rows, cols);


  // Wvlt_lvl* wvlt_curr=vector_wavelets->at(levels_-1);
  out->image_= vector_wavelets->at(levels_-1)->c->image_;

  // unsigned char offset=UCHAR_MAX/2;
  float offset=0.5;
  // out->image_= wvlt_curr->c->image_;


  for (int i=levels_-1; i>=0; i--){
    int cur_rows=rows>>i;
    int cur_cols=cols>>i;
    Wvlt_lvl* wvlt_curr=vector_wavelets->at(i);

    cv::Mat_<cv::Vec3f> tmp;
    cv::hconcat((wvlt_curr->dv->image_/8)+offset,(wvlt_curr->c->image_),tmp);

    if(i==levels_-1)
      out->setAllPixels(black);
    cv::hconcat(out->image_,(wvlt_curr->dh->image_/8)+offset,out->image_);

    cv::vconcat(out->image_,tmp,out->image_);
  }
  out->show(size);
}

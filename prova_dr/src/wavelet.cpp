#include "wavelet.h"

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

  std::vector<cv::Mat> channels_i1(3);
  split(dh->image_, channels_i1);

  std::vector<cv::Mat> channels_i2(3);
  split(dv->image_, channels_i2);

  for(int i=0; i<3; i++){
    cv::sqrt(channels_i1[i].mul(channels_i1[i])+channels_i2[i].mul(channels_i2[i]),channels_iMagn[i]);
  }

  merge(channels_iMagn, magn->image_);

  return magn;
}


void Wvlt_lvl::WaveletDecHaar(const Image<colorRGB>* img){
  c=new Image<colorRGB>("c");

  dd=new Image<colorRGB>("dd");
  dh=new Image<colorRGB>("dh");
  dv=new Image<colorRGB>("dv");
  magnitude3C_img=new Image<colorRGB>("magnitude3C");
  magnitude_img=new Image<float>("magnitude");

  int width=img->image_.cols;
  int height=img->image_.rows;

  c->initImage(height/2, width/2);

  magnitude3C_img->initImage(height/2, width/2);
  magnitude_img->initImage(height/2, width/2);
  dh->initImage(height/2, width/2);
  dv->initImage(height/2, width/2);

  dd->initImage(height/2, width/2);
  dd->setAllPixels(black);


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

    cv::Mat_<colorRGB> tmp;
    cv::hconcat((wvlt_curr->c->image_),(wvlt_curr->magnitude3C_img->image_/8),tmp);

    if(i==levels_-1)
      out->setAllPixels(black);

    Image<colorRGB>* magn = new Image<colorRGB>;
    cv::cvtColor(wvlt_curr->magnitude_img->image_,magn->image_, cv::COLOR_GRAY2BGR );

    cv::hconcat(out->image_,(magn->image_/(8*3))+offset,out->image_);
    // cv::hconcat(out->image_,(wvlt_curr->dh->image_/8)+offset,out->image_);

    cv::vconcat(out->image_,tmp,out->image_);
  }
  out->show(size);
}

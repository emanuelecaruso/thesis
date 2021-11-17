#pragma once
#include "defs.h"
#include "image.h"



class Wvlt_lvl{
  public:
    const int level;

    Image<cv::Vec3f>* c;
    Image<cv::Vec3f>* dh;
    Image<cv::Vec3f>* dv;
    Image<cv::Vec3f>* dd;

    Image<cv::Vec3i>* max_idx;
    Image<float>* norm_max;

    // std::vector<Image<cv::Vec3i>*>* max_idx_vec = new std::vector<Image<cv::Vec3i>*>(level<<2);
    // std::vector<Image<float>*>* norm_max_vec = new std::vector<Image<float>*>(level<<2);



    Wvlt_lvl(Image<cv::Vec3f>* c_, Image<cv::Vec3f>* dh_,
            Image<cv::Vec3f>* dv_, Image<cv::Vec3f>* dd_,
            Image<cv::Vec3i>* max_idx_, Image<float>* norm_max_,
           const int level_):
              level(level_)
    {
        c=c_->clone();
        dh=dh_->clone();
        dv=dv_->clone();
        dd=dd_->clone();
        max_idx=max_idx_->clone();
        norm_max=norm_max_->clone();
    };

    Wvlt_lvl(const Image<cv::Vec3f>* img ):
    level(0){
      WaveletDecHaar(img);
    };

    Wvlt_lvl(Wvlt_lvl* wvlt_lvl_previous):
    level(wvlt_lvl_previous->level+1){
      WaveletDecHaar( wvlt_lvl_previous);
    };
    inline Wvlt_lvl* clone(){ return new Wvlt_lvl(c,dh,dv,dd, max_idx, norm_max, level); };
  private:
    void WaveletDecHaar(const Image<cv::Vec3f>* img);
    void WaveletDecHaar(Wvlt_lvl* wvlt_lvl_previous);
    float filterWaves(int x, int y,const Image<cv::Vec3f>* img );

};

class Wvlt_dec{

  public:
    const int levels_;
    const Image<cv::Vec3f>* image_;
    std::vector< Wvlt_lvl* >* vector_wavelets;

    // clone wlt decomposition
    Wvlt_dec(Wvlt_dec* wvlt_dec ):
    levels_( wvlt_dec->levels_ ),
    image_( wvlt_dec->image_ )
    {
      vector_wavelets = new std::vector<Wvlt_lvl*>(levels_);
      for (int i=0; i<levels_; i++){

        Wvlt_lvl* wvlt_lvl = wvlt_dec->vector_wavelets->at(i);
        vector_wavelets->at(i)=wvlt_lvl->clone();
      }

    }

    // compute wlt decomposition
    Wvlt_dec(int levels,const Image<cv::Vec3f>* img ):
    levels_(levels),
    image_(img)
    {

      vector_wavelets = new std::vector<Wvlt_lvl*>(levels_);

      Wvlt_lvl* wvlt_0 = new Wvlt_lvl( image_ );
      vector_wavelets->at(0) = wvlt_0;

      for(int i=1; i<levels_; i++){
        Wvlt_lvl* wvlt_i = new Wvlt_lvl( vector_wavelets->at(i-1) );
        // Wvlt_lvl* wvlt_i = new Wvlt_lvl( vector_wavelets->at(i-1)->c );
        vector_wavelets->at(i)=wvlt_i;
      }
    }


    void signThresholdedPoints(float threshold, bool printNPix=false);
    void reconstructImage();
    void compareThreshold(float threshold, float size=1);
    void showWaveletDec(float size=1);
    void showWaveletDec(const std::string& name, float size=1);

};

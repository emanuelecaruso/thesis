#include "wavelet.h"



float Wvlt_lvl::filterWaves(int x, int y, const Image<cv::Vec3f>* img){
  const cv::Mat* src=&(img->image_);

  // cv::Vec3f c_=(src->at<cv::Vec3f>(2*y,2*x)+src->at<cv::Vec3f>(2*y,2*x+1)+src->at<cv::Vec3f>(2*y+1,2*x)+src->at<cv::Vec3f>(2*y+1,2*x+1))*(0.21);
  cv::Vec3f c_=(src->at<cv::Vec3f>(2*y,2*x)+src->at<cv::Vec3f>(2*y,2*x+1)+src->at<cv::Vec3f>(2*y+1,2*x)+src->at<cv::Vec3f>(2*y+1,2*x+1))*(0.25);
  // cv::Vec3f c_=(src->at<cv::Vec3f>(2*y,2*x)+src->at<cv::Vec3f>(2*y,2*x+1)+src->at<cv::Vec3f>(2*y+1,2*x)+src->at<cv::Vec3f>(2*y+1,2*x+1))*(0.3);
  c->image_.at<cv::Vec3f>(y,x)=c_;

  cv::Vec3f dh_=(src->at<cv::Vec3f>(2*y,2*x)+src->at<cv::Vec3f>(2*y+1,2*x)-src->at<cv::Vec3f>(2*y,2*x+1)-src->at<cv::Vec3f>(2*y+1,2*x+1))*0.5;
  dh->image_.at<cv::Vec3f>(y,x)=dh_;

  cv::Vec3f dv_=(src->at<cv::Vec3f>(2*y,2*x)+src->at<cv::Vec3f>(2*y,2*x+1)-src->at<cv::Vec3f>(2*y+1,2*x)-src->at<cv::Vec3f>(2*y+1,2*x+1))*0.5;
  dv->image_.at<cv::Vec3f>(y,x)=dv_;

  cv::Vec3f dd_=(src->at<cv::Vec3f>(2*y,2*x)-src->at<cv::Vec3f>(2*y,2*x+1)-src->at<cv::Vec3f>(2*y+1,2*x)+src->at<cv::Vec3f>(2*y+1,2*x+1))*0.5;
  dd->image_.at<cv::Vec3f>(y,x)=dd_;

  float norm=l1Norm(dh_)+l1Norm(dv_)+l1Norm(dd_);
  return norm;
}


void Wvlt_lvl::WaveletDecHaar(const Image<cv::Vec3f>* img){
  c=new Image<cv::Vec3f>("c");
  dh=new Image<cv::Vec3f>("dh");
  dv=new Image<cv::Vec3f>("dv");
  dd=new Image<cv::Vec3f>("dd");
  max_idx=new Image<cv::Vec3i>("max_idx");
  norm_max=new Image<float>("norm_max");

  int width=img->image_.cols;
  int height=img->image_.rows;

  c->initImage(height/2, width/2);
  dh->initImage(height/2, width/2);
  dv->initImage(height/2, width/2);
  dd->initImage(height/2, width/2);
  max_idx->initImage(height/2, width/2);
  norm_max->initImage(height/2, width/2);


  for (int y=0;y<(height/2);y++)
  {
      for (int x=0; x<(width/2);x++)
      {

          float norm=filterWaves( x, y, img);

          norm_max->setPixel(y,x,norm);
          cv::Vec3i max_idx_(y,x,level);
          max_idx->setPixel(y,x,max_idx_);

      }
  }

}

void Wvlt_lvl::WaveletDecHaar(Wvlt_lvl* wvlt_lvl_previous){
  c=new Image<cv::Vec3f>("c");
  dh=new Image<cv::Vec3f>("dh");
  dv=new Image<cv::Vec3f>("dv");
  dd=new Image<cv::Vec3f>("dd");
  max_idx=new Image<cv::Vec3i>("max_idx");
  norm_max=new Image<float>("norm_max");

  int width=wvlt_lvl_previous->c->image_.cols;
  int height=wvlt_lvl_previous->c->image_.rows;

  c->initImage(height/2, width/2);
  dh->initImage(height/2, width/2);
  dv->initImage(height/2, width/2);
  dd->initImage(height/2, width/2);
  max_idx->initImage(height/2, width/2);
  norm_max->initImage(height/2, width/2);

  for (int y=0;y<(height/2);y++)
  {
      for (int x=0; x<(width/2);x++)
      {

        float norm=filterWaves( x, y, wvlt_lvl_previous->c);

        float max_norm_=norm;
        cv::Vec3i max_idx_(y,x,level);


        for(int i=0; i<2; i++){
          for(int j=0; j<2; j++){
            float norm_max_ij = wvlt_lvl_previous->norm_max->evalPixel(2*y+i,2*x+j);
            if(norm_max_ij>norm){
              max_norm_=norm_max_ij;
              max_idx_=wvlt_lvl_previous->max_idx->evalPixel(2*y+i,2*x+j);
            }
          }
        }
        norm_max->setPixel(y,x,max_norm_);
        max_idx->setPixel(y,x,max_idx_);
    }
  }

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
    int cur_rows=rows>>i+1;
    int cur_cols=cols>>i+1;
    Wvlt_lvl* wvlt_curr=vector_wavelets->at(i);

    cv::hconcat(out->image_,(wvlt_curr->dh->image_/4)+offset,out->image_);
    cv::Mat_<cv::Vec3f> tmp;
    cv::hconcat((wvlt_curr->dv->image_/4)+offset,(wvlt_curr->dd->image_/4)+offset,tmp);
    cv::vconcat(out->image_,tmp,out->image_);
  }
  out->show(size);
}



void Wvlt_dec::signThresholdedPoints(float threshold, bool printNPix){
  int cols=image_->image_.cols;
  int rows=image_->image_.rows;
  int n_pixels_kept=0;

  int cur_rows=rows>>levels_;
  int cur_cols=cols>>levels_;

  Wvlt_lvl* wvlt_curr=vector_wavelets->at(levels_-1);

  for (int row=0; row<cur_rows; row++)
  {
    for (int col=0; col<cur_cols ;col++)
    {

      float norm=wvlt_curr->norm_max->evalPixel(row,col);
      cv::Vec3i idx=wvlt_curr->max_idx->evalPixel(row,col);

      if (norm>threshold){
        // sharedCout("lvl: "+std::to_string(idx[2]));
        Wvlt_lvl* wvlt_curr_=vector_wavelets->at(idx[2]);
        wvlt_curr_->dh->setPixel(idx[0],idx[1],white);
        wvlt_curr_->dv->setPixel(idx[0],idx[1],white);
        wvlt_curr_->dd->setPixel(idx[0],idx[1],white);
        n_pixels_kept++;
      }
    }
  }

  if (printNPix)
    sharedCout("n° pixels kept: "+std::to_string(n_pixels_kept));
}


void Wvlt_dec::compareThreshold(float threshold, float size){
  Wvlt_dec* thresholded = new Wvlt_dec(this);
  thresholded->signThresholdedPoints(threshold,true);

  thresholded->showWaveletDec("thresholded",size);
  this->showWaveletDec("original",size);

}
// }


/*
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace cv;
using namespace std;

// Filter type
#define NONE 0  // no filter
#define HARD 1  // hard shrinkage
#define SOFT 2  // soft shrinkage
#define GARROT 3  // garrot filter
//--------------------------------
// signum
//--------------------------------
float sgn(float x)
{
    float res=0;
    if(x==0)
    {
        res=0;
    }
    if(x>0)
    {
        res=1;
    }
    if(x<0)
    {
        res=-1;
    }
    return res;
}
//--------------------------------
// Soft shrinkage
//--------------------------------
float soft_shrink(float d,float T)
{
    float res;
    if(fabs(d)>T)
    {
        res=sgn(d)*(fabs(d)-T);
    }
    else
    {
        res=0;
    }

    return res;
}
//--------------------------------
// Hard shrinkage
//--------------------------------
float hard_shrink(float d,float T)
{
    float res;
    if(fabs(d)>T)
    {
        res=d;
    }
    else
    {
        res=0;
    }

    return res;
}
//--------------------------------
// Garrot shrinkage
//--------------------------------
float Garrot_shrink(float d,float T)
{
    float res;
    if(fabs(d)>T)
    {
        res=d-((T*T)/d);
    }
    else
    {
        res=0;
    }

    return res;
}
//--------------------------------
// Wavelet transform
//--------------------------------
static void cvHaarWavelet(Mat &src,Mat &dst,int NIter)
{
    float c,dh,dv,dd;
    assert( src.type() == CV_32FC1 );
    assert( dst.type() == CV_32FC1 );
    int width = src.cols;
    int height = src.rows;
    for (int k=0;k<NIter;k++)
    {
        for (int y=0;y<(height>>(k+1));y++)
        {
            for (int x=0; x<(width>>(k+1));x++)
            {
                c=(src.at<float>(2*y,2*x)+src.at<float>(2*y,2*x+1)+src.at<float>(2*y+1,2*x)+src.at<float>(2*y+1,2*x+1))*0.5;
                dst.at<float>(y,x)=c;

                dh=(src.at<float>(2*y,2*x)+src.at<float>(2*y+1,2*x)-src.at<float>(2*y,2*x+1)-src.at<float>(2*y+1,2*x+1))*0.5;
                dst.at<float>(y,x+(width>>(k+1)))=dh;

                dv=(src.at<float>(2*y,2*x)+src.at<float>(2*y,2*x+1)-src.at<float>(2*y+1,2*x)-src.at<float>(2*y+1,2*x+1))*0.5;
                dst.at<float>(y+(height>>(k+1)),x)=dv;

                dd=(src.at<float>(2*y,2*x)-src.at<float>(2*y,2*x+1)-src.at<float>(2*y+1,2*x)+src.at<float>(2*y+1,2*x+1))*0.5;
                dst.at<float>(y+(height>>(k+1)),x+(width>>(k+1)))=dd;
            }
        }
        dst.copyTo(src);
    }
}
//--------------------------------
//Inverse wavelet transform
//--------------------------------
static void cvInvHaarWavelet(Mat &src,Mat &dst,int NIter, int SHRINKAGE_TYPE=0, float SHRINKAGE_T=50)
{
    float c,dh,dv,dd;
    assert( src.type() == CV_32FC1 );
    assert( dst.type() == CV_32FC1 );
    int width = src.cols;
    int height = src.rows;
    //--------------------------------
    // NIter - number of iterations
    //--------------------------------
    for (int k=NIter;k>0;k--)
    {
        for (int y=0;y<(height>>k);y++)
        {
            for (int x=0; x<(width>>k);x++)
            {
                c=src.at<float>(y,x);
                dh=src.at<float>(y,x+(width>>k));
                dv=src.at<float>(y+(height>>k),x);
                dd=src.at<float>(y+(height>>k),x+(width>>k));

               // (shrinkage)
                switch(SHRINKAGE_TYPE)
                {
                case HARD:
                    dh=hard_shrink(dh,SHRINKAGE_T);
                    dv=hard_shrink(dv,SHRINKAGE_T);
                    dd=hard_shrink(dd,SHRINKAGE_T);
                    break;
                case SOFT:
                    dh=soft_shrink(dh,SHRINKAGE_T);
                    dv=soft_shrink(dv,SHRINKAGE_T);
                    dd=soft_shrink(dd,SHRINKAGE_T);
                    break;
                case GARROT:
                    dh=Garrot_shrink(dh,SHRINKAGE_T);
                    dv=Garrot_shrink(dv,SHRINKAGE_T);
                    dd=Garrot_shrink(dd,SHRINKAGE_T);
                    break;
                }

                //-------------------
                dst.at<float>(y*2,x*2)=0.5*(c+dh+dv+dd);
                dst.at<float>(y*2,x*2+1)=0.5*(c-dh+dv-dd);
                dst.at<float>(y*2+1,x*2)=0.5*(c+dh-dv-dd);
                dst.at<float>(y*2+1,x*2+1)=0.5*(c-dh-dv+dd);
            }
        }
        Mat C=src(Rect(0,0,width>>(k-1),height>>(k-1)));
        Mat D=dst(Rect(0,0,width>>(k-1),height>>(k-1)));
        D.copyTo(C);
    }
}
//--------------------------------
//
//--------------------------------
*/

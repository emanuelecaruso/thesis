#pragma once
#include "defs.h"
// #include "wavelet.h"

using namespace pr;

template<class T>
class Image{
  public:
    const std::string name_;
    cv::Mat_< T > image_;

    Image(const std::string& name):
    name_(name){};

    Image(const std::string& name, cv::Mat_<T>& image):
    name_(name),
    image_(image){};

    Image(const Image* img):
    name_(img->name_),
    image_(img->image_){};

    Image():
    name_("undefined"){};

    inline void initImage(const int rows, const int cols){
      image_ = cv::Mat_< T >(rows ,cols);
    }

    inline void setAllPixels( const T& color){
      image_ = color;
    }

    // inline void show(float image_scale=1, const std::string& name_ext="") const{
    //   cv::Mat_< T > resized_image;
    //   cv::resize(image_, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
    //   cv::imshow(name_+name_ext, resized_image);
    // }

    inline void show(float image_scale, const std::string& name) const{
      cv::Mat_< T > resized_image;
      cv::resize(image_, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
      cv::imshow(name, resized_image);
    }

    inline void show(float image_scale=1) const{
      show(image_scale, name_);
    }

    inline void showWithOtherImage(const Image<T>* image_2, float image_scale=1) const{
      cv::Mat_<T> collage;
      cv::hconcat(image_,image_2->image_,collage);
      cv::Mat_< T > resized_image;
      cv::resize(collage, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
      cv::imshow(name_+","+image_2->name_, resized_image);
    }

    inline void showWithOtherImage(const Image<T>* image_2,const std::string& name,
                                    float image_scale=1 ) const{
      cv::Mat_<T> collage;
      cv::hconcat(image_,image_2->image_,collage);
      cv::Mat_< T > resized_image;
      cv::resize(collage, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
      cv::imshow(name, resized_image);
    }

    inline Image* clone(const std::string& new_name) const{
      Image<T>* new_img = new Image<T>(new_name);
      new_img->image_ = image_.clone();
      return new_img;
    }
    inline Image* clone() const{
      Image<T>* new_img = new Image<T>(name_);
      new_img->image_ = image_.clone();
      return new_img;
    }

    inline Image<colorRGB>* returnColoredImgFromIntensityImg(const std::string& new_name) const{

      Image<colorRGB>* new_img = new Image<colorRGB>(new_name);
      cv::cvtColor(image_, new_img->image_, cv::COLOR_GRAY2BGR);
      return new_img;
    }

    inline T evalPixel(const int row, const int col) const{
      T color;
      if (row>=0 && row<image_.rows && col>=0 && col<image_.cols)
      {
        color = image_.template at<T>(row,col);
      }
      return color;
    }

    inline bool evalPixel(const Eigen::Vector2i& pixel_coords, T& color) const{
      if (pixel_coords.y()>=0 && pixel_coords.y()<image_.rows && pixel_coords.x()>=0 && pixel_coords.x()<image_.cols)
      {
        color = image_.template at<T>(pixel_coords.y(),pixel_coords.x());
        return true;
      }
      return false;
    }

    inline bool evalPixel(const int row, const int col, T& color) const{
      if (row>=0 && row<image_.rows && col>=0 && col<image_.cols)
      {
        color = image_.template at<T>(row,col);
        return true;
      }
      return false;
    }

    inline bool setPixel(const Eigen::Vector2i& pixel_coords, const T& color){
      if (pixel_coords.y()>=0 && pixel_coords.y()<image_.rows && pixel_coords.x()>=0 && pixel_coords.x()<image_.cols)
      {
        image_.template at<T>(pixel_coords.y(),pixel_coords.x()) = color;
        return true;
      }
      return false;
    }

    inline bool setPixel(const int row, const int col, const T& color){
      if (row>=0 && row<image_.rows && col>=0 && col<image_.cols)
      {
        image_.template at<T>(row,col) = color;
        return true;
      }
      return false;
    }

    inline void loadJpg(const std::string& path){
      image_ = cv::imread(path);
      if(image_.empty())
      std::cout << "Could not read the image: " << path << std::endl;
    }

    inline Image< float >* getChannel(int channel) const{
      if (channel<0 || channel>2){
        sharedCout("wrong channel in 'get channel'");
        return nullptr;
      }

      Image<  float >* out =new Image< float >(std::to_string(channel)+"chan_"+name_);

      std::vector<cv::Mat> channels(3);

      split(image_, channels);

      out->image_=channels[channel];

      return out;
    }

    //
    // inline Image* filter3ChannImg(cv::Mat_<float> kernel,
    //                                 const std::string& name) const{
    //
    //   Image< T >* img_filtered =new Image< T >(name);
    //
    //   std::vector<cv::Mat> channels_o(3);
    //   std::vector<cv::Mat> channels_i(3);
    //
    //   split(image_, channels_i);
    //
    //   filter2D(channels_i[0], channels_o[0], colorRGB_CODE, kernel);
    //   filter2D(channels_i[1], channels_o[1], colorRGB_CODE, kernel);
    //   filter2D(channels_i[2], channels_o[2], colorRGB_CODE, kernel);
    //
    //   /// Merge the three channels
    //   merge(channels_o, img_filtered->image_);
    //
    //   return img_filtered;
    // }
    //
    // inline int thresholdHard(float threshold) {
    //   int cols=image_.cols;
    //   int rows=image_.rows;
    //
    //   int n_pixels_kept=0;
    //
    //   for (int row=0;row<rows;row++)
    //   {
    //       for (int col=0; col<cols;col++)
    //       {
    //           colorRGB clr ;
    //           evalPixel(row,col,clr);
    //
    //           float norm=l1Norm(clr);
    //           if (norm<threshold){
    //             setPixel(row,col,black);
    //           }
    //           else
    //             n_pixels_kept++;
    //       }
    //   }
    //   return n_pixels_kept;
    // }

    inline Image< pixelIntensity>* compute_sobel_x(const std::string& name) const{
      Image<  pixelIntensity >* img_sobel_x=new Image< pixelIntensity >(name);

      cv::Mat_<float> kernel(3,3);
      kernel <<  1,  0, -1,
                 2,  0, -2,
                 1,  0, -1;

      filter2D(image_, img_sobel_x->image_, pixelIntensity_CODE, kernel);

      return img_sobel_x;
    }

    inline Image<pixelIntensity>* compute_sobel_y(const std::string& name) const{
      Image< pixelIntensity >* img_sobel_y =new Image< pixelIntensity >(name);

      cv::Mat_<float> kernel(3,3);
      kernel <<   1,  2,  1,
                  0,  0,  0,
                 -1, -2, -1;

      filter2D(image_, img_sobel_y->image_, pixelIntensity_CODE, kernel);

      return img_sobel_y;
    }

    inline Image< pixelIntensity>* compute_sobel_x() const{
      return compute_sobel_x("fx_"+name_);
    }

    inline Image<pixelIntensity>* compute_sobel_y() const{
      return compute_sobel_y("fy_"+name_);
    }

    inline Image<pixelIntensity>* squared() const{
      Image< pixelIntensity >* squared =new Image< pixelIntensity >("^2_"+name_);

      squared->image_=image_.mul(image_);

      return squared;
    }

    inline Image<float>* getComponentSum() const{
      Image< float >* intensity =new Image< float >("^intensity_"+name_);

      std::vector<cv::Mat> channels_i(3);
      split(image_, channels_i);

      intensity->image_=channels_i[0]+channels_i[1]+channels_i[2];

      return intensity;
    }

    inline void drawRectangle(cv::Rect rect, colorRGB color, int type, float alpha=1){
      cv::Mat roi = image_(rect);
      cv::Mat clr(roi.size(), colorRGB_CODE, color);
      cv::addWeighted(clr, alpha, roi, 1.0 - alpha , 0.0, roi);
      // cv::rectangle(image_,rect, color, type);
    }

    inline void showImgWithColoredPixel(const Eigen::Vector2i pixel, float size, const std::string& name) const{
      Image< colorRGB >* out =this->clone();
      out->setPixel( pixel, red);
      out->show(size, name);
    }
};

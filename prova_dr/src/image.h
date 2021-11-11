#pragma once
#include "defs.h"

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

    Image():
    name_("undefined"){};

    inline void initImage(const int rows, const int cols){
      image_ = cv::Mat_< T >(rows ,cols);
    }

    inline void setAllPixels( const T& color){
      image_ = color;
    }

    inline void show(float image_scale=1, const std::string& name_ext="") const{
      cv::Mat_< T > resized_image;
      cv::resize(image_, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
      cv::imshow(name_+name_ext, resized_image);
    }
    inline void showWithOtherImage(const Image<T>* image_2, float image_scale=1) const{
      cv::Mat_<T> collage;
      cv::hconcat(image_,image_2->image_,collage);
      cv::Mat_< T > resized_image;
      cv::resize(collage, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
      cv::imshow(name_+","+image_2->name_, collage);
    }

    inline void showWithOtherImage(const Image<T>* image_2,const std::string& name,
                                    float image_scale=1 ) const{
      cv::Mat_<T> collage;
      cv::hconcat(image_,image_2->image_,collage);
      cv::Mat_< T > resized_image;
      cv::resize(collage, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
      cv::imshow(name, collage);
    }

    inline Image* clone(const std::string& new_name) const{
      Image<T>* new_img = new Image<T>(new_name);
      new_img->image_ = image_.clone();
      return new_img;
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

    inline Image< cv::Vec3f>* compute_sobel_x() const{
      Image<  cv::Vec3f >* img_sobel_x =new Image< cv::Vec3f >("fx_"+name_);

      cv::Mat_<float> kernel(3,3);
      kernel <<  1,  0, -1,
                 2,  0, -2,
                 1,  0, -1;

      std::vector<cv::Mat> channels_o(3);
      std::vector<cv::Mat> channels_i(3);

      split(image_, channels_i);

      filter2D(channels_i[0], channels_o[0], CV_32FC3, kernel);
      filter2D(channels_i[1], channels_o[1], CV_32FC3, kernel);
      filter2D(channels_i[2], channels_o[2], CV_32FC3, kernel);

      /// Merge the three channels
      merge(channels_o, img_sobel_x->image_);

      return img_sobel_x;
    }

    inline Image<cv::Vec3f>* compute_sobel_y() const{
      Image< cv::Vec3f >* img_sobel_x =new Image< cv::Vec3f >("fy_"+name_);

      cv::Mat_<float> kernel(3,3);
      kernel <<   1,  2,  1,
                  0,  0,  0,
                 -1, -2, -1;

      std::vector<cv::Mat> channels_o(3);
      std::vector<cv::Mat> channels_i(3);

      split(image_, channels_i);

      filter2D(channels_i[0], channels_o[0], CV_32FC3, kernel);
      filter2D(channels_i[1], channels_o[1], CV_32FC3, kernel);
      filter2D(channels_i[2], channels_o[2], CV_32FC3, kernel);

      /// Merge the three channels
      merge(channels_o, img_sobel_x->image_);

      return img_sobel_x;
    }

    inline Image<cv::Vec3f>* squared() const{
      Image< cv::Vec3f >* squared =new Image< cv::Vec3f >("^2_"+name_);

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

};

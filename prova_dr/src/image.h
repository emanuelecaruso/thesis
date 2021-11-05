#pragma once
#include "defs.h"

using namespace pr;

template<class T>
class Image{
  public:
    std::string name_;
    cv::Mat_< T > image_;
    Image(std::string name){
      name_ = name;
    };

    inline void initImage(const int rows, const int cols){
      image_ = cv::Mat_< T >(rows ,cols);
    }

    inline void setAllPixels( const T& color){
      image_ = color;
    }

    inline void loadJpg(const std::string& path){
      image_ = cv::imread(path);
      if(image_.empty())
      std::cout << "Could not read the image: " << path << std::endl;
    }

    inline void show(int image_scale=1, const std::string& name_ext=""){
      cv::Mat_< T > resized_image;
      cv::resize(image_, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST );
      cv::imshow(name_+name_ext, resized_image);

    }


    inline Image* clone(const std::string& new_name){
      Image<T>* new_img = new Image<T>(new_name);
      new_img->image_ = image_.clone();
      return new_img;
    }

    inline bool evalPixel(const Eigen::Vector2i& pixel_coords, T& color){
      if (pixel_coords.y()>=0 && pixel_coords.y()<image_.rows && pixel_coords.x()>=0 && pixel_coords.x()<image_.cols)
      {
        color = image_.template at<T>(pixel_coords.y(),pixel_coords.x());
        return true;
      }
      return false;
    }

    inline bool evalPixel(const int row, const int col, T& color){
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

};

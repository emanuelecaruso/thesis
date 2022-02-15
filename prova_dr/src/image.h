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

    inline Image<T>* returnImgForGradientVisualization(const std::string& name){
      Image<T>* new_img = this->clone();
      new_img->image_/=cv::Scalar(2);
      new_img->image_+=cv::Scalar(0.5);
      return new_img;
    }

    inline Image<colorRGB>* returnColoredImgFromIntensityImg(const std::string& new_name) const{

      Image<colorRGB>* new_img = new Image<colorRGB>(new_name);
      cv::cvtColor(image_, new_img->image_, cv::COLOR_GRAY2BGR);
      return new_img;
    }

    inline bool pixelInRange(const pxl& pixel_coords) const{
      if (pixel_coords.y()>0.5 && pixel_coords.y()<image_.rows-0.5 && pixel_coords.x()>0.5 && pixel_coords.x()<image_.cols-0.5){
        return true;
      }
      return false;
    }

    inline bool pixelInRange(const int row, const int col) const{
      if (row>=0 && row<image_.rows && col>=0 && col<image_.cols){
        return true;
      }
      return false;
    }

    inline T evalPixel(const pxl& pixel_coords) const{
      T color;
      if(pixelInRange(pixel_coords))
      {
        color = image_.template at<T>((int)(pixel_coords.y()-0.5),(int)(pixel_coords.x()-0.5));
      }
      return color;
    }

    inline T evalPixelBilinear(const pxl& pixel_coords) const{
      T color;
      if(pixelInRange(pixel_coords))
      {
        float x1 = trunc(pixel_coords.x());
        float x2 = x1+1;
        float y1 = trunc(pixel_coords.y());
        float y2 = y1+1;
        float x_x1 = pixel_coords.x()-x1;
        float x2_x = 1-x_x1;
        float y_y1 = pixel_coords.y()-y1;
        float y2_y = 1-y_y1;
        T q11 = image_.template at<T>(y1,x1);
        T q21 = image_.template at<T>(y1,x2);
        T q12 = image_.template at<T>(y2,x1);
        T q22 = image_.template at<T>(y2,x2);
        T r1  = q11 * x2_x + q21 * x_x1;
        T r2  = q12 * x2_x + q22 * x_x1;
        color = r1 * y2_y + r2 * y_y1 ;

        // R1(x, y) = Q11 · (x2 – x) / (x2 – x1) + Q21 · (x – x1) / (x2 – x1)
        // R2(x, y) = Q12 · (x2 – x) / (x2 – x1) + Q22 · (x – x1) / (x2 – x1)
        // P(x, y) = R1 · (y2 – y) / (y2 – y1) + R2 · (y – y1) / (y2 – y1)

      }
      return color;
    }


    inline bool evalPixelBilinear(const pxl& pixel_coords, T& color) const{

      if(pixelInRange(pixel_coords))
      {
        float x1 = trunc(pixel_coords.x());
        float x2 = x1+1;
        float y1 = trunc(pixel_coords.y());
        float y2 = y1+1;
        float x_x1 = pixel_coords.x()-x1;
        float x2_x = 1-x_x1;
        float y_y1 = pixel_coords.y()-y1;
        float y2_y = 1-y_y1;
        T q11 = image_.template at<T>(y1,x1);
        T q21 = image_.template at<T>(y1,x2);
        T q12 = image_.template at<T>(y2,x1);
        T q22 = image_.template at<T>(y2,x2);
        T r1  = q11 * x2_x + q21 * x_x1;
        T r2  = q12 * x2_x + q22 * x_x1;
        color = r1 * y2_y + r2 * y_y1 ;

        // R1(x, y) = Q11 · (x2 – x) / (x2 – x1) + Q21 · (x – x1) / (x2 – x1)
        // R2(x, y) = Q12 · (x2 – x) / (x2 – x1) + Q22 · (x – x1) / (x2 – x1)
        // P(x, y) = R1 · (y2 – y) / (y2 – y1) + R2 · (y – y1) / (y2 – y1)
        return true;
      }
      return false;
    }

    inline T evalPixel(const int row, const int col) const{
      T color;
      if(pixelInRange(row, col))
      {
        color = image_.template at<T>(row,col);
      }
      return color;
    }

    inline bool evalPixel(const pxl& pixel_coords, T& color) const{
      if(pixelInRange(pixel_coords))
      {
        color = image_.template at<T>((int)(pixel_coords.y()-0.5),(int)(pixel_coords.x()-0.5));
        return true;
      }
      return false;
    }

    inline bool evalPixel(const int row, const int col, T& color) const{
      if(pixelInRange(row, col))
      {
        color = image_.template at<T>(row,col);
        return true;
      }
      return false;
    }

    inline bool setPixel(const pxl& pixel_coords, const T& color){
      if(pixelInRange(pixel_coords))
      {
        image_.template at<T>((int)(pixel_coords.y()-0.5),(int)(pixel_coords.x()-0.5)) = color;
        return true;
      }
      return false;
    }

    inline bool setPixel(const int row, const int col, const T& color){
      if(pixelInRange(row, col))
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


    inline Image< pixelIntensity>* compute_sobel_x(const std::string& name) const{
      Image<  pixelIntensity >* img_sobel_x=new Image< pixelIntensity >(name);

      cv::Mat_<float> kernel(3,3);
      kernel <<  -1,  0, 1,
                 -2,  0, 2,
                 -1,  0, 1;
      // normalize
      kernel/=4;

      // cv::Mat_<float> kernel(5,5);
      // kernel <<  -2, -1,  0, 1, 2,
      //            -2, -1,  0, 1, 2,
      //            -4, -2,  0, 2, 4,
      //            -2, -1,  0, 1; 2,
      //            -2, -1,  0, 1, 2;
      //
      // // normalize
      // kernel/=18;

      // kernel <<  -1,  0, 1,
      //            -1,  0, 1,
      //            -1,  0, 1;
      // // normalize
      // kernel/=3;

      // kernel <<   0,  0, 0,
      //            -1,  0, 1,
      //             0,  0, 0;

      filter2D(image_, img_sobel_x->image_, pixelIntensity_CODE, kernel);

      return img_sobel_x;
    }

    inline Image<pixelIntensity>* compute_sobel_y(const std::string& name) const{
      Image< pixelIntensity >* img_sobel_y =new Image< pixelIntensity >(name);

      cv::Mat_<float> kernel(3,3);
      kernel <<  -1, -2, -1,
                  0,  0,  0,
                  1,  2,  1;
      // normalize
      kernel/=4;

      // cv::Mat_<float> kernel(5,5);
      // kernel <<  -2, -2, -4, -2, -2,
      //            -1, -1, -2, -1, -1,
      //             0,  0,  0,  0,  0,
      //             1,  1,  2,  1,  1,
      //             2,  2,  4,  2,  2;
      // // normalize
      // kernel/=18;

      // kernel <<  -1, -1, -1,
      //             0,  0,  0,
      //             1,  1,  1;
      // // normalize
      // kernel/=3;

      // kernel <<   0, -1,  0,
      //             0,  0,  0,
      //             0,  1,  0;

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

    inline void drawCircle(colorRGB color, cv::Point2f point, int radius=2, int thickness=2){
      cv::circle	(	image_,point,radius,color, thickness);
    }

    inline void drawCircle(colorRGB color, pxl point, int radius=2, int thickness=2){
      cv::Point2f point_(point.x(),point.y());
      cv::circle	(	image_,point_ ,radius,color, thickness);
    }

    inline void drawRectangle(cv::Rect rect, colorRGB color, int type, float alpha=1){
      cv::Mat roi = image_(rect);
      cv::Mat clr(roi.size(), colorRGB_CODE, color);
      cv::addWeighted(clr, alpha, roi, 1.0 - alpha , 0.0, roi);
      // cv::rectangle(image_,rect, color, type);
    }


    inline void showImgWithColoredPixel(const pxl pixel, float size, const std::string& name) const{
      Image< colorRGB >* show_image = returnColoredImgFromIntensityImg(name);
      show_image->setPixel( pixel, red);
      show_image->show(size, name);
    }

    inline void showImgWithCircledPixel(const pxl pixel, float size, const std::string& name, int radius=2, int thickness=2) const{
      Image< colorRGB >* show_image = returnColoredImgFromIntensityImg(name);
      // show_image->setPixel( pixel, red);
      show_image->drawCircle(red, pixel, radius, thickness);
      show_image->show(size, name);
    }

    inline void destroyWindow() const{
      cv::destroyWindow(name_);
    }
};

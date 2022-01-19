#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
#include <unistd.h>
#include "opencv2/opencv.hpp"
#include <sys/types.h>
#include <dirent.h>
#include <mutex>


namespace pr {

  //ds opencv keys
  #define OPENCV_KEY_UP 2490368
  #define OPENCV_KEY_DOWN 2621440
  #define OPENCV_KEY_LEFT 2424832
  #define OPENCV_KEY_RIGHT 2555904
  #define OPENCV_KEY_SPACE 32
  #define OPENCV_KEY_DELETE 3014656
  #define OPENCV_KEY_ESCAPE 27

  typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
  typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
  typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > Matrix3fVector;
  typedef std::vector<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f> > Matrix2fVector;


  typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
  typedef Eigen::Matrix<float, 4, 6> Matrix4_6f;
  typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
  typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  typedef Eigen::Matrix<float, 6, 1> Vector6f;

  typedef Eigen::Matrix<float, 9, 6> Matrix9_6f;
  typedef Eigen::Matrix<float, 9, 9> Matrix9f;
  typedef Eigen::Matrix<float, 9, 1> Vector9f;

  typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;
  typedef Eigen::Matrix<float, 6, 2> Matrix6_2f;
  typedef Eigen::Matrix<float, 4, 3> Matrix4_3f;



  template <class T>
  bool isNan(const T& m){
    for (int i=0; i< m.rows(); i++) {
      for (int j=0; j< m.cols(); j++) {
	float v = m(i,j);
	if ( std::isnan( v ) )
	  return true;
      }
    }
    return false;
  }


  inline Eigen::Isometry3f v2t(const Vector6f& t){
    Eigen::Isometry3f T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
      w=sqrt(1-w);
      T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
      T.linear().setIdentity();
    }
    return T;
  }

  inline Vector6f t2v(const Eigen::Isometry3f& t){
    Vector6f v;
    v.head<3>()=t.translation();
    Eigen::Quaternionf q(t.linear());
    v.block<3,1>(3,0)=q.matrix().block<3,1>(1,0);
    if (q.w()<0)
      v.block<3,1>(3,0) *= -1.0f;
    return v;
  }

  inline Eigen::Isometry2f v2t(const Eigen::Vector3f& t){
    Eigen::Isometry2f T;
    T.setIdentity();
    T.translation()=t.head<2>();
    float c = cos(t(2));
    float s = sin(t(2));
    T.linear() << c, -s, s, c;
    return T;
  }

  inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
    Eigen::Vector3f v;
    v.head<2>()=t.translation();
    v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
    return v;
  }

  inline Eigen::Matrix3f Rx(float rot_x){
    float c=cos(rot_x);
    float s=sin(rot_x);
    Eigen::Matrix3f R;
    R << 1,  0, 0,
      0,  c,  -s,
      0,  s,  c;
    return R;
  }

  inline Eigen::Matrix3f Ry(float rot_y){
    float c=cos(rot_y);
    float s=sin(rot_y);
    Eigen::Matrix3f R;
    R << c,  0,  s,
      0 , 1,  0,
      -s,  0, c;
    return R;
  }

  inline Eigen::Matrix3f Rz(float rot_z){
    float c=cos(rot_z);
    float s=sin(rot_z);
    Eigen::Matrix3f R;
    R << c,  -s,  0,
      s,  c,  0,
      0,  0,  1;
    return R;
  }


  inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
    Eigen::Isometry3f T;
    T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
    T.translation()=v.head<3>();
    return T;
  }


  inline Eigen::Matrix3f skew(const Eigen::Vector3f& v){
    Eigen::Matrix3f S;
    S << 0, -v[2], v[1],
      v[2], 0, -v[0],
      -v[1], v[0], 0;
    return S;
  }




  /** \typedef UnsignedCharImage
   * \brief An unsigned char cv::Mat.
   */
  typedef cv::Mat_<unsigned char> UnsignedCharImage;

  /** \typedef CharImage
   * \brief A char cv::Mat.
   */
  typedef cv::Mat_<char> CharImage;

  /** \typedef UnsignedShortImage
   * \brief An unsigned short cv::Mat.
   */
  typedef cv::Mat_<unsigned short> UnsignedShortImage;

  /** \typedef UnsignedIntImage
   * \brief An unsigned int cv::Mat.
   */
  typedef cv::Mat_<unsigned int> UnsignedIntImage;

  /** \typedef IntImage
   * \brief An int cv::Mat.
   */
  typedef cv::Mat_<int> IntImage;

  /** \typedef FloatImage
   * \brief A float cv::Mat.
   */
  typedef cv::Mat_<float> FloatImage;

  /** \typedef Float3Image
   * \brief A float cv::Mat.
   */
  typedef cv::Mat_<cv::Vec3f> Float3Image;

  /** \typedef DoubleImage
   * \brief A double cv::Mat.
   */
  typedef cv::Mat_<double> DoubleImage;

  /** \typedef RawDepthImage
   * \brief An unsigned char cv::Mat used to for depth images with depth values expressed in millimeters.
   */
  typedef UnsignedShortImage RawDepthImage;

  /** \typedef IndexImage
   * \brief An int cv::Mat used to save the indeces of the points of a depth image inside a vector of points.
   */
  typedef IntImage IndexImage;

  /** \typedef DepthImage
   * \brief A float cv::Mat used to for depth images with depth values expressed in meters.
   */
  typedef cv::Mat_< cv::Vec3b > RGBImage;

  typedef cv::Mat_< uchar > GreyImage;

  typedef std::vector< cv::Vec3b > RGBVector;

  typedef std::vector<int> IntVector;

  typedef std::vector<float> FloatVector;

  typedef std::pair<int,int> IntPair;

  typedef std::vector<IntPair > IntPairVector;



  // inline float extractRollAngle(Eigen::Isometry3f& T){
  //   Eigen::Matrix3f R=T.linear();
  //   float r10 = R(1,0);
  //   float r00 = R(0,0);
  //   return atan2(r10,r00);
  // }

  #define PI 3.14159265

  struct CamParameters{
    const int resolution_x;
    const int resolution_y;
    const float width;
    const float height;
    const float aspect;
    const float lens;
    const float min_depth;
    const float max_depth;
    const float pixel_width;

    CamParameters(int resolution_x_, int resolution_y_,
      float width_,float height_,float aspect_,float lens_,float min_depth_,float max_depth_):
    resolution_x(resolution_x_), resolution_y(resolution_y_), width(width_), height(height_),
    aspect(aspect_), lens(lens_), min_depth(min_depth_), max_depth(max_depth_),
    pixel_width(width/(float)resolution_x)
    { };

    void printMembers() const {
      std::cout << "lens: " << lens << std::endl;
      std::cout << "aspect: " << aspect << std::endl;
      std::cout << "width: " << width << std::endl;
      std::cout << "height: " << height << std::endl;
      std::cout << "resolution_x: " << resolution_x << std::endl;
      std::cout << "resolution_y: " << resolution_y << std::endl;
      std::cout << "max_depth: " << max_depth << std::endl;
      std::cout << "min_depth: " << min_depth << std::endl;
    }

  };


  static bool debug_mode = 1;
  static std::mutex mu_cout;

  typedef cv::Vec3f colorRGB;
  const float colorRGB_maxval = 1;
  const int colorRGB_CODE = CV_32FC3;


  typedef float pixelIntensity;
  const float pixelIntensity_maxval = 1;
  const int pixelIntensity_CODE = CV_32FC1;


  struct Cp // Colored point (in 3D)
  {
    Eigen::Vector3f point;
    cv::Vec3b color;
  };



  inline void sharedCout(const std::string& msg){
    std::lock_guard<std::mutex> locker(mu_cout);
    std::cout << msg << std::endl;
  }

  inline void sharedCoutDebug(const std::string& msg){
    std::lock_guard<std::mutex> locker(mu_cout);
    if (debug_mode){
      std::cout << msg << std::endl;
    }
  }

  inline float l1Norm(cv::Vec3f vec){
    return abs(vec[0])+abs(vec[1])+abs(vec[2]);
  }

  const colorRGB black = colorRGB(0,0,0);
  const colorRGB white = colorRGB(colorRGB_maxval,colorRGB_maxval,colorRGB_maxval);
  const colorRGB grey = colorRGB(colorRGB_maxval/2,colorRGB_maxval/2,colorRGB_maxval/2);
  const colorRGB magenta = colorRGB(colorRGB_maxval,0,colorRGB_maxval);
  const colorRGB red = colorRGB(0,0,colorRGB_maxval);
  const colorRGB green = colorRGB(0,colorRGB_maxval,0);
  const colorRGB blue = colorRGB(colorRGB_maxval,0,0);
  const colorRGB cyan = colorRGB(colorRGB_maxval,colorRGB_maxval,0);
  const colorRGB yellow = colorRGB(0,colorRGB_maxval,colorRGB_maxval);

  const colorRGB red_ = colorRGB(colorRGB_maxval/2,colorRGB_maxval/2,colorRGB_maxval);
  const colorRGB green_ = colorRGB(colorRGB_maxval/2,colorRGB_maxval,colorRGB_maxval/2);
  const colorRGB blue_ = colorRGB(colorRGB_maxval,colorRGB_maxval/2,colorRGB_maxval/2);
  const colorRGB cyan_ = colorRGB(colorRGB_maxval,colorRGB_maxval,colorRGB_maxval/2);
  const colorRGB magenta_ = colorRGB(colorRGB_maxval,colorRGB_maxval/2,colorRGB_maxval);

  inline float radiansSub(float rad1, float rad2){
    float sub = rad1-rad2;
    if (sub<-PI){
      sub+=2*PI;
    }else if(sub>PI){
      sub-=2*PI;
    }
    return sub;
  }

  inline float squareNorm(float a){
    return pow(a,2);
  }

  inline float squareNormDerivative(float a){
    return 2*a;
  }

  inline float huberNorm(float a, float b){
    float huber_norm;
    if (abs(a)<=b){
      huber_norm= (pow(a,2))/(2*b);
    }else{
      huber_norm= abs(a)-b/2;
    }
    return huber_norm;
  }

  inline float huberNormDerivative(float a, float b){
    float huber_norm_der;
    if (abs(a)<=b){
      huber_norm_der= a/b;
    }else if (a>0){
      huber_norm_der= 1;
    }else if (a<0){
      huber_norm_der= -1;
    }
    return huber_norm_der;
  }

  inline int lowerBound(std::vector<int> const& vec, int value) {
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }

    return *it;
  }

  inline int upperBound(std::vector<int> const& vec, int value) {
    auto const it = std::upper_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }

    return *it;
  }
}

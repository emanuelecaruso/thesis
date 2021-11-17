#pragma once
#include "defs.h"
#include "json.hpp"
// #include "image.h"
#include "wavelet.h"
#include <sys/stat.h>

using namespace pr;

class CameraForStudy;

class Camera{
  public:

    const std::string name_;
    const CamParameters* cam_parameters_;
    const Eigen::Matrix3f* K_;
    const Eigen::Matrix3f* Kinv_;
    const Image<cv::Vec3b>* image_rgb_;
    Image<float>* invdepth_map_;
    Eigen::Isometry3f* frame_world_wrt_camera_;
    Eigen::Isometry3f* frame_camera_wrt_world_;
    Wvlt_dec* wavelet_dec_;

    Camera(const std::string& name, const CamParameters* cam_parameters,
           const Image<cv::Vec3b>* image_rgb, Eigen::Isometry3f* frame_world_wrt_camera,
               Eigen::Isometry3f* frame_camera_wrt_world, int wav_levels=4 ):
           name_(name),
           cam_parameters_(cam_parameters),
           K_(compute_K()),
           Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
           image_rgb_( image_rgb )
           {
             Image<cv::Vec3f>* img = new Image<cv::Vec3f>();
             image_rgb_->image_.convertTo(img->image_, CV_32FC3, 1/255.0);
             wavelet_dec_= new Wvlt_dec(wav_levels,img);
             frame_world_wrt_camera_=frame_world_wrt_camera;
             frame_camera_wrt_world_=frame_camera_wrt_world;
           };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           const std::string& path_rgb, int wav_levels=4 ):
           name_(name),
           cam_parameters_(cam_parameters),
           K_(compute_K()),
           Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
           image_rgb_( returnRGBFromPath( path_rgb ) )
           {
             Image<cv::Vec3f>* img = new Image<cv::Vec3f>();
             image_rgb_->image_.convertTo(img->image_, CV_32FC3, 1/255.0);
             wavelet_dec_= new Wvlt_dec(wav_levels,img);
           };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb ):
    Camera( name,cam_parameters, path_rgb )
    {
      loadPoseFromJsonVal(f);
      invdepth_map_ = new Image< float >("invdepth_"+name_);
      loadWhiteDepth();
    };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb,  const std::string& path_depth ):
    Camera( name,cam_parameters, f, path_rgb )
    {
      loadDepthMap(path_depth);
    };



    void printMembers() const;

    // sampling
    void sampleRandomUv(Eigen::Vector2f& uv);
    void sampleRandomPixel(Eigen::Vector2i& pixel_coords);

    // access
    void getCenterAsUV(Eigen::Vector2f& uv);
    void getCentreAsPixel(Eigen::Vector2i& pixel_coords);

    // functions for projections/transformations
    void pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords) const;
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv) const;

    // functions for images
    void clearImgs();
    void saveRGB(const std::string& path) const;
    void saveDepthMap(const std::string& path) const;
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;

    Image<cv::Vec3b>* setPixelAndReturnImg(const Eigen::Vector2i& pixel_coords,const std::string& name);

    inline Camera* clone(){
      return new Camera(*this);
    }
  protected:
    Eigen::Matrix3f* compute_K();
    Image<cv::Vec3b>* returnRGBFromPath(const std::string& path_rgb);
    void loadWhiteDepth();
    void loadDepthMap(const std::string& path);
    void loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f);



};

class CameraForStudy: public Camera{
  public:
    const Image<cv::Vec3f>* curvature_;
    const Image<cv::Vec3f>* grad_x_;
    const Image<cv::Vec3f>* grad_y_;
    const Image<cv::Vec3f>* grad_robust_x_;
    const Image<float>* grad_intensity_;

    CameraForStudy(const std::string& name, const CamParameters* cam_parameters,
           const std::string& path_rgb):
           Camera( name, cam_parameters, path_rgb),
           curvature_( computeCurvature(100) ),
           grad_x_( gradientX() ),
           grad_y_( gradientY() ),
           grad_robust_x_( gradientRobustX() ),
           grad_intensity_( gradientintensity() )
           { };

    CameraForStudy(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb):
           CameraForStudy( name,cam_parameters, path_rgb )
           {
             loadPoseFromJsonVal(f);
             invdepth_map_ = new Image< float >("invdepth_"+name_);
             loadWhiteDepth();
           };

    CameraForStudy(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb,  const std::string& path_depth ):
           CameraForStudy( name,cam_parameters, f, path_rgb )
           {
             loadDepthMap(path_depth);
           };

  private:
    //feature types
    Image<cv::Vec3f>* computeCurvature(float gain=1);
    Image<cv::Vec3f>* gradientX();
    Image<cv::Vec3f>* gradientY();
    Image<cv::Vec3f>* gradientRobustX();
    Image<float>* gradientintensity();

};

#pragma once
#include "defs.h"
#include "image.h"

using namespace pr;

struct Pyramid{
  public:
    Image<cv::Vec3b>* pyr_2;
    Image<cv::Vec3b>* pyr_4;
    Image<cv::Vec3b>* pyr_8;

    Pyramid(Image<cv::Vec3b>* image_rgb){
      // https://docs.opencv.org/3.4.15/d4/d1f/tutorial_pyramids.html TODO
    };
  // private:
  //   void makePyramid();

};

class Camera{
  public:

    const std::string name_;
    const CamParameters* cam_parameters_;
    const Eigen::Matrix3f* K_;
    const Eigen::Matrix3f* Kinv_;
    Image<float>* invdepth_map_;
    Image<cv::Vec3b>* image_rgb_;
    Eigen::Isometry3f* frame_world_wrt_camera_;
    Eigen::Isometry3f* frame_camera_wrt_world_;

    Camera(const std::string& name, const CamParameters* cam_parameters, Eigen::Isometry3f* frame_camera_wrt_world):
    name_(name),
    cam_parameters_(cam_parameters),
    K_(compute_K()),
    Kinv_( new Eigen::Matrix3f(K_->inverse()) )

    {

       float resolution_x = cam_parameters->resolution_x;
       float resolution_y = cam_parameters->resolution_y;
       float max_depth = cam_parameters->max_depth;
       float min_depth = cam_parameters->min_depth;

       Eigen::Isometry3f* frame_world_wrt_camera = new Eigen::Isometry3f;
       *frame_world_wrt_camera=frame_camera_wrt_world->inverse();

       frame_camera_wrt_world_ = frame_camera_wrt_world;
       frame_world_wrt_camera_ = frame_world_wrt_camera;

       invdepth_map_ = new Image< float >("invdepth_"+name_);
       image_rgb_ = new Image< cv::Vec3b >("rgb_"+name_);

       // initialize images with white color
       invdepth_map_->initImage(resolution_y,resolution_x);
       invdepth_map_->setAllPixels(1.0);
       image_rgb_->initImage(resolution_y,resolution_x);
       image_rgb_->setAllPixels(cv::Vec3b(255,255,255));

    };

    void printMembers() const;

    // sampling
    void sampleRandomUv(Eigen::Vector2f& uv);
    void sampleRandomPixel(Eigen::Vector2i& pixel_coords);

    // access
    void getCentreAsUV(Eigen::Vector2f& uv);
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
    void loadRGB(const std::string& path);
    void loadDepthMap(const std::string& path);
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;

    void showWorldFrame(Eigen::Vector3f& origin, float delta,int length);

    inline Camera* clone(){
      return new Camera(*this);
    }
  private:
    Eigen::Matrix3f* compute_K();

};

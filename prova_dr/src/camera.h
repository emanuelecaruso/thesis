#pragma once
#include "defs.h"
#include "json.hpp"
// #include "image.h"
#include "wavelet.h"
#include <sys/stat.h>

using namespace pr;

typedef std::pair<float,float> bound;

class Candidate{
  public:

    Candidate(int level, Eigen::Vector2i pixel, Eigen::Vector2f uv,
              float grad_magnitude, colorRGB grad3C_magnitude, colorRGB color,
              std::vector<bound>* bounds):
    level_(level), pixel_(pixel), uv_(uv), grad_magnitude_(grad_magnitude),
    grad3C_magnitude_(grad3C_magnitude), color_(color), bounds_(bounds)
    {}

    const int level_;
    const Eigen::Vector2i pixel_;
    const Eigen::Vector2f uv_;
    const float grad_magnitude_;
    const colorRGB grad3C_magnitude_;
    const colorRGB color_;
    std::vector<bound>* bounds_;


};

typedef std::vector<Candidate*> Region;

class Camera{
  public:

    const std::string name_;
    const CamParameters* cam_parameters_;
    const Eigen::Matrix3f* K_;
    const Eigen::Matrix3f* Kinv_;
    const Image<colorRGB>* image_rgb_;
    Image<float>* invdepth_map_;
    Eigen::Isometry3f* frame_camera_wrt_world_;
    Eigen::Isometry3f* frame_world_wrt_camera_;


    Camera(const std::string& name, const CamParameters* cam_parameters,
           const Image<colorRGB>* image_rgb):
           name_(name),
           cam_parameters_(cam_parameters),
           K_(compute_K()),
           Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
           image_rgb_( image_rgb ),
           frame_camera_wrt_world_(new Eigen::Isometry3f),
           frame_world_wrt_camera_(new Eigen::Isometry3f)
           { };

     Camera(const std::string& name, const CamParameters* cam_parameters,
            const std::string& path_rgb):
            name_(name),
            cam_parameters_(cam_parameters),
            K_(compute_K()),
            Kinv_( new Eigen::Matrix3f(K_->inverse()) ),
            image_rgb_( returnRGBFromPath( path_rgb ) ),
            frame_camera_wrt_world_(new Eigen::Isometry3f),
            frame_world_wrt_camera_(new Eigen::Isometry3f)
            { };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           const Image<colorRGB>* image_rgb, Eigen::Isometry3f* frame_world_wrt_camera,
               Eigen::Isometry3f* frame_camera_wrt_world ):
           Camera(name, cam_parameters, image_rgb )
           {
             frame_world_wrt_camera_=frame_world_wrt_camera;
             frame_camera_wrt_world_=frame_camera_wrt_world;
           };

    Camera(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb):
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
    void pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv, int level) const;
    void pixelCoords2uv(const Eigen::Vector2i& pixel_coords, Eigen::Vector2f& uv) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords, int level) const;
    void uv2pixelCoords(const Eigen::Vector2f& uv, Eigen::Vector2i& pixel_coords) const;
    void pointAtDepth(const Eigen::Vector2f& uv, float depth, Eigen::Vector3f& p) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv, float& p_cam_z ) const;
    bool projectPoint(const Eigen::Vector3f& p, Eigen::Vector2f& uv) const;
    bool projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv) const;
    bool projectCam(const Camera* cam_to_be_projected, Eigen::Vector2f& uv, float& p_cam_z) const;

    // functions for images
    void clearImgs();
    void saveRGB(const std::string& path) const;
    void saveDepthMap(const std::string& path) const;
    void showRGB(int image_scale=1) const;
    void showDepthMap(int image_scale=1) const;


    inline Camera* clone(){
      return new Camera(*this);
    }
  protected:
    Eigen::Matrix3f* compute_K();
    Image<colorRGB>* returnRGBFromPath(const std::string& path_rgb);
    void loadWhiteDepth();
    void loadDepthMap(const std::string& path);
    void loadPoseFromJsonVal(nlohmann::basic_json<>::value_type f);



};

class CameraForStudy: public Camera{
  public:
    const Image<colorRGB>* curvature_;
    const Image<colorRGB>* grad_x_;
    const Image<colorRGB>* grad_y_;
    const Image<colorRGB>* grad_robust_x_;
    const Image<float>* grad_intensity_;
    Wvlt_dec* wavelet_dec_;

    CameraForStudy(const std::string& name, const CamParameters* cam_parameters,
           const std::string& path_rgb, int wav_levels):
           Camera( name, cam_parameters, path_rgb),
           curvature_( computeCurvature(100) ),
           grad_x_( gradientX() ),
           grad_y_( gradientY() ),
           grad_robust_x_( gradientRobustX() ),
           grad_intensity_( gradientintensity() ),
           wavelet_dec_(new Wvlt_dec(wav_levels,new Image<colorRGB>(image_rgb_)))
           { };

    CameraForStudy(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb, int wav_levels):
           CameraForStudy( name,cam_parameters, path_rgb, wav_levels )
           {
             loadPoseFromJsonVal(f);
             invdepth_map_ = new Image< float >("invdepth_"+name_);
             loadWhiteDepth();
           };

    CameraForStudy(const std::string& name, const CamParameters* cam_parameters,
           nlohmann::basic_json<>::value_type f,
           const std::string& path_rgb,  const std::string& path_depth, int wav_levels ):
           CameraForStudy( name,cam_parameters, f, path_rgb, wav_levels )
           {
             loadDepthMap(path_depth);
           };

  private:
    //feature types
    Image<colorRGB>* computeCurvature(float gain=1);
    Image<colorRGB>* gradientX();
    Image<colorRGB>* gradientY();
    Image<colorRGB>* gradientRobustX();
    Image<float>* gradientintensity();

};

class Mapper; // forward declaration

class CameraForMapping: public Camera{

  public:


    Wvlt_dec* wavelet_dec_;
    std::vector<Region*>* regions_;
    std::vector<Candidate*>* candidates_;
    int n_candidates_;
    friend class Mapper;
    friend class Dtam;

    CameraForMapping(const std::string& name, const CamParameters* cam_parameters,
           const Image<colorRGB>* image_rgb, int wav_levels):
           Camera( name, cam_parameters, image_rgb),
           wavelet_dec_(new Wvlt_dec(wav_levels,new Image<colorRGB>(image_rgb_))),
           regions_(new std::vector<Region*>),
           candidates_(new std::vector<Candidate*>),
           n_candidates_(0)
           {  };

    CameraForMapping(const std::string& name, const CamParameters* cam_parameters,
            const Image<colorRGB>* image_rgb, Eigen::Isometry3f* frame_world_wrt_camera,
                Eigen::Isometry3f* frame_camera_wrt_world, int wav_levels):
            CameraForMapping( name, cam_parameters, image_rgb, wav_levels)
            {
              frame_world_wrt_camera_=frame_world_wrt_camera;
              frame_camera_wrt_world_=frame_camera_wrt_world;
            };

    CameraForMapping(Camera* cam, int wav_levels):
           CameraForMapping( cam->name_, cam->cam_parameters_, cam->image_rgb_, wav_levels)
           {  };

  private:
    void collectRegions(float grad_threshold);
    void selectNewCandidates(int max_num_candidates);
    void showCandidates_1(float size);
    void showCandidates_2(float size);


};

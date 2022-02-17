#pragma once
#include "camera.h"
#include "image.h"
#include "environment.h"

class Dtam; //forward declaration
class JacobiansAndError;


class deltaUpdateIncrements{
  public:
    Eigen::VectorXf* dx_poses;
    Eigen::VectorXf* dx_points;

    deltaUpdateIncrements(Eigen::VectorXf* dx_poses_, Eigen::VectorXf* dx_points_):
    dx_poses(dx_poses_),
    dx_points(dx_points_){}
};

class JacobiansAndError{
  public:
    Eigen::Matrix<float,1,6>* J_r;
    Eigen::Matrix<float,1,6>* J_m;
    float J_d;
    Eigen::Matrix<float,6,1>* J_r_transp;
    Eigen::Matrix<float,6,1>* J_m_transp;
    float weight_total;

    ActivePoint* active_pt;
    CameraForMapping* cam_m;

    float error;

    JacobiansAndError(Eigen::Matrix<float,1,6>* J_r_, Eigen::Matrix<float,1,6>* J_m_, float J_d_,
                      CameraForMapping* cam_m_, ActivePoint* active_pt_, float error_, float weight_total_ ):
    J_r(J_r_),
    J_m(J_m_),
    J_d(J_d_),
    J_r_transp(new Eigen::Matrix<float,6,1>),
    J_m_transp(new Eigen::Matrix<float,6,1>),

    active_pt(active_pt_),
    cam_m(cam_m_),

    error(error_),
    weight_total(weight_total_)
    {
      *J_r_transp=J_r_->transpose();
      *J_m_transp=J_m_->transpose();
    }

    ~JacobiansAndError(){
      delete J_r;
      delete J_m;
      delete J_r_transp;
      delete J_m_transp;
    }

};

class HessianAndB_base{
public:

  HessianAndB_base(int pose_block_size_, int point_block_size_ ):

  // block sizes
  pose_block_size(pose_block_size_),
  point_block_size(point_block_size_),

  // initialize H blocks
  H_pose_pose(new Eigen::MatrixXf(pose_block_size_,pose_block_size_) ),
  H_pose_point(new Eigen::MatrixXf(pose_block_size_,point_block_size_)),
  H_point_pose(new Eigen::MatrixXf(point_block_size_,pose_block_size_)),
  H_point_point(new Eigen::DiagonalMatrix<float,Eigen::Dynamic>(point_block_size_) ),

  // initialize b blocks
  b_pose(new Eigen::VectorXf(pose_block_size_)),
  b_point(new Eigen::VectorXf(point_block_size_))
  {
    H_pose_pose->setZero();
    H_pose_point->setZero();
    H_point_pose->setZero();
    H_point_point->setZero();
    b_pose->setZero();
    b_point->setZero();

  }

  ~HessianAndB_base(){
    delete H_pose_pose;
    delete H_pose_point;
    delete H_point_pose;
    delete H_point_point;
    delete b_pose;
    delete b_point;
  }
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* invertHPointPoint();
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* invertHPointPointDLS(float mu);

  void mirrorTriangH();

  int pose_block_size;
  int point_block_size;
  Eigen::MatrixXf* H_pose_pose;
  Eigen::MatrixXf* H_pose_point;
  Eigen::MatrixXf* H_point_pose;
  Eigen::DiagonalMatrix<float,Eigen::Dynamic>* H_point_point;
  Eigen::VectorXf* b_pose;
  Eigen::VectorXf* b_point;

};

class HessianAndB : public HessianAndB_base{
  public:

    HessianAndB(int pose_block_size_, int point_block_size_ ):
    HessianAndB_base(pose_block_size_, point_block_size_){}

    void updateHessianAndB(JacobiansAndError* jacobians_and_error );

    deltaUpdateIncrements* getDeltaUpdateIncrements();
    deltaUpdateIncrements* getDeltaUpdateIncrements_Slow();

    void visualizeH();

};

class HessianAndB_Marg : public HessianAndB_base{
  public:

    HessianAndB_Marg(int pose_block_size_, int point_block_size_ ):
    HessianAndB_base(pose_block_size_, point_block_size_){}

    void updateHessianAndB_marg(JacobiansAndError* jacobians_and_error );
    //
    // deltaUpdateIncrements* getDeltaUpdateIncrements();
    // deltaUpdateIncrements* getDeltaUpdateIncrements_Slow();
    //
    // void visualizeH();

};

class priorMarg{
  public:
    std::vector<CameraForMapping*>* activeKeyframes;
    std::vector<ActivePoint*>* marginalizedPoint;
    bool active;

    HessianAndB* hessian_b;

    // default constructor
    priorMarg(int n_cams, int n_points_marg ):
      activeKeyframes(new std::vector<CameraForMapping*>),
      marginalizedPoint(new std::vector<ActivePoint*>),
      active(false),
      hessian_b(new HessianAndB(n_cams,n_points_marg))
    {}

    void build(int n_cams, int n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec);
};

class BundleAdj{

  public:
    BundleAdj(Dtam* dtam, Params* parameters):
    debug_optimization_(false),
    dtam_(dtam),
    parameters_(parameters),
    keyframe_vector_ba_(new std::vector<int>),
    frame_current_ba(-1),
    num_active_points_(0),
    min_num_of_active_pts_per_region_(INT_MAX)
    {};


    void projectActivePoints_prepMarg(bool take_fixed_point=false);
    void projectActivePoints(bool take_fixed_point=false);
    void updateCurrentGuess();
    void updateActivePointsAfterNewPose();
    void updateActivePointsAfterNewPose(CameraForMapping* keyframe);


    ActivePointProjected* projectActivePoint(ActivePoint* active_pt, CamCouple* cam_couple);
    void marginalizeActivePoints();
    void activateNewPoints();
    void getCoarseActivePoints();
    void collectCoarseActivePoints();

    float getWeightTotal(float error);
    JacobiansAndError* getJacobiansAndError(ActivePoint* active_pt, CameraForMapping* cam_m, bool marg=false);

    void initializeStateStructure( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyM( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyR( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyD( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );
    // void initializeStateStructure_onlyMandPoints( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec );

    void initializeStateStructureMarg( int& n_cams, int& n_points, std::vector<JacobiansAndError*>* jacobians_and_error_vec, CameraForMapping* keyframe_to_marginalize );
    void marginalize();
    priorMarg* getMarginalizationPrior( int n_cams, int n_points_marg, std::vector<JacobiansAndError*>* jacobians_and_error_vec);


    float optimizationStep();
    void optimize();

    inline void addKeyframe(int idx){
      keyframe_vector_ba_->push_back(idx);
    }

    inline int getFrameCurrentIdxBA(){
      return frame_current_ba;
    }

    CameraForMapping* getFrameCurrentBA();
    bool debug_optimization_;
    std::vector<int>* keyframe_vector_ba_;
    int frame_current_ba;

  private:

    Dtam* const dtam_;
    Params* const parameters_;
    int num_active_points_;
    int min_num_of_active_pts_per_region_;


    ActivePoint* activateCandidate(CandidateProjected* cand_proj, RegionWithProjCandidates* reg, RegionsWithProjActivePoints* regs);
    void addCoarseActivePointInRegion(ActivePoint* active_pt);

    // void projectCandidates(CameraForMapping* keyframe, CameraForMapping* new_keyframe);

    void updateDeltaUpdates(deltaUpdateIncrements* delta);
    void updateDeltaUpdatesOnlyD(deltaUpdateIncrements* delta);
    void fixNewTangentSpace();
    void fixNewTangentSpaceOnlyD();

    Eigen::Matrix<float,1,3>* getJfirst(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Vector3f& point_m_0, pxl& pixel_m);
    Eigen::Matrix<float,1,6>* getJr(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Matrix<float,1,3>* J_first);
    Eigen::Matrix<float,1,6>* getJm(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Matrix<float,1,3>* J_first, Eigen::Vector3f& point_m);
    float getJd(ActivePoint* active_pt, CameraForMapping* cam_m, Eigen::Matrix<float,1,3>* J_first);
    float getError(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m);
    bool getError(ActivePoint* active_pt, CameraForMapping* cam_m, pxl& pixel_m, float& error);

};

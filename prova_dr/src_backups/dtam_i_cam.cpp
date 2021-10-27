#include "dtam.h"
#include <math.h>


bool Dtam::get1stDepthWithUV(Camera* camera_r, Camera* camera_m, Eigen::Vector2f& uv_r, Eigen::Vector2f& uv_m, float& depth){

  Eigen::Isometry3f T = camera_m->frame_world_wrt_camera_*camera_r->frame_camera_wrt_world_;
  auto r=T.linear();
  auto t=T.translation();
  float f = camera_r->lens_;
  float w=camera_m->width_;
  float h=camera_m->width_/camera_m->aspect_;
  depth = (2*f*(f+t(2)))/(2*f*r(2,2)-2*r(2,0)*uv_r.x()+r(2,0)*w-r(2,1)*(h-2*uv_r.y()));
  uv_m.x() = t(0)+(w/2)-depth*r(0,2)+((depth*r(0,0)*(2*uv_r.x()-w))/(2*f))+((depth*r(0,1)*(h-2*uv_r.y()))/(2*f));
  uv_m.y() = (h/2)-t(1)+depth*r(1,2)-((depth*r(1,0)*(2*uv_r.x()-w))/(2*f))-((depth*r(1,1)*(h-2*uv_r.y()))/(2*f));
}

struct cameraData{
  Eigen::Vector2f uv1, uv2, uv1_fixed, uv2_fixed;
  float depth1_m, depth2_m, depth1_m_fixed, depth2_m_fixed;
  Eigen::Matrix3f r;
  Eigen::Vector3f t;
};

void Dtam::getDepthMap(CameraVector& camera_vector, int num_interpolations, bool check){
  int cols = camera_vector[0]->depth_map_->image_.cols;
  int rows = camera_vector[0]->depth_map_->image_.rows;

  std::vector<cameraData*> cameraData_vector;
  cameraData_vector.reserve(camera_vector.size());

  float f = camera_vector[0]->lens_;
  float w=camera_vector[0]->width_;
  float h=camera_vector[0]->width_/camera_vector[0]->aspect_;

  for (int camera_iterator=0; camera_iterator<camera_vector.size()-1; camera_iterator++){
    cameraData* camera_data = new cameraData;

    Eigen::Isometry3f T = camera_vector[0]->frame_world_wrt_camera_*camera_vector[camera_iterator+1]->frame_camera_wrt_world_;
    Eigen::Matrix3f r=T.linear();
    Eigen::Vector3f t=T.translation();
    camera_data->r=r;
    camera_data->t=t;

    cameraData_vector[camera_iterator]=camera_data;
  }




  for (int row = 0; row<rows; row++){
    for (int col = 0; col<cols; col++){

      if (check){row= rows/2; col=cols/2;}

      Eigen::Vector2i pixel_coords_r(col,row);



      //initialize costs
      std::vector<int> costs(num_interpolations, -1 );
      // std::vector<float> depths(num_interpolations, 0 );

      // reference camera
      Camera* camera_r = camera_vector[0];
      Eigen::Vector3f camera_r_p = camera_r->frame_camera_wrt_world_.translation();
      float depth1_r=camera_r->lens_;
      float depth2_r=camera_r->max_depth_;
      cv::Vec3b clr_r;
      camera_r->image_rgb_->evalPixel(pixel_coords_r,clr_r);

      // query point
      Eigen::Vector3f query_p;
      Eigen::Vector2f uv_r;
      camera_r->pixelCoords2uv(pixel_coords_r, uv_r);
      camera_r->pointAtDepth(uv_r, depth2_r, query_p);


      for (int camera_iterator=1; camera_iterator<camera_vector.size(); camera_iterator++){
        Camera* camera_m = camera_vector[camera_iterator];

        // project camera_r on camera_m
        Eigen::Vector2f cam_r_projected_on_cam_m;
        float cam_r_depth_on_camera_m;
        bool cam_r_in_front = camera_m->projectPoint(camera_r_p, cam_r_projected_on_cam_m, cam_r_depth_on_camera_m);

        // project query point
        Eigen::Vector2f query_p_projected_on_cam_m;
        float query_depth_on_camera_m;
        bool query_in_front = camera_m->projectPoint(query_p, query_p_projected_on_cam_m, query_depth_on_camera_m);

        // initializations
        Eigen::Vector2f uv1, uv2, uv1_fixed, uv2_fixed;
        float depth1_m, depth2_m, depth1_m_fixed, depth2_m_fixed;
        bool resized1, resized2;

        if (check){ cv::Vec3b clr = cv::Vec3b(0,0,255);
          camera_r->image_rgb_->setPixel(pixel_coords_r,clr); }

        // if both camera r and query point are on back of camera m return false
        if (!query_in_front && !cam_r_in_front)
          continue;
        // if query point is in front of camera m whereas camera r is on the back
        else if (query_in_front && !cam_r_in_front){
          // std::cout << "query in front" << std::endl;
          // uv2=query_p_projected_on_cam_m;
          // depth2=query_depth_on_camera_m;
          // Dtam::get1stDepthWithUV(camera_r, camera_m, uv_r, uv1, depth1);
          // depth1=camera_r->lens_;
          // bool flag = camera_m->resizeLine(uv1 , uv2, depth1, depth2);
          // if (!flag)
          //   return false;
          continue;

        }
        // if camera r is in front of camera m whereas query point is on the back
        else if (!query_in_front && cam_r_in_front){
          // TODO
          continue;
        }
        // if both camera r and query point are in front of camera m
        else {
          // std::cout << "both in front" << std::endl;
          uv1=cam_r_projected_on_cam_m;
          uv2=query_p_projected_on_cam_m;
          uv1_fixed=cam_r_projected_on_cam_m;
          uv2_fixed=query_p_projected_on_cam_m;
          depth1_m=cam_r_depth_on_camera_m;
          depth2_m=query_depth_on_camera_m;
          depth1_m_fixed=cam_r_depth_on_camera_m;
          depth2_m_fixed=query_depth_on_camera_m;
          bool flag = camera_m->resizeLine(uv1 , uv2, depth1_m, depth2_m, resized1, resized2);
          if (!flag)
            continue;

          cameraData_vector[camera_iterator-1]->uv1=uv1;
          cameraData_vector[camera_iterator-1]->uv2=uv2;
          cameraData_vector[camera_iterator-1]->uv1_fixed=uv1_fixed;
          cameraData_vector[camera_iterator-1]->uv2_fixed=uv2_fixed;
          cameraData_vector[camera_iterator-1]->depth1_m=depth1_m;
          cameraData_vector[camera_iterator-1]->depth2_m=depth2_m;
          cameraData_vector[camera_iterator-1]->depth1_m_fixed=depth1_m_fixed;
          cameraData_vector[camera_iterator-1]->depth2_m_fixed=depth2_m_fixed;


        }

      }

      // compute when to to start and finish
      int iterator_beginning=0;
      int iterator_end=num_interpolations;

      int cost_min = 999999;
      // float depth_min = 0;
      int iterator = 0;

      for (int i=iterator_beginning; i<iterator_end; i++){
      // for (int i=0; i<5; i++){

        float ratio_depth_r = (float)i/((float)num_interpolations-1);
        float depth_r = depth1_r+ratio_depth_r*(depth2_r-depth1_r);

        int cost_i = -1;

        for (int camera_iterator=1; camera_iterator<camera_vector.size(); camera_iterator++){
          Camera* camera_m = camera_vector[camera_iterator];

          auto uv1=cameraData_vector[camera_iterator-1]->uv1;
          auto uv2=cameraData_vector[camera_iterator-1]->uv2;
          auto uv1_fixed=cameraData_vector[camera_iterator-1]->uv1_fixed;
          auto uv2_fixed=cameraData_vector[camera_iterator-1]->uv2_fixed;
          auto depth1_m=cameraData_vector[camera_iterator-1]->depth1_m;
          auto depth2_m=cameraData_vector[camera_iterator-1]->depth2_m;
          auto depth1_m_fixed=cameraData_vector[camera_iterator-1]->depth1_m_fixed;
          auto depth2_m_fixed=cameraData_vector[camera_iterator-1]->depth2_m_fixed;
          auto r=cameraData_vector[camera_iterator-1]->r;
          auto t=cameraData_vector[camera_iterator-1]->t;


          Eigen::Vector2f uv_current;

          float depth_m = depth_r*r(2,2)-t(2)-((depth_r*r(2,0)*(2*uv_r.x()-w))/(2*f))-((depth_r*r(2,1)*(-2*uv_r.y()+h))/(2*f));

          if (depth_m< depth1_m)
            continue;
          if (depth_m> depth2_m)
            break;

          float invdepth_m = 1.0/depth_m;
          float invdepth_delta = (1.0/depth2_m_fixed)-(1.0/depth1_m_fixed);
          float ratio_invdepth_m = (invdepth_m-(1.0/depth1_m_fixed))/invdepth_delta;

          uv_current.x()=uv1_fixed.x()+ratio_invdepth_m*(uv2_fixed.x()-uv1_fixed.x()) ;
          uv_current.y()=uv1_fixed.y()+ratio_invdepth_m*(uv2_fixed.y()-uv1_fixed.y()) ;

          Eigen::Vector2i pixel_current;
          camera_m->uv2pixelCoords( uv_current, pixel_current);
          cv::Vec3b clr_current;
          bool flag = camera_m->image_rgb_->evalPixel(pixel_current,clr_current);

          int cost_current = mseBetween2Colors(clr_r, clr_current);

          if (cost_i<0)
            cost_i=0;

          cost_i+=cost_current;

          if (check){
            // std::cout << uv_current.x() << " " << uv_current.y() << std::endl;
            cv::Vec3b clr = cv::Vec3b(255,0,255);
            bool fl = camera_m->image_rgb_->setPixel(pixel_current,clr);
            // std::cout << << std::endl;
            // if (fl)
            // std::cout << ratio_invdepth_m << std::endl;
          }

        }

        if (cost_i>0 && cost_i<cost_min){
          cost_min=cost_i;
          iterator=i;
        }
      }

      float depth_min = (depth1_r+(float)iterator/((float)num_interpolations-1)*(depth2_r-depth1_r));

      float depth_value = depth_min/camera_vector[0]->max_depth_;
      if (depth_min==0)
        camera_vector[0]->depth_map_->setPixel(pixel_coords_r,1.0);
      else
        camera_vector[0]->depth_map_->setPixel(pixel_coords_r,depth_value);


      // for (int i=0; i< num_interpolations; i++){
      //   if (costs[i]>=0){
      //     if (costs[i]<cost_min){
      //       cost_min=costs[i];
      //       depth_min = (depth1_r+(float)i/((float)num_interpolations-1)*(depth2_r-depth1_r));
      //     }
      //   }
      // }
      // float depth_value = depth_min/camera_vector[0]->max_depth_;
      // if (depth_min==0)
      //   camera_vector[0]->depth_map_->setPixel(pixel_coords_r,1.0);
      // else
      //   camera_vector[0]->depth_map_->setPixel(pixel_coords_r,depth_value);


      if (check)  {break;}
    }
    if (check)  {break;}
  }

}

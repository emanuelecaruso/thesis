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
  return true;
}

struct cameraData{
  Eigen::Vector2f uv1, uv2, uv1_fixed, uv2_fixed;
  float depth1_m, depth2_m;
  float depth_r_1, depth_r_2;
  Eigen::Matrix3f r_m2r;
  Eigen::Vector3f t_m2r;
  Eigen::Matrix3f r_r2m;
  Eigen::Vector3f t_r2m;
};

void Dtam::getDepthMap(CameraVector& camera_vector, float interpolation_ratio, bool check){
  int cols = camera_vector[0]->depth_map_->image_.cols;
  int rows = camera_vector[0]->depth_map_->image_.rows;

  std::vector<cameraData*> cameraData_vector;
  cameraData_vector.reserve(camera_vector.size());

  float f = camera_vector[0]->lens_;
  float w=camera_vector[0]->width_;
  float h=camera_vector[0]->width_/camera_vector[0]->aspect_;

  for (int camera_iterator=0; camera_iterator<camera_vector.size()-1; camera_iterator++){
    cameraData* camera_data = new cameraData;

    Eigen::Isometry3f T_m2r = camera_vector[0]->frame_world_wrt_camera_*camera_vector[camera_iterator+1]->frame_camera_wrt_world_;
    Eigen::Matrix3f r_m2r=T_m2r.linear();
    Eigen::Vector3f t_m2r=T_m2r.translation();
    camera_data->r_m2r=r_m2r;
    camera_data->t_m2r=t_m2r;

    Eigen::Isometry3f T_r2m = camera_vector[camera_iterator+1]->frame_world_wrt_camera_*camera_vector[0]->frame_camera_wrt_world_;
    Eigen::Matrix3f r_r2m=T_r2m.linear();
    Eigen::Vector3f t_r2m=T_r2m.translation();
    camera_data->r_r2m=r_r2m;
    camera_data->t_r2m=t_r2m;

    cameraData_vector[camera_iterator]=camera_data;
  }




  for (int row = 0; row<rows; row++){
    for (int col = 0; col<cols; col++){

      if (check){row= rows*0.36; col=cols*0.45;}

      Eigen::Vector2i pixel_coords_r(col,row);



      //initialize costs
      // std::vector<int> costs(num_interpolations, -1 );
      int cost_min = 999999;
      float depth_min = 0;

      std::vector<float> depths_vec;
      std::vector<float> depths_vec_limits;
      std::vector<int> num_interpolations_vec;

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
        float depth1_m, depth2_m;
        bool resized1, resized2;

        if (check){ cv::Vec3b clr = cv::Vec3b(0,0,255);
          camera_r->image_rgb_->setPixel(pixel_coords_r,clr); }

        // if both camera r and query point are on back of camera m return false
        if (!query_in_front && !cam_r_in_front){
          // std::cout << "both on back" << std::endl;
          continue;}
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
          // std::cout << "cam in front" << std::endl;
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
          bool flag = camera_m->resizeLine(uv1 , uv2, depth1_m, depth2_m, resized1, resized2);
          if (!flag)
            continue;

          cameraData_vector[camera_iterator-1]->uv1=uv1;
          cameraData_vector[camera_iterator-1]->uv2=uv2;
          cameraData_vector[camera_iterator-1]->uv1_fixed=uv1_fixed;
          cameraData_vector[camera_iterator-1]->uv2_fixed=uv2_fixed;
          cameraData_vector[camera_iterator-1]->depth1_m=depth1_m;
          cameraData_vector[camera_iterator-1]->depth2_m=depth2_m;

          auto r=cameraData_vector[camera_iterator-1]->r_m2r;
          auto t=cameraData_vector[camera_iterator-1]->t_m2r;
          float depth_r_1 = depth1_m*r(2,2)-t(2)-((depth1_m*r(2,0)*(2*uv1.x()-w))/(2*f))-((depth1_m*r(2,1)*(-2*uv1.y()+h))/(2*f));
          float depth_r_2 = depth2_m*r(2,2)-t(2)-((depth2_m*r(2,0)*(2*uv2.x()-w))/(2*f))-((depth2_m*r(2,1)*(-2*uv2.y()+h))/(2*f));

          cameraData_vector[camera_iterator-1]->depth_r_1=depth_r_1;
          cameraData_vector[camera_iterator-1]->depth_r_2=depth_r_2;

          depths_vec_limits.push_back(depth_r_1);
          depths_vec_limits.push_back(depth_r_2);


        }
        // std::cout << depths_vec.size() << std::endl;
        // std::cout << depths_vec[0] << std::endl;

      }

      // std::cout << depths_vec.size() << std::endl;
      // std::cout << depths_vec[0] << std::endl;

      std::sort(depths_vec_limits.begin(), depths_vec_limits.end());
      float pixel_width= camera_vector[0]->width_/camera_vector[0]->resolution_;
      if (depths_vec_limits.size()>0){
        for (int i=0; i<depths_vec_limits.size()-1; i++){
          // std::cout << depths_vec[i] << std::endl;
          int interpolations=0;
          for (int camera_iterator=1; camera_iterator<camera_vector.size(); camera_iterator++){
            // num_interpolations_vec
            float depth_r_1 = cameraData_vector[camera_iterator-1]->depth_r_1;
            float depth_r_2 = cameraData_vector[camera_iterator-1]->depth_r_2;
            if(depth_r_1<=depths_vec_limits[i] && depth_r_2>=depths_vec_limits[i+1]){
              auto r=cameraData_vector[camera_iterator-1]->r_r2m;
              auto t=cameraData_vector[camera_iterator-1]->t_r2m;
              float depth_m_low = depths_vec_limits[i]*r(2,2)-t(2)-((depths_vec_limits[i]*r(2,0)*(2*uv_r.x()-w))/(2*f))-((depths_vec_limits[i]*r(2,1)*(-2*uv_r.y()+h))/(2*f));
              float depth_m_up = depths_vec_limits[i+1]*r(2,2)-t(2)-((depths_vec_limits[i+1]*r(2,0)*(2*uv_r.x()-w))/(2*f))-((depths_vec_limits[i+1]*r(2,1)*(-2*uv_r.y()+h))/(2*f));
              float depth1_m = cameraData_vector[camera_iterator-1]->depth1_m;
              float depth2_m = cameraData_vector[camera_iterator-1]->depth2_m;
              // std::cout << depth_m_low << " " << depth1_m << " " << depth_m_up << " " << depth2_m << std::endl;

              float invdepth_delta = (1.0/depth_m_up)-(1.0/depth_m_low);
              float invdepth_delta_ = (1.0/depth2_m)-(1.0/depth1_m);
              float ratio = invdepth_delta/invdepth_delta_;

              Eigen::Vector2f uv1 = cameraData_vector[camera_iterator-1]->uv1;
              Eigen::Vector2f uv2 = cameraData_vector[camera_iterator-1]->uv2;
              Eigen::Vector2f uv_delta = uv2-uv1;
              Eigen::Vector2f uv_delta_ = ratio*uv_delta;
              int interpolations_current = (abs(uv_delta_.x())+abs(uv_delta_.y()))/(pixel_width);
              if (interpolations_current>interpolations){
                interpolations=interpolations_current;
              }

            }
          }

          num_interpolations_vec.push_back(interpolations);

        }
      }

      if (num_interpolations_vec.size()>0){
        for (int j=0; j<num_interpolations_vec.size(); j++){
          int interpolations = num_interpolations_vec[j]*interpolation_ratio;
          // std::cout << interpolations << std::endl;
          for (int i=0; i<interpolations; i++){

            for (int camera_iterator=1; camera_iterator<camera_vector.size(); camera_iterator++){
              Camera* camera_m = camera_vector[camera_iterator];

              auto uv1=cameraData_vector[camera_iterator-1]->uv1;
              auto uv2=cameraData_vector[camera_iterator-1]->uv2;
              auto depth1_m=cameraData_vector[camera_iterator-1]->depth1_m;
              auto depth2_m=cameraData_vector[camera_iterator-1]->depth2_m;
              auto r_r2m=cameraData_vector[camera_iterator-1]->r_r2m;
              auto t_r2m=cameraData_vector[camera_iterator-1]->t_r2m;
              auto r_m2r=cameraData_vector[camera_iterator-1]->r_m2r;
              auto t_m2r=cameraData_vector[camera_iterator-1]->t_m2r;

              float depth_m_low = depths_vec_limits[j]*r_r2m(2,2)-t_r2m(2)-((depths_vec_limits[j]*r_r2m(2,0)*(2*uv_r.x()-w))/(2*f))-((depths_vec_limits[j]*r_r2m(2,1)*(-2*uv_r.y()+h))/(2*f));
              float depth_m_up = depths_vec_limits[j+1]*r_r2m(2,2)-t_r2m(2)-((depths_vec_limits[j+1]*r_r2m(2,0)*(2*uv_r.x()-w))/(2*f))-((depths_vec_limits[j+1]*r_r2m(2,1)*(-2*uv_r.y()+h))/(2*f));

              float ratio_local = (float)i/((float)interpolations-1);
              float invdepth_m = (1.0/depth_m_low)+ratio_local*((1.0/depth_m_up)-(1.0/depth_m_low));
              float ratio_total = (invdepth_m-(1.0/depth1_m))/((1.0/depth2_m)-(1.0/depth1_m));

              Eigen::Vector2f uv_current;
              uv_current.x()=uv1.x()+ratio_total*(uv2.x()-uv1.x()) ;
              uv_current.y()=uv1.y()+ratio_total*(uv2.y()-uv1.y()) ;

              float depth_m = 1.0/invdepth_m;
              float depth_r =  depth_m*r_m2r(2,2)-t_m2r(2)-((depth_m*r_m2r(2,0)*(2*uv_current.x()-w))/(2*f))-((depth_m*r_m2r(2,1)*(-2*uv_current.y()+h))/(2*f));

              if (depth_m< depth1_m || depth_m> depth2_m)
                continue;


              Eigen::Vector2i pixel_current;
              camera_m->uv2pixelCoords( uv_current, pixel_current);
              cv::Vec3b clr_current;
              bool flag = camera_m->image_rgb_->evalPixel(pixel_current,clr_current);

              int cost_current = mseBetween2Colors(clr_r, clr_current);

              // if (check)
              if (cost_current<cost_min){
                cost_min=cost_current;
                depth_min=depth_r;

              }

              // if (costs[i]<0)
              //   costs[i]=0;
              //
              // costs[i]+=cost_current;
              //
              if (check){
                // std::cout << uv_current.x() << " " << uv_current.y() << std::endl;
                cv::Vec3b magenta = cv::Vec3b(255,0,255);
                bool fl = camera_m->image_rgb_->setPixel(pixel_current,magenta);
                // std::cout << << std::endl;
                // if (!fl)
                // std::cout << "----" << std::endl;
              }


              // if (check){
              //   cv::Vec3b blue = cv::Vec3b(255,0,0);
              //   bool fl = camera_m->image_rgb_->setPixel(pixel_min,blue);
              // }

            }


          }
        }
      }



      // for (int i=0; i< num_interpolations; i++){
      //   if (costs[i]>=0){
      //     if (costs[i]<cost_min){
      //       cost_min=costs[i];
      //       depth_min = (depth1_r+(float)i/((float)num_interpolations-1)*(depth2_r-depth1_r));
      //     }
      //   }
      // }
      
      float depth_value = depth_min/camera_vector[0]->max_depth_;
      if (check)
        depth_value=1.0;
      if (depth_min==0)
        camera_vector[0]->depth_map_->setPixel(pixel_coords_r,1.0);
      else
        camera_vector[0]->depth_map_->setPixel(pixel_coords_r,depth_value);

      if (check)  {break;}
    }
    if (check)  {break;}
  }

}

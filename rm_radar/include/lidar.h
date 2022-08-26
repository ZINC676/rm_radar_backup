//
// Created by jg on 2022/4/2.
//

#ifndef SRC_RM_RADAR_INCLUDE_LIDAR_H_
#define SRC_RM_RADAR_INCLUDE_LIDAR_H_
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <dynamic_reconfigure/server.h>
#include <rm_radar/LidarConfig.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread/shared_mutex.hpp>
#include <thread>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <vector>
#include <array>
//#include <mutex>
//#include <image_geometry/pinhole_camera_model.h>


namespace rm_radar
{
#define PCL_QUEUE_SIZE 100
#define TARGET_NUM 5
#define PI acos(-1.0)
#define SITE_WIDTH 15
#define SITE_HIGH 28

void onMouse(int event, int x, int y, int flags, void *param);

struct PointUV
{
  float u;
  float v;
  float depth;
};
class Lidar : public nodelet::Nodelet
{
friend class MouseCB;

public:
  Lidar();
  ~Lidar() override;
  void onInit() override;

  ros::NodeHandle nh_;
  std::vector<cv::Point3f> target_list_{};



private:
  ros::Subscriber pcl_sub_;
  ros::Subscriber target_sub_1_;
  ros::Subscriber target_sub_2_;
  ros::Subscriber target_sub_3_;
  ros::Subscriber target_sub_4_;
  ros::Subscriber target_sub_5_;

  std::vector<ros::Publisher> target_pub_list;
  ros::Publisher pcl_pub_;
  sensor_msgs::PointCloud2 output_;

  void getParam(std::vector<float> world_to_cam_MCB);
  std::string camera_name_;
  cv::Matx33f intrinsic_;
  cv::Matx34f extrinsic_;
  cv::Matx34f in_dot_ex_;
  cv::Matx34f projection_;
  cv::Matx<float, 1, 5> distortion_;
  cv::Matx44f cam_to_world_;

  cv::Matx44f rot_x_;
  cv::Matx44f rot_y_;
  cv::Matx44f rot_z_;
  cv::Matx44f trans_;
  cv::Matx44f extrinsic_1_;

  bool is_extrinsic_;

  std::vector<boost::shared_ptr<std::vector<PointUV>>> uv_queue_{};
  boost::shared_mutex pcl_queue_lock_;

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter_;
  double radius_search_{};
  int min_neighbors_in_radius_{};

  dynamic_reconfigure::Server<LidarConfig>* srv_{};
  void reconfigCB(LidarConfig& config, uint32_t level);

  template <const int ROBOT_ID>
  void targetCB(const std_msgs::Float32MultiArrayConstPtr& roi_msg);

  bool getTarget(cv::Matx22f* roi_points, cv::Point3f& target);
  cv::Point3f coordinateUVToCam(const PointUV& uv_depth);
  cv::Point3f coordinateCamToWorld(const cv::Point3f& target_in_cam);
  void getUVDepth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                  boost::shared_ptr<std::vector<PointUV>>& uv_with_depth);

  boost::mutex target_lock_;
  tf::TransformBroadcaster target_broadcaster_;

  std::thread UI_thread;

};



class MouseCB : Lidar
{
public:
    cv::Mat img_;
    ros::NodeHandle parent_nh_;
    std::vector<cv::Point2d> points_2d_;
    std::vector<cv::Point3d> points_3d_;
    sensor_msgs::CameraInfo camera_info_;
    cv::Mat_<double> cam_intrinsic_mat_k_;
    std::vector<double> dist_coefficients_;
    std::vector<float> mouseCB_mat_;

public:
    MouseCB(ros::NodeHandle& nh);

    ~MouseCB() = default;

    void initialize(ros::NodeHandle& nh);

    void execute();

    template<typename T>
    inline T getParam(const ros::NodeHandle &nh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        nh.param<T>(param_name, param_val, default_val);
        return param_val;
    }
};


}  // namespace rm_radar
#endif  // SRC_RM_RADAR_INCLUDE_LIDAR_H_

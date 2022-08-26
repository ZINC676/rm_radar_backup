//
// Created by jg on 2022/4/2.
//

#ifndef SRC_RM_RADAR_INCLUDE_PREALARM_H_
#define SRC_RM_RADAR_INCLUDE_PREALARM_H_
#include "lidar.h"
#include <image_transport/image_transport.h>
#include <std_msgs/Int8MultiArray.h>
#include <ros/time.h>

namespace rm_radar
{
#define DEPTH_OMISSION 5
#define AREA_NUM 5

class UI : public nodelet::Nodelet
{
public:
  UI();
  ~UI() override;

  void onInit() override;

private:
  ros::NodeHandle nh_;
  cv::Mat map_;
  image_transport::Publisher img_pub_;
  ros::Publisher warning_pub_;

  ros::Subscriber target_sub_left_1_;
  ros::Subscriber target_sub_left_2_;
  ros::Subscriber target_sub_left_3_;
  ros::Subscriber target_sub_left_4_;
  ros::Subscriber target_sub_left_5_;
  ros::Subscriber target_sub_right_1_;
  ros::Subscriber target_sub_right_2_;
  ros::Subscriber target_sub_right_3_;
  ros::Subscriber target_sub_right_4_;
  ros::Subscriber target_sub_right_5_;

  template <const int ID>
  void targetCB(const std_msgs::Float32MultiArrayConstPtr& target_msg);

  float fx_{}, fy_{};

  std::vector<boost::shared_ptr<std::vector<cv::Point>>> prealarm_areas_{};

  void getParm();

  bool is_red_{}, is_polys_{};

  int check(const cv::Point& target, const int& robot_id,
            const std::vector<boost::shared_ptr<std::vector<cv::Point>>>& prealarm_areas);
};
}  // namespace rm_radar
#endif  // SRC_RM_RADAR_INCLUDE_PREALARM_H_

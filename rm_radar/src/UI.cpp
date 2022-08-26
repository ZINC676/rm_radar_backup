//
// Created by jg on 2022/4/2.
//

#include "UI.h"

namespace rm_radar
{

PLUGINLIB_EXPORT_CLASS(rm_radar::UI, nodelet::Nodelet)

UI::UI()
{
  this->map_ = cv::imread(ros::package::getPath("rm_radar") + "/config/map.jpg");  // read in RM map image

  prealarm_areas_.reserve(AREA_NUM);
}

void UI::onInit()
{
  nh_ = this->getMTPrivateNodeHandle();
  image_transport::ImageTransport it(nh_);
  this->img_pub_ = it.advertise("map", 10);
  getParm();

  if (is_red_)  //  default is false means the enemy is not red, you can change it in radar_double_camera.launch
    cv::rotate(map_, map_, cv::ROTATE_180);

  fx_ = map_.cols / SITE_WIDTH; // width: 15
  fy_ = map_.rows / SITE_HIGH; // height: 28
  std::cout << map_.cols << "    " << map_.rows << std::endl; // col" 778 || row: 1437
  std::cout << "fx_ is: " << fx_ << std::endl; // 51
  std::cout << "fy_ is：" << fy_ << std::endl; // 51

  if (is_polys_)
  {
    for (auto& i : prealarm_areas_)
    {
      cv::polylines(map_, *i, 1, cv::Scalar(255, 255, 0), 10);
    }
  }

  warning_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("warning", 5);

  //  subscribe topic which advertised from lidar.cpp,
  target_sub_left_1_ =
      nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_left/left_camera_proc_target_1",
                                                 5, &UI::targetCB<1>, this);
  target_sub_left_2_ =
      nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_left/left_camera_proc_target_2", 5, &UI::targetCB<2>, this);
  target_sub_left_3_ =
      nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_left/left_camera_proc_target_3", 5, &UI::targetCB<3>, this);
  target_sub_left_4_ =
      nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_left/left_camera_proc_target_4", 5, &UI::targetCB<4>, this);
  target_sub_left_5_ =
      nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_left/left_camera_proc_target_5", 5, &UI::targetCB<5>, this);
  target_sub_right_1_ = nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_right/right_camera_proc_target_1", 5,
                                                                   &UI::targetCB<1>, this);
  target_sub_right_2_ = nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_right/right_camera_proc_target_2", 5,
                                                                   &UI::targetCB<2>, this);
  target_sub_right_3_ = nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_right/right_camera_proc_target_3", 5,
                                                                   &UI::targetCB<3>, this);
  target_sub_right_4_ = nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_right/right_camera_proc_target_4", 5,
                                                                   &UI::targetCB<4>, this);
  target_sub_right_5_ = nh_.subscribe<std_msgs::Float32MultiArray>("/rm_radar_right/right_camera_proc_target_5", 5,
                                                                   &UI::targetCB<5>, this);

  ros::Rate rate(0.5);
  while (ros::ok)
  {
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map_).toImageMsg();
    img_pub_.publish(img_msg);
    ROS_INFO("UI_node_1");
    assert(ros::ok());
    ros::spinOnce();
    assert(ros::ok());
    ROS_INFO("UI_node_2");
    rate.sleep();
    ROS_INFO("UI_node_3");
    assert(ros::ok());
  }
}

UI::~UI()
{
  prealarm_areas_.clear();
}

void UI::getParm()
{
  nh_.getParam("is_red", is_red_);
  nh_.getParam("map_config/is_polys", is_polys_);

  std::vector<float> area_flying_slope;
  nh_.getParam("map_config/area_flying_slope", area_flying_slope);
  boost::shared_ptr<std::vector<cv::Point>> area_points_0(new std::vector<cv::Point>);
  for (int i = 0; i < area_flying_slope.size(); i = i + 2)
  {
    area_points_0->emplace_back(cv::Point(area_flying_slope[i], area_flying_slope[i + 1]));
  }
  prealarm_areas_.emplace_back(area_points_0);

  std::vector<float> area_shooting_windmill;
  nh_.getParam("map_config/area_shooting_windmill", area_shooting_windmill);
  boost::shared_ptr<std::vector<cv::Point>> area_points_1(new std::vector<cv::Point>);
  for (int i = 0; i < area_shooting_windmill.size(); i = i + 2)
  {
    area_points_1->emplace_back(cv::Point(area_shooting_windmill[i], area_shooting_windmill[i + 1]));
  }
  prealarm_areas_.emplace_back(area_points_1);

  std::vector<float> area_lobbing_1;
  nh_.getParam("map_config/area_lobbing_1", area_lobbing_1);
  boost::shared_ptr<std::vector<cv::Point>> area_points_2(new std::vector<cv::Point>);
  for (int i = 0; i < area_lobbing_1.size(); i = i + 2)
  {
    area_points_2->emplace_back(cv::Point(area_lobbing_1[i], area_lobbing_1[i + 1]));
  }
  prealarm_areas_.emplace_back(area_points_2);

  std::vector<float> area_lobbing_2;
  nh_.getParam("map_config/area_lobbing_2", area_lobbing_2);
  boost::shared_ptr<std::vector<cv::Point>> area_points_3(new std::vector<cv::Point>);
  for (int i = 0; i < area_lobbing_1.size(); i = i + 2)
  {
    area_points_3->emplace_back(cv::Point(area_lobbing_2[i], area_lobbing_2[i + 1]));
  }
  prealarm_areas_.emplace_back(area_points_3);

  std::vector<float> area_refilling;
  nh_.getParam("map_config/area_refilling", area_refilling);
  boost::shared_ptr<std::vector<cv::Point>> area_points_4(new std::vector<cv::Point>);
  for (int i = 0; i < area_refilling.size(); i = i + 2)
  {
    area_points_4->emplace_back(cv::Point(area_refilling[i], area_refilling[i + 1]));
  }
  prealarm_areas_.emplace_back(area_points_4);

}

int UI::check(const cv::Point& target, const int& robot_id,
              const std::vector<boost::shared_ptr<std::vector<cv::Point>>>& prealarm_areas)
{
  switch (robot_id)
  {
    case 1:
      if (cv::pointPolygonTest(*prealarm_areas[0], target, false)!=-1)
      {
          // cv::pointPolygonTest: check the point is in those pre-alarm areas or not
        std_msgs::Int8MultiArray msg;
        msg.data = { 1, 0 };
        warning_pub_.publish(msg);
        return 0;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[2], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 1, 2 };
        warning_pub_.publish(msg);
        return 2;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[3], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 1, 3 };
        warning_pub_.publish(msg);
        return 3;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[4], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = std::vector<int8_t>{ 1, 4 };
        warning_pub_.publish(msg);
        return 4;
      }
      else
        return -1;
    case 2:
      return -1;
    case 3:
      if (cv::pointPolygonTest(*prealarm_areas[0], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 3, 0 };
        warning_pub_.publish(msg);
        return 0;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[1], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 3, 1 };
        warning_pub_.publish(msg);
        return 1;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[4], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 3, 4 };
        warning_pub_.publish(msg);
        return 4;
      }
      return -1;
    case 4:
      if (cv::pointPolygonTest(*prealarm_areas[0], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 4, 0 };
        warning_pub_.publish(msg);
        return 0;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[1], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 4, 1 };
        warning_pub_.publish(msg);
        return 1;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[4], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 4, 4 };
        warning_pub_.publish(msg);
        return 4;
      }
      return -1;
    case 5:
      if (cv::pointPolygonTest(*prealarm_areas[0], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 5, 0 };
        warning_pub_.publish(msg);
        return 0;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[1], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 5, 1 };
        warning_pub_.publish(msg);
        return 1;
      }
      else if (cv::pointPolygonTest(*prealarm_areas[4], target, false)!=-1)
      {
        std_msgs::Int8MultiArray msg;
        msg.data = { 5, 4 };
        warning_pub_.publish(msg);
        return 4;
      }
    default:
      return -1;
  }
  return -1;
}

template <const int ROBOT_ID>
//void UI::targetCB(const std_msgs::Float32MultiArrayConstPtr& target_msg)
void UI::targetCB(const std_msgs::Float32MultiArrayConstPtr& target_msg)
{
  cv::Point3f target(target_msg->data[0], target_msg->data[1], target_msg->data[2]);

  cv::Mat map = this->map_.clone();

  cv::Point point_xy(target.x * fx_, target.y * fy_); // data is sth * 51, transfer xy point into map

  std::cout << "point_xy: "  << point_xy << std::endl;
  int area_id = check(point_xy, ROBOT_ID, prealarm_areas_);
  //  check if hero or footman show up in some special areas, if they do, return the rebot ID

  // if some aggressive units show up in some special areas, ring the alarm!!
  if (area_id >= 0)
    cv::fillPoly(map, std::vector<std::vector<cv::Point>>{*prealarm_areas_[area_id]}, cv::Scalar(0, 255, 255));
    //  fill those pre_alarm area with yellow, to warn our team(填充颜色)

 if (is_red_){
      cv::circle(map, point_xy, 20, cv::Scalar(255, 0, 0), -1);
      int target_id = 0;
      target_id = ROBOT_ID;
      cv::putText(map, std::to_string(target_id), point_xy, cv::FONT_HERSHEY_SIMPLEX,
                  2, cv::Scalar(255,51,204), 4, 8);
      //  show detected enemy's number in UI
  }
  else{
      cv::circle(map, point_xy, 20, cv::Scalar(0, 0, 255), -1);
      int target_id = 0;
      target_id = ROBOT_ID;
      cv::putText(map, std::to_string(target_id), point_xy, cv::FONT_HERSHEY_SIMPLEX,
                  2, cv::Scalar(255,51,204), 4, 8);
      //  show detected enemy's number in UI
  }

  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map).toImageMsg();
  img_pub_.publish(img_msg);
  // publish processed UI map, we can see the map through rviz


}

}  // namespace rm_radar
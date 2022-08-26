//
// Created by jg on 22-4-22.
//
#include "lidar.h"
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_node");

  rm_radar::Lidar lidar;
  lidar.nh_ = ros::NodeHandle("~");
  lidar.onInit();

  ros::spin();
  return 0;
}


// Code down here is use for testing if receive target from camera, then publish the target point_xy
//#include <typeinfo>
//#include <ros/ros.h>

//int main(int argc, char **argv)
//{
//    // ROS节点初始化
//    ros::init(argc, argv, "test_ui_publisher");
//
//    // 创建节点句柄
//    ros::NodeHandle n;
//
//    std_msgs::Float32MultiArray target_msg_hero;
//    target_msg_hero.data = { 1.0, 10.0, 0 };
//
//    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
//    ros::Publisher test_ui_publisher_hero = n.advertise<std_msgs::Float32MultiArray>("/rm_radar_left/left_camera_proc_target_test1", 10);
//
//    // 设置循环的频率
//    ros::Rate loop_rate(1);
//
//    int count = 0;
//    while (ros::ok())
//    {
//        test_ui_publisher_hero.publish(target_msg_hero);
//    }
//
//    return 0;
//}

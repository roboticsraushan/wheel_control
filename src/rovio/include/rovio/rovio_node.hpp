#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include <mutex>

class ROVIONode : public rclcpp::Node
{
public:
  explicit ROVIONode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  std::string camera_image_topic_;
  std::string camera_info_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  bool publish_tf_;

  // Camera intrinsics
  double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
  bool have_camera_info_ = false;

  // VO state
  std::mutex vo_mutex_;
  cv::Mat prev_gray_;
  std::vector<cv::Point2f> prev_pts_;
  Eigen::Matrix4d global_pose_ = Eigen::Matrix4d::Identity();
  bool have_prev_ = false;
  int min_features_ = 100;
  // smoothing / quality
  int min_inliers_ = 30;
  double smoothing_alpha_ = 0.6; // 0..1, higher = less smoothing
  rclcpp::Time last_image_stamp_;
  Eigen::Matrix4d last_published_pose_ = Eigen::Matrix4d::Identity();

  // Last received sensor messages (lightweight)
  sensor_msgs::msg::Image::SharedPtr last_image_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
};

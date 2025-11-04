// Lightweight visual odometry implementation using OpenCV optical flow.
#include "rovio/rovio_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Dense>

#include "rclcpp_components/register_node_macro.hpp"

ROVIONode::ROVIONode(const rclcpp::NodeOptions & options)
: Node("rovio_node", options)
{
	// Declare parameters with defaults (updated to workspace camera topics)
	this->declare_parameter<std::string>("camera_image_topic", "/camera/camera/color/image_raw");
	this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
	this->declare_parameter<std::string>("imu_topic", "/camera/imu");
	this->declare_parameter<std::string>("odom_topic", "/odom/rovio");
	this->declare_parameter<std::string>("base_frame", "base_link");
	this->declare_parameter<std::string>("odom_frame", "odom");
	this->declare_parameter<bool>("publish_tf", true);
		this->declare_parameter<int>("min_features", 100);
		this->declare_parameter<int>("min_inliers", 30);
		this->declare_parameter<double>("smoothing_alpha", 0.6);

	camera_image_topic_ = this->get_parameter("camera_image_topic").as_string();
	camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
	imu_topic_ = this->get_parameter("imu_topic").as_string();
	odom_topic_ = this->get_parameter("odom_topic").as_string();
	base_frame_ = this->get_parameter("base_frame").as_string();
	odom_frame_ = this->get_parameter("odom_frame").as_string();
	publish_tf_ = this->get_parameter("publish_tf").as_bool();
		min_features_ = this->get_parameter("min_features").as_int();
		min_inliers_ = this->get_parameter("min_inliers").as_int();
		smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();

	RCLCPP_INFO(this->get_logger(), "ROVIO-VO node starting. Subscribing to '%s' and '%s'", camera_image_topic_.c_str(), camera_info_topic_.c_str());

	image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		camera_image_topic_, rclcpp::SensorDataQoS(),
		std::bind(&ROVIONode::image_callback, this, std::placeholders::_1));

	imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
		imu_topic_, rclcpp::SensorDataQoS(),
		std::bind(&ROVIONode::imu_callback, this, std::placeholders::_1));

	camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		camera_info_topic_, rclcpp::SensorDataQoS(),
		std::bind(&ROVIONode::camera_info_callback, this, std::placeholders::_1));

	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

		// Publish timer kept as fallback, but main publishing happens after image processing.
		publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
			std::lock_guard<std::mutex> lk(vo_mutex_);
			if (!have_camera_info_) return;
			// publish last_published_pose_ with its original timestamp if available
			if (last_image_stamp_.nanoseconds() == 0) return;

			nav_msgs::msg::Odometry odom;
			odom.header.stamp = last_image_stamp_;
			odom.header.frame_id = odom_frame_;
			odom.child_frame_id = base_frame_;
			Eigen::Matrix3d R = last_published_pose_.block<3,3>(0,0);
			Eigen::Vector3d t = last_published_pose_.block<3,1>(0,3);
			Eigen::Quaterniond eq(R);
			odom.pose.pose.position.x = t.x();
			odom.pose.pose.position.y = t.y();
			odom.pose.pose.position.z = t.z();
			odom.pose.pose.orientation.x = eq.x();
			odom.pose.pose.orientation.y = eq.y();
			odom.pose.pose.orientation.z = eq.z();
			odom.pose.pose.orientation.w = eq.w();
			odom_pub_->publish(odom);
			if (publish_tf_) {
				static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bcast = std::make_shared<tf2_ros::TransformBroadcaster>(this);
				geometry_msgs::msg::TransformStamped tmsg;
				tmsg.header.stamp = last_image_stamp_;
				tmsg.header.frame_id = odom_frame_;
				tmsg.child_frame_id = base_frame_;
				tmsg.transform.translation.x = t.x();
				tmsg.transform.translation.y = t.y();
				tmsg.transform.translation.z = t.z();
				tmsg.transform.rotation.x = eq.x();
				tmsg.transform.rotation.y = eq.y();
				tmsg.transform.rotation.z = eq.z();
				tmsg.transform.rotation.w = eq.w();
				tf_bcast->sendTransform(tmsg);
			}
		});
}

void ROVIONode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	std::lock_guard<std::mutex> lk(vo_mutex_);
	if (have_camera_info_) return; // only read once
	if (msg->k.size() >= 9) {
		fx_ = msg->k[0];
		fy_ = msg->k[4];
		cx_ = msg->k[2];
		cy_ = msg->k[5];
		have_camera_info_ = true;
		RCLCPP_INFO(this->get_logger(), "Camera intrinsics received: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx_, fy_, cx_, cy_);
	}
}

void ROVIONode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	if (!have_camera_info_) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for camera_info on '%s'", camera_info_topic_.c_str());
		return;
	}

		cv_bridge::CvImageConstPtr cv_ptr;
		try {
			// preserve original encoding and convert to mono if needed
			cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
		} catch (cv_bridge::Exception & e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat gray;
		if (msg->encoding == "mono8") {
			gray = cv_ptr->image;
		} else if (msg->encoding == "rgb8" || msg->encoding == "bgr8") {
			cv::Mat color = cv_ptr->image;
			cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
		} else {
			// try conversion via cv_bridge
			try {
				cv::Mat tmp = cv_bridge::toCvCopy(msg, "mono8")->image;
				gray = tmp;
			} catch (...) {
				RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
				return;
			}
		}
	if (gray.empty()) return;

	std::lock_guard<std::mutex> lk(vo_mutex_);

	if (!have_prev_) {
		// First frame: detect features
		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(gray, corners, 500, 0.01, 8);
		if (corners.size() < 20) {
			RCLCPP_WARN(this->get_logger(), "Too few features detected: %zu", corners.size());
			return;
		}
		prev_gray_ = gray.clone();
		prev_pts_ = corners;
		have_prev_ = true;
		return;
	}

	// Track features with LK
	std::vector<cv::Point2f> next_pts;
	std::vector<unsigned char> status;
	std::vector<float> err;
	cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_pts_, next_pts, status, err);

	std::vector<cv::Point2f> p0, p1;
	for (size_t i = 0; i < status.size(); ++i) {
		if (status[i]) {
			p0.push_back(prev_pts_[i]);
			p1.push_back(next_pts[i]);
		}
	}

	if (p0.size() < static_cast<size_t>(std::max(20, min_features_))) {
		// Not enough tracked points — re-detect
		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(gray, corners, 500, 0.01, 8);
		prev_gray_ = gray.clone();
		prev_pts_ = corners;
		RCLCPP_DEBUG(this->get_logger(), "Re-detected %zu features", corners.size());
		return;
	}

	// Normalize points to camera coordinates for essential matrix
	double focal = (fx_ + fy_) * 0.5;
	cv::Point2d pp(cx_, cy_);
	cv::Mat mask;
	cv::Mat E = cv::findEssentialMat(p0, p1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
	if (E.empty()) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Essential matrix estimation failed");
		prev_gray_ = gray.clone();
		prev_pts_ = p1; // try to continue
		return;
	}

		cv::Mat R_cv, t_cv;
		int inliers = cv::recoverPose(E, p0, p1, R_cv, t_cv, focal, pp, mask);
		RCLCPP_DEBUG(this->get_logger(), "recoverPose inliers=%d", inliers);
		if (inliers < min_inliers_) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "recoverPose inliers (%d) below threshold (%d) — skipping update", inliers, min_inliers_);
			// update prev state for next attempt
			prev_gray_ = gray.clone();
			prev_pts_ = p1;
			return;
		}
	if (inliers < 10) {
		RCLCPP_DEBUG(this->get_logger(), "recoverPose inliers %d too low", inliers);
	}

	// Convert to Eigen
	Eigen::Matrix3d R_e;
	for (int r = 0; r < 3; ++r)
		for (int c = 0; c < 3; ++c)
			R_e(r,c) = R_cv.at<double>(r,c);

	Eigen::Vector3d t_e;
	t_e.x() = t_cv.at<double>(0);
	t_e.y() = t_cv.at<double>(1);
	t_e.z() = t_cv.at<double>(2);

	// Build relative transform (prev -> curr)
	Eigen::Matrix4d T_rel = Eigen::Matrix4d::Identity();
	T_rel.block<3,3>(0,0) = R_e;
	// Scale is unknown. We use unit scale (1.0) — odom will be up-to-scale.
	double scale = 1.0;
	T_rel.block<3,1>(0,3) = t_e * scale;

		// Update global pose: global = global * T_rel
		Eigen::Matrix4d new_global = global_pose_ * T_rel;

		// Simple exponential smoothing between last published pose and new_global to reduce jitter
		Eigen::Matrix4d smoothed = new_global;
		if (last_published_pose_.determinant() != 0) {
			// interpolate translation
			Eigen::Vector3d t_new = new_global.block<3,1>(0,3);
			Eigen::Vector3d t_last = last_published_pose_.block<3,1>(0,3);
			Eigen::Vector3d t_sm = smoothing_alpha_ * t_last + (1.0 - smoothing_alpha_) * t_new;
			// interpolate rotation via quaternion slerp
			Eigen::Quaterniond q_new(new_global.block<3,3>(0,0));
			Eigen::Quaterniond q_last(last_published_pose_.block<3,3>(0,0));
			q_last.normalize(); q_new.normalize();
			Eigen::Quaterniond q_sm = q_last.slerp(1.0 - smoothing_alpha_, q_new);
			smoothed.setIdentity();
			smoothed.block<3,3>(0,0) = q_sm.toRotationMatrix();
			smoothed.block<3,1>(0,3) = t_sm;
		}

		global_pose_ = new_global;
		last_published_pose_ = smoothed;
		last_image_stamp_ = rclcpp::Time(msg->header.stamp);

		// Publish odom and TF immediately using image timestamp
		nav_msgs::msg::Odometry odom;
		odom.header.stamp = last_image_stamp_;
		odom.header.frame_id = odom_frame_;
		odom.child_frame_id = base_frame_;
		Eigen::Matrix3d R = smoothed.block<3,3>(0,0);
		Eigen::Vector3d t = smoothed.block<3,1>(0,3);
		Eigen::Quaterniond eq(R);
		odom.pose.pose.position.x = t.x();
		odom.pose.pose.position.y = t.y();
		odom.pose.pose.position.z = t.z();
		odom.pose.pose.orientation.x = eq.x();
		odom.pose.pose.orientation.y = eq.y();
		odom.pose.pose.orientation.z = eq.z();
		odom.pose.pose.orientation.w = eq.w();
		odom_pub_->publish(odom);

		if (publish_tf_) {
			static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bcast = std::make_shared<tf2_ros::TransformBroadcaster>(this);
			geometry_msgs::msg::TransformStamped tmsg;
			tmsg.header.stamp = last_image_stamp_;
			tmsg.header.frame_id = odom_frame_;
			tmsg.child_frame_id = base_frame_;
			tmsg.transform.translation.x = t.x();
			tmsg.transform.translation.y = t.y();
			tmsg.transform.translation.z = t.z();
			tmsg.transform.rotation.x = eq.x();
			tmsg.transform.rotation.y = eq.y();
			tmsg.transform.rotation.z = eq.z();
			tmsg.transform.rotation.w = eq.w();
			tf_bcast->sendTransform(tmsg);
		}

	// Prepare for next frame: keep tracked points where mask==1
	std::vector<cv::Point2f> new_prev_pts;
	for (size_t i = 0; i < p1.size(); ++i) {
		if (mask.at<unsigned char>(static_cast<int>(i))) {
			new_prev_pts.push_back(p1[i]);
		}
	}

	// If too few points remain, re-detect from current frame
	if (new_prev_pts.size() < static_cast<size_t>(min_features_)) {
		std::vector<cv::Point2f> corners;
		cv::goodFeaturesToTrack(gray, corners, 500, 0.01, 8);
		prev_pts_ = corners;
	} else {
		prev_pts_ = new_prev_pts;
	}

	prev_gray_ = gray.clone();
}

void ROVIONode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr /*msg*/)
{
	// We currently don't use IMU for this lightweight VO. Keep the callback for future fusion.
}

RCLCPP_COMPONENTS_REGISTER_NODE(ROVIONode)


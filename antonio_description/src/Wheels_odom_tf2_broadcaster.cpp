// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("odom_tf2_frame_publisher")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_Wheels_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Wheels/odom", 10,
      std::bind(&FramePublisher::handle_Wheels_odom, this, std::placeholders::_1));

     sub_Z_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/ccd_Z", 10,
      std::bind(&FramePublisher::handle_Z_pose, this, std::placeholders::_1));

      sub_IMU_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/IMU/odom", 10,
      std::bind(&FramePublisher::handle_IMU, this, std::placeholders::_1));
  }

private:
  geometry_msgs::msg::TransformStamped t;

  double IMU_pitch, IMU_roll, IMU_yaw;

  void handle_IMU(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
  {
    IMU_pitch = (msg->orientation.y) * -1;
    IMU_roll = msg->orientation.x;

  }

  void handle_Z_pose(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
  {
    t.transform.translation.z = (msg->position.z) * 0.01;
  }

  void handle_Wheels_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
  
    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    //t.transform.translation.z = msg->pose.pose.position.z;

    tf2::Quaternion q;
    q.setRPY(IMU_roll, IMU_pitch, msg->pose.pose.orientation.z );
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_Z_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Wheels_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_IMU_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
 // std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}

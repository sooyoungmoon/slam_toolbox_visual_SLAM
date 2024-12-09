#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "eigen3/Eigen/Geometry"

class MyTf2Broadcaster : public rclcpp::Node
{
public:
  MyTf2Broadcaster()
  : Node("my_tf2_broadcaster")
  {

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

   
    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MyTf2Broadcaster::handle_odom, this, std::placeholders::_1));

    //timer_ = this->create_wall_timer(
     // std::chrono::milliseconds(1000), std::bind(&MyTf2Broadcaster::timer_callback, this));
  }

private:
    void handle_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
      // print the position in the odom msg
      //RCLCPP_INFO(this->get_logger(), "position: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      // print the orientation in the odom msg
      //RCLCPP_INFO(this->get_logger(), "orientation: x=%f, y=%f, z=%f, w=%f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      // prepare transform message

      //this->declare_parameter<std::string>("base_frame_id");
      std::string base_frame_id = "camera_link";
      std::string odom_frame_id = "odom";
  

      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = msg->header.stamp;      
      transform_stamped.header.frame_id = odom_frame_id;
      transform_stamped.child_frame_id = base_frame_id;        

      // publish new pose in transform message
      transform_stamped.transform.translation.x = msg->pose.pose.position.x;
      transform_stamped.transform.translation.y = msg->pose.pose.position.y;
      transform_stamped.transform.translation.z = msg->pose.pose.position.z;

      transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
      transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
      transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
      transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;
      
      tf_broadcaster_->sendTransform(transform_stamped);
    }

    
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    geometry_msgs::msg::Pose last_pose;
    sensor_msgs::msg::Imu last_imu;
    builtin_interfaces::msg::Time last_time;
    double velocity[3]; // x, y, z
    double orientation[3]; // roll, pitch, yaw

    double linear_accel_default[3] = {0, 0, 0};
    double angular_velocity_default[3] = {0, 0, 0};
    bool linear_accel_default_set = false;
    bool angular_velocity_default_set = false;
   
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyTf2Broadcaster>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "mpc_rbt_simulator/RobotConfig.hpp"

#include "Localization.hpp" 

LocalizationNode::LocalizationNode() : rclcpp::Node("localization_node"), last_time_(this->get_clock()->now()) {
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";

    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    RCLCPP_INFO(this->get_logger(), "Received JointState message:");
    for (size_t i = 0; i < msg.name.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Joint: %s | Position: %f | Velocity: %f | Effort: %f",
                    msg.name[i].c_str(),
                    msg.position.empty() ? 0.0 : msg.position[i],
                    msg.velocity.empty() ? 0.0 : msg.velocity[i],
                    msg.effort.empty() ? 0.0 : msg.effort[i]);
    }

    if (msg.velocity.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received joint state but missing velocity data.");
        return;
    }

    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // Adjust wheel distance as per your robot specifications
    
    double left_wheel_linear_vel = left_wheel_vel * robot_config::WHEEL_RADIUS;
    double right_wheel_linear_vel = right_wheel_vel * robot_config::WHEEL_RADIUS;

    double linear_velocity = (left_wheel_linear_vel + right_wheel_linear_vel) / 2.0;
    double angular_velocity = -(right_wheel_linear_vel - left_wheel_linear_vel) / (robot_config::HALF_DISTANCE_BETWEEN_WHEELS*2);

    // Extract current orientation (yaw angle)
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    // Update pose
    yaw += angular_velocity * dt;
    odometry_.pose.pose.position.x += linear_velocity * dt * std::cos(yaw);
    odometry_.pose.pose.position.y += linear_velocity * dt * std::sin(yaw);

    // Update quaternion
    tf2::Quaternion new_quat;
    new_quat.setRPY(0, 0, yaw);
    odometry_.pose.pose.orientation = tf2::toMsg(new_quat);
}

void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = odometry_.pose.pose.position.x;
    transform.transform.translation.y = odometry_.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
}
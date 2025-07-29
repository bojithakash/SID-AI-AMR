#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include<rclcpp/rclcpp.hpp>
#include<tf2_ros/static_transform_broadcaster.h>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/buffer.h>
#include<tf2_ros/transform_listener.h>
#include<geometry_msgs/msg/transform_stamped.hpp>
#include<walle_msg/srv/get_transform.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include<memory>

class SimpleTfKinematics : public rclcpp::Node 
{
public:
    SimpleTfKinematics(const std::string &name);

private:

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::Service<walle_msg::srv::GetTransform>::SharedPtr get_transform_srv_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    double x_increment;
    double last_x; 
    int rotation_counter_;

    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;

    
    void timerCallback();

    bool getTransformCallback(const std::shared_ptr<walle_msg::srv::GetTransform::Request> req,
                             const std::shared_ptr<walle_msg::srv::GetTransform::Response> res );


};

#endif
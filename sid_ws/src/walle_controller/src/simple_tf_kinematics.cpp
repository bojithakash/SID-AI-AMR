#include "walle_controller/simple_tf_kinematics.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) :Node(name)
    , x_increment(0.0005)
    , last_x(0.0)
    , rotation_counter_(0)

{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    dynamic_tf_broadcaster_= std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_= std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "walle_base";
    static_transform_stamped_.child_frame_id = "walle_top";
    static_transform_stamped_.transform.translation.x =0.2;
    static_transform_stamped_.transform.translation.y =0.0;
    static_transform_stamped_.transform.translation.z = 0.3;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.3;
    static_transform_stamped_.transform.rotation.z = 0.2;
    static_transform_stamped_.transform.rotation.w = 1.0;


    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(get_logger(), "the transform is between : "<<static_transform_stamped_.header.frame_id<<" and "<<static_transform_stamped_.child_frame_id);

    timer_ = create_wall_timer(0.01s, std::bind(&SimpleTfKinematics::timerCallback, this));

    get_transform_srv_ = create_service<walle_msg::srv::GetTransform>("get_transform", 
                            std::bind(&SimpleTfKinematics::getTransformCallback, this, _1,_2));

    last_orientation_.setRPY(0, 0, 0);
    orientation_increment_.setRPY(0, 0, 0.05);

}


void SimpleTfKinematics::timerCallback()
{
    dynamic_transform_stamped_.header.stamp = get_clock()->now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "walle_base";
    dynamic_transform_stamped_.transform.translation.x = last_x + x_increment;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q = last_orientation_ * orientation_increment_;
    q.normalize();
    dynamic_transform_stamped_.transform.rotation.x = q.x();
    dynamic_transform_stamped_.transform.rotation.y = q.y();
    dynamic_transform_stamped_.transform.rotation.z = q.z();
    dynamic_transform_stamped_.transform.rotation.w = q.w();

    dynamic_tf_broadcaster_ ->sendTransform(dynamic_transform_stamped_);

    last_x = dynamic_transform_stamped_.transform.translation.x;

    rotation_counter_++;
    last_orientation_= q;

    if(rotation_counter_>=50)
    {
        orientation_increment_ = orientation_increment_.inverse();
        rotation_counter_= 0;
    }
}

bool SimpleTfKinematics::getTransformCallback(const std::shared_ptr<walle_msg::srv::GetTransform::Request> req,
                             const std::shared_ptr<walle_msg::srv::GetTransform::Response> res )
{
    RCLCPP_INFO_STREAM(get_logger(), "Requested Transform Between "<<req->frame_id<<" and "<<req->child_frame_id);
    geometry_msgs::msg::TransformStamped requested_transform;

    try
    {
        requested_transform = tf_buffer_->lookupTransform(req->frame_id, req->child_frame_id, tf2::TimePointZero);
    }

    catch(tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "An error occured while transforming from "<<req->frame_id<<" and "<<req->child_frame_id<<" : "<<ex.what());

        res-> success= false;
        return true;
    }
    res->transform = requested_transform;
    res->success = true;
    return true; 
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
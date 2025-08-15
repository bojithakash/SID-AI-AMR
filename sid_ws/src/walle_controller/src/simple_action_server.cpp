#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "walle_msg/action/Fibonacci.hpp"

#include <memory>
#include <thread>


using namespace std::placeholders;

namespace walle_controller
{
class SimpleActionServer : public rclcpp::Node
{
public:
  explicit SimpleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("simple_action_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<walle_msg::action::Fibonacci>(
        this, "fibonacci", std::bind(&SimpleActionServer::goalCallback, this, _1, _2),
        std::bind(&SimpleActionServer::cancelCallback, this, _1),
        std::bind(&SimpleActionServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<walle_msg::action::Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const walle_msgs::action::Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<walle_msg::action::Fibonacci>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<walle_msg::action::Fibonacci>> goal_handle)
  {
    std::thread{ std::bind(&SimpleActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<walle_msg::action::Fibonacci>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<walle_msg::action::Fibonacci::Feedback>();
    auto& sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<walle_msg::action::Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
      if (goal_handle->is_canceling())
      {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }
      sequence.push_back(sequence[i] + sequence[i - 1]);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Publish feedback");

      loop_rate.sleep();
    }
    if (rclcpp::ok())
    {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
  }
};
}  

RCLCPP_COMPONENTS_REGISTER_NODE(walle_controller::SimpleActionServer)
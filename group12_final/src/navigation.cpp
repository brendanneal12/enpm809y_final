#include "navigation.hpp"

//===============================================

//===============================================
void Final::Navigation::set_initial_pose()
{
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
  message.header.frame_id = "map";
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = -1.59;
  message.pose.pose.orientation.x = 0.007825;
  message.pose.pose.orientation.y = 0.0;
  message.pose.pose.orientation.z = -0.7068252;
  message.pose.pose.orientation.w = 0.7073883;
  initial_pose_pub_->publish(message);
}
//===============================================
void Final::Navigation::send_goal()
{
  using namespace std::placeholders;

  if (!this->client_->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }

  // auto waypoints = robot_controller->get_waypoints();

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map"; // MAKE SURE TO INLCUDE THIS!!!!
  // goal_msg.pose.pose.position.x = waypoints[0][0];
  // goal_msg.pose.pose.position.y = waypoints[0][1];
  goal_msg.pose.pose.position.x = 1.5;
  goal_msg.pose.pose.position.y = -1.5;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.x = 0.0;
  goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = 0.0;
  goal_msg.pose.pose.orientation.w = 0.0;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&Navigation::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&Navigation::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&Navigation::result_callback, this, _1);

  client_->async_send_goal(goal_msg, send_goal_options);
}

//===============================================
void Final::Navigation::goal_response_callback(
    std::shared_future<GoalHandleNavigation::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

//===============================================
void Final::Navigation::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  (void)feedback;
  RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

//===============================================
void Final::Navigation::result_callback(
    const GoalHandleNavigation::WrappedResult &result)
{
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Final::Navigation>("navigation");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
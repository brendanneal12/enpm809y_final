#include "robot_controller.hpp"

void Final::RobotController::turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{

    // Look Get Aruco Marker ID.
    marker_id_ = "aruco_marker_" + std::to_string(msg->marker_ids[0]);
}

void Final::RobotController::advanced_camera_sub_cb_1_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
    }
}

void Final::RobotController::advanced_camera_sub_cb_2_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
    }
}

void Final::RobotController::advanced_camera_sub_cb_3_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
    }
}

void Final::RobotController::advanced_camera_sub_cb_4_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
    }
}

void Final::RobotController::advanced_camera_sub_cb_5_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
    }
}

void Final::RobotController::clock_sub_cb_(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    // Update current time attribute. Needed for broadcaster to work.
    current_time_ = msg->clock;
}

void Final::RobotController::part_broadcast_timer_cb_1_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Broadcast part position to tf tree.

    geometry_msgs::msg::TransformStamped part_transform_stamped;
    part_transform_stamped.header.stamp = current_time_;
    part_transform_stamped.header.frame_id = "logical_camera_link";
    part_transform_stamped.child_frame_id = "part_frame";

    part_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    part_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    part_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    part_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    // Send the transform
    part_tf_broadcaster_1_->sendTransform(part_transform_stamped);
}

void Final::RobotController::part_broadcast_timer_cb_2_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Broadcast part position to tf tree.

    geometry_msgs::msg::TransformStamped part_transform_stamped;
    part_transform_stamped.header.stamp = current_time_;
    part_transform_stamped.header.frame_id = "logical_camera_link";
    part_transform_stamped.child_frame_id = "part_frame";

    part_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    part_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    part_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    part_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    // Send the transform
    part_tf_broadcaster_2_->sendTransform(part_transform_stamped);
}

void Final::RobotController::part_broadcast_timer_cb_3_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Broadcast part position to tf tree.

    geometry_msgs::msg::TransformStamped part_transform_stamped;
    part_transform_stamped.header.stamp = current_time_;
    part_transform_stamped.header.frame_id = "logical_camera_link";
    part_transform_stamped.child_frame_id = "part_frame";

    part_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    part_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    part_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    part_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    // Send the transform
    part_tf_broadcaster_3_->sendTransform(part_transform_stamped);
}

void Final::RobotController::part_broadcast_timer_cb_4_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Broadcast part position to tf tree.

    geometry_msgs::msg::TransformStamped part_transform_stamped;
    part_transform_stamped.header.stamp = current_time_;
    part_transform_stamped.header.frame_id = "logical_camera_link";
    part_transform_stamped.child_frame_id = "part_frame";

    part_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    part_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    part_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    part_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    // Send the transform
    part_tf_broadcaster_4_->sendTransform(part_transform_stamped);
}

void Final::RobotController::part_broadcast_timer_cb_5_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // Broadcast part position to tf tree.

    geometry_msgs::msg::TransformStamped part_transform_stamped;
    part_transform_stamped.header.stamp = current_time_;
    part_transform_stamped.header.frame_id = "logical_camera_link";
    part_transform_stamped.child_frame_id = "part_frame";

    part_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    part_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    part_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

    part_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

    // Send the transform
    part_tf_broadcaster_5_->sendTransform(part_transform_stamped);
}

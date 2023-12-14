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
        part_type_1_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
        part_color_1_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
        Final::RobotController::part_broadcast_timer_cb_1_(msg);
    }
}

void Final::RobotController::advanced_camera_sub_cb_2_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
        part_type_2_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
        part_color_2_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
        Final::RobotController::part_broadcast_timer_cb_2_(msg);
    }
}

void Final::RobotController::advanced_camera_sub_cb_3_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
        part_type_3_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
        part_color_3_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
        Final::RobotController::part_broadcast_timer_cb_3_(msg);
    }
}

void Final::RobotController::advanced_camera_sub_cb_4_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
        part_type_4_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
        part_color_4_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
        Final::RobotController::part_broadcast_timer_cb_4_(msg);
    }
}

void Final::RobotController::advanced_camera_sub_cb_5_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the incoming message is not empty.
    if (msg->part_poses.size() != 0)
    {
        part_type_5_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
        part_color_5_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
        Final::RobotController::part_broadcast_timer_cb_5_(msg);
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

void Final::RobotController::part_frame_listener_1_()
{
    geometry_msgs::msg::TransformStamped part;

    try
    {
        // Look up transformation between detected part and odom frame and store.
        part = part_tf_buffer_1_->lookupTransform("odom", "part_frame", tf2::TimePointZero);
        std::array<double, 3> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        part_location[2] = part.transform.translation.z;
        Final::RobotController::add_seen_part_1(part_color_1_, part_type_1_, part_location);
    }
    catch (const tf2::TransformException &except)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
    }
}

void Final::RobotController::part_frame_listener_2_()
{
    geometry_msgs::msg::TransformStamped part;

    try
    {
        // Look up transformation between detected part and odom frame and store.
        part = part_tf_buffer_1_->lookupTransform("odom", "part_frame", tf2::TimePointZero);
        std::array<double, 3> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        part_location[2] = part.transform.translation.z;
        Final::RobotController::add_seen_part_1(part_color_2_, part_type_2_, part_location);
    }
    catch (const tf2::TransformException &except)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
    }
}

void Final::RobotController::part_frame_listener_3_()
{
    geometry_msgs::msg::TransformStamped part;

    try
    {
        // Look up transformation between detected part and odom frame and store.
        part = part_tf_buffer_1_->lookupTransform("odom", "part_frame", tf2::TimePointZero);
        std::array<double, 3> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        part_location[2] = part.transform.translation.z;
        Final::RobotController::add_seen_part_1(part_color_3_, part_type_3_, part_location);
    }
    catch (const tf2::TransformException &except)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
    }
}

void Final::RobotController::part_frame_listener_4_()
{
    geometry_msgs::msg::TransformStamped part;

    try
    {
        // Look up transformation between detected part and odom frame and store.
        part = part_tf_buffer_1_->lookupTransform("odom", "part_frame", tf2::TimePointZero);
        std::array<double, 3> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        part_location[2] = part.transform.translation.z;
        Final::RobotController::add_seen_part_1(part_color_4_, part_type_4_, part_location);
    }
    catch (const tf2::TransformException &except)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
    }
}

void Final::RobotController::part_frame_listener_5_()
{
    geometry_msgs::msg::TransformStamped part;

    try
    {
        // Look up transformation between detected part and odom frame and store.
        part = part_tf_buffer_1_->lookupTransform("odom", "part_frame", tf2::TimePointZero);
        std::array<double, 3> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        part_location[2] = part.transform.translation.z;
        Final::RobotController::add_seen_part_1(part_color_5_, part_type_5_, part_location);
    }
    catch (const tf2::TransformException &except)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
    }
}

std::string Final::RobotController::convert_part_type_to_string(uint8_t part_type)
{
    // Match numeric part type to string part type based off of message definition.
    if (part_type == mage_msgs::msg::Part::BATTERY)
        return "battery";
    else if (part_type == mage_msgs::msg::Part::PUMP)
        return "pump";
    else if (part_type == mage_msgs::msg::Part::REGULATOR)
        return "regulator";
    else if (part_type == mage_msgs::msg::Part::SENSOR)
        return "sensor";
    else
        return "unknown";
}

std::string Final::RobotController::convert_part_color_to_string(uint8_t part_color)
{
    // Match numeric part color to string part color based off of message definition.
    if (part_color == mage_msgs::msg::Part::RED)
        return "Red";
    else if (part_color == mage_msgs::msg::Part::GREEN)
        return "Green";
    else if (part_color == mage_msgs::msg::Part::BLUE)
        return "Blue";
    else if (part_color == mage_msgs::msg::Part::PURPLE)
        return "Purple";
    else if (part_color == mage_msgs::msg::Part::ORANGE)
        return "Orange";
    else
        return "Uknown";
}

bool Final::RobotController::check_duplicate_parts_1(const std::string &new_part_color, const std::string &new_part_type)
{
    // For all parts (rows) in custom data structure
    for (const auto &part : detected_parts_cam_1_)
    {
        // Compare new part color and type to existing members.
        if (std::get<0>(part) == new_part_color && std::get<1>(part) == new_part_type)
        {
            // Return true if there is a match
            return true;
        }
    }
    // False otherwise.
    return false;
}

void Final::RobotController::add_seen_part_1(const std::string &color, const std::string &type, const std::array<double, 3> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_1(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_1_.emplace_back(color, type, position);
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type);
    }
}

bool Final::RobotController::check_duplicate_parts_2(const std::string &new_part_color, const std::string &new_part_type)
{
    // For all parts (rows) in custom data structure
    for (const auto &part : detected_parts_cam_2_)
    {
        // Compare new part color and type to existing members.
        if (std::get<0>(part) == new_part_color && std::get<1>(part) == new_part_type)
        {
            // Return true if there is a match
            return true;
        }
    }
    // False otherwise.
    return false;
}

void Final::RobotController::add_seen_part_2(const std::string &color, const std::string &type, const std::array<double, 3> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_2(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_2_.emplace_back(color, type, position);
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type);
    }
}

bool Final::RobotController::check_duplicate_parts_3(const std::string &new_part_color, const std::string &new_part_type)
{
    // For all parts (rows) in custom data structure
    for (const auto &part : detected_parts_cam_3_)
    {
        // Compare new part color and type to existing members.
        if (std::get<0>(part) == new_part_color && std::get<1>(part) == new_part_type)
        {
            // Return true if there is a match
            return true;
        }
    }
    // False otherwise.
    return false;
}

void Final::RobotController::add_seen_part_3(const std::string &color, const std::string &type, const std::array<double, 3> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_3(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_3_.emplace_back(color, type, position);
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type);
    }
}

bool Final::RobotController::check_duplicate_parts_4(const std::string &new_part_color, const std::string &new_part_type)
{
    // For all parts (rows) in custom data structure
    for (const auto &part : detected_parts_cam_4_)
    {
        // Compare new part color and type to existing members.
        if (std::get<0>(part) == new_part_color && std::get<1>(part) == new_part_type)
        {
            // Return true if there is a match
            return true;
        }
    }
    // False otherwise.
    return false;
}

void Final::RobotController::add_seen_part_4(const std::string &color, const std::string &type, const std::array<double, 3> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_4(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_4_.emplace_back(color, type, position);
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type);
    }
}

bool Final::RobotController::check_duplicate_parts_5(const std::string &new_part_color, const std::string &new_part_type)
{
    // For all parts (rows) in custom data structure
    for (const auto &part : detected_parts_cam_5_)
    {
        // Compare new part color and type to existing members.
        if (std::get<0>(part) == new_part_color && std::get<1>(part) == new_part_type)
        {
            // Return true if there is a match
            return true;
        }
    }
    // False otherwise.
    return false;
}

void Final::RobotController::add_seen_part_5(const std::string &color, const std::string &type, const std::array<double, 3> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_5(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_5_.emplace_back(color, type, position);
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type);
    }
}

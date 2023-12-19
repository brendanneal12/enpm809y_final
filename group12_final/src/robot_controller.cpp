#include "robot_controller.hpp"

void Final::RobotController::turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    // If the marker has not been gotten
    if (!marker_got_)
    {
        // Get Aruco Marker ID.
        marker_id_ = "aruco_" + std::to_string(msg->marker_ids[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got Aruco Marker: " << marker_id_);
        marker_got_ = true;
        // Stop subscribing to the RGB camera.
        turtle_camera_subscription_.reset();
    }
}

void Final::RobotController::advanced_camera_sub_cb_1_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the part from camera 1 has not been gotten
    if (!part_got_cam_1_)
    {
        // If the incoming message is not empty.
        if (msg->part_poses.size() != 0)
        {
            // Grab the part type and color
            part_type_1_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
            part_color_1_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
            // Broadcast the part's pose
            Final::RobotController::part_broadcast_timer_cb_1_(msg);
            // Stop subscribing to camera 1.
            advanced_camera_subscription_1_.reset();
            part_got_cam_1_ = true;
        }
    }
}

void Final::RobotController::advanced_camera_sub_cb_2_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the part from camera 2 has not been gotten
    if (!part_got_cam_2_)
    {
        // If the incoming message is not empty.
        if (msg->part_poses.size() != 0)
        {
            // Grab the part type and color
            part_type_2_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
            part_color_2_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
            // Broadcast the pose
            Final::RobotController::part_broadcast_timer_cb_2_(msg);
            // Stop subscribing to camera 2.
            advanced_camera_subscription_2_.reset();
            part_got_cam_2_ = true;
        }
    }
}

void Final::RobotController::advanced_camera_sub_cb_3_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the part from camera 3 has not been gotten
    if (!part_got_cam_3_)
    {
        // If the incoming message is not empty.
        if (msg->part_poses.size() != 0)
        {
            // Grab the part type and color
            part_type_3_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
            part_color_3_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
            // Broadcast the Pose
            Final::RobotController::part_broadcast_timer_cb_3_(msg);
            // Stop subscribing to camera 3.
            advanced_camera_subscription_3_.reset();
            part_got_cam_3_ = true;
        }
    }
}

void Final::RobotController::advanced_camera_sub_cb_4_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the part from camera 4 has not been gotten
    if (!part_got_cam_4_)
    {
        // If the incoming message is not empty.
        if (msg->part_poses.size() != 0)
        {
            // Grab the part type and color
            part_type_4_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
            part_color_4_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
            // Broadcast the pose
            Final::RobotController::part_broadcast_timer_cb_4_(msg);
            // Stop subscribing to camera 4
            advanced_camera_subscription_4_.reset();
            part_got_cam_4_ = true;
        }
    }
}

void Final::RobotController::advanced_camera_sub_cb_5_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // If the part from camera 5 has not been gotten
    if (!part_got_cam_5_)
    {
        // If the incoming message is not empty.
        if (msg->part_poses.size() != 0)
        {
            // Grab the part type and color
            part_type_5_ = Final::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
            part_color_5_ = Final::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);
            // Broadcast the pose
            Final::RobotController::part_broadcast_timer_cb_5_(msg);
            // Stop subscribing to camera 5.
            advanced_camera_subscription_5_.reset();
            part_got_cam_5_ = true;
        }
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
    part_transform_stamped.header.frame_id = "camera1_frame";
    part_transform_stamped.child_frame_id = "part_1_frame";

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
    part_transform_stamped.header.frame_id = "camera2_frame";
    part_transform_stamped.child_frame_id = "part_2_frame";

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
    part_transform_stamped.header.frame_id = "camera3_frame";
    part_transform_stamped.child_frame_id = "part_3_frame";

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
    part_transform_stamped.header.frame_id = "camera4_frame";
    part_transform_stamped.child_frame_id = "part_4_frame";

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
    part_transform_stamped.header.frame_id = "camera5_frame";
    part_transform_stamped.child_frame_id = "part_5_frame";

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
        // Look up transformation between detected part and map frame and store.
        part = part_tf_buffer_1_->lookupTransform("map", "part_1_frame", tf2::TimePointZero);
        std::array<double, 2> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
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
        // Look up transformation between detected part and map frame and store.
        part = part_tf_buffer_1_->lookupTransform("map", "part_2_frame", tf2::TimePointZero);
        std::array<double, 2> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        Final::RobotController::add_seen_part_2(part_color_2_, part_type_2_, part_location);
    }
    catch (const tf2::TransformException &except)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), except.what());
    }
}

void Final::RobotController::part_frame_listener_3_()
{
    geometry_msgs::msg::TransformStamped part;

    try
    {
        // Look up transformation between detected part and map frame and store.
        part = part_tf_buffer_1_->lookupTransform("map", "part_3_frame", tf2::TimePointZero);
        std::array<double, 2> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        Final::RobotController::add_seen_part_3(part_color_3_, part_type_3_, part_location);
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
        // Look up transformation between detected part and map frame and store.
        part = part_tf_buffer_1_->lookupTransform("map", "part_4_frame", tf2::TimePointZero);
        std::array<double, 2> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        Final::RobotController::add_seen_part_4(part_color_4_, part_type_4_, part_location);
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
        // Look up transformation between detected part and map frame and store.
        part = part_tf_buffer_1_->lookupTransform("map", "part_5_frame", tf2::TimePointZero);
        std::array<double, 2> part_location;
        part_location[0] = part.transform.translation.x;
        part_location[1] = part.transform.translation.y;
        Final::RobotController::add_seen_part_5(part_color_5_, part_type_5_, part_location);
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
        return "red";
    else if (part_color == mage_msgs::msg::Part::GREEN)
        return "green";
    else if (part_color == mage_msgs::msg::Part::BLUE)
        return "blue";
    else if (part_color == mage_msgs::msg::Part::PURPLE)
        return "purple";
    else if (part_color == mage_msgs::msg::Part::ORANGE)
        return "orange";
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

void Final::RobotController::add_seen_part_1(const std::string &color, const std::string &type, const std::array<double, 2> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_1(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_1_.emplace_back(color, type, position);
        parts_in_world_[std::make_tuple(color, type)] = position;
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type << " at (" << parts_in_world_[std::make_tuple(color, type)][0] << "," << parts_in_world_[std::make_tuple(color, type)][1] << ")");
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

void Final::RobotController::add_seen_part_2(const std::string &color, const std::string &type, const std::array<double, 2> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_2(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_2_.emplace_back(color, type, position);
        parts_in_world_[std::make_tuple(color, type)] = position;
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type << " at (" << parts_in_world_[std::make_tuple(color, type)][0] << "," << parts_in_world_[std::make_tuple(color, type)][1] << ")");
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

void Final::RobotController::add_seen_part_3(const std::string &color, const std::string &type, const std::array<double, 2> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_3(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_3_.emplace_back(color, type, position);
        parts_in_world_[std::make_tuple(color, type)] = position;
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type << " at (" << parts_in_world_[std::make_tuple(color, type)][0] << "," << parts_in_world_[std::make_tuple(color, type)][1] << ")");
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

void Final::RobotController::add_seen_part_4(const std::string &color, const std::string &type, const std::array<double, 2> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_4(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_4_.emplace_back(color, type, position);
        parts_in_world_[std::make_tuple(color, type)] = position;
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type << " at (" << parts_in_world_[std::make_tuple(color, type)][0] << "," << parts_in_world_[std::make_tuple(color, type)][1] << ")");
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

void Final::RobotController::add_seen_part_5(const std::string &color, const std::string &type, const std::array<double, 2> &position)
{
    // Check if new detected part matches an existing member.
    bool duplicate_check = Final::RobotController::check_duplicate_parts_5(color, type);
    // If it is unique
    if (!duplicate_check)
    {
        // Add new part to data structure.
        detected_parts_cam_5_.emplace_back(color, type, position);
        parts_in_world_.emplace(std::make_tuple(color, type), position);
        RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type << " at (" << parts_in_world_[std::make_tuple(color, type)][0] << "," << parts_in_world_[std::make_tuple(color, type)][1] << ")");
    }
}

void Final::RobotController::generate_waypoints_from_params()
{
    // Check detected marker id against parameter values
    if (this->has_parameter(marker_id_))
    {
        // Set the marker instruction based on parameters
        marker_instruction_ = this->get_parameter(marker_id_).get_value<std::string>();
    }

    // Decide which set of waypoints to follow based on ht marker instruction.
    if (marker_instruction_ == std::get<0>(aruco_0_waypoints_[0]))
    {
        use_waypoints_ = aruco_0_waypoints_;
    }

    else if (marker_instruction_ == std::get<0>(aruco_1_waypoints_[0]))
    {
        use_waypoints_ = aruco_1_waypoints_;
    }
}

rcl_interfaces::msg::SetParametersResult Final::RobotController::parameters_cb(const std::vector<rclcpp::Parameter> &parameters)
{
    // Watch for change in parameters and set the result.
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
        if (param.get_name() == "aruco_0")
        {
            aruco_0_ = param.as_string();
        }
        else if (param.get_name() == "aruco_0.wp1.type")
        {
            a0_wp1_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_0.wp1.color")
        {
            a0_wp1_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_0.wp2.type")
        {
            a0_wp2_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_0.wp2.color")
        {
            a0_wp2_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_0.wp3.type")
        {
            a0_wp3_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_0.wp3.color")
        {
            a0_wp3_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_0.wp4.type")
        {
            a0_wp4_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_0.wp4.color")
        {
            a0_wp4_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_0.wp5.type")
        {
            a0_wp5_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_0.wp5.color")
        {
            a0_wp5_color_ = param.as_string();
        }

        if (param.get_name() == "aruco_1")
        {
            aruco_1_ = param.as_string();
        }
        else if (param.get_name() == "aruco_1.wp1.type")
        {
            a1_wp1_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_1.wp1.color")
        {
            a1_wp1_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_1.wp2.type")
        {
            a1_wp2_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_1.wp2.color")
        {
            a1_wp2_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_1.wp3.type")
        {
            a1_wp3_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_1.wp3.color")
        {
            a1_wp3_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_1.wp4.type")
        {
            a1_wp4_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_1.wp4.color")
        {
            a1_wp4_color_ = param.as_string();
        }

        else if (param.get_name() == "aruco_1.wp5.type")
        {
            a1_wp5_type_ = param.as_string();
        }
        else if (param.get_name() == "aruco_1.wp5.color")
        {
            a1_wp5_color_ = param.as_string();
        }

        else
        {
            result.successful = false;
            result.reason = "parameter not authorized to be modified";
        }
    }
    return result;
}

void Final::RobotController::amcl_sub_cb_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Grab the current pose of the robot from amcl
    robot_current_pose_ = *msg;
}

void Final::RobotController::set_initial_pose()
{
    // Set the initial pose of the robot from odom.
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting Initial Pose");
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header.frame_id = "map";
    message.pose.pose.position.x = robot_initial_pose_.pose.pose.position.x;
    message.pose.pose.position.y = robot_initial_pose_.pose.pose.position.y;
    message.pose.pose.orientation.x = robot_initial_pose_.pose.pose.orientation.x;
    message.pose.pose.orientation.y = robot_initial_pose_.pose.pose.orientation.y;
    message.pose.pose.orientation.z = robot_initial_pose_.pose.pose.orientation.z;
    message.pose.pose.orientation.w = robot_initial_pose_.pose.pose.orientation.w;
    initial_pose_pub_->publish(message);
}
//===============================================
void Final::RobotController::send_goal()
{
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Action server not available after waiting");
        rclcpp::shutdown();
    }

    // Generate the desired order of waypoints
    generate_waypoints_from_params();
    // Initialize the goal message and grab the appropriate (x,y) from the waypoints.
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = parts_in_world_[std::make_tuple(std::get<1>(use_waypoints_[movement_ctr_]), std::get<2>(use_waypoints_[movement_ctr_]))][0];
    goal_msg.pose.pose.position.y = parts_in_world_[std::make_tuple(std::get<1>(use_waypoints_[movement_ctr_]), std::get<2>(use_waypoints_[movement_ctr_]))][1];
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 0.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&RobotController::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&RobotController::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&RobotController::result_callback, this, _1);

    client_->async_send_goal(goal_msg, send_goal_options);

    movement_ctr_ += 1;
}

//===============================================
void Final::RobotController::goal_response_callback(
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
void Final::RobotController::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
    RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

//===============================================
void Final::RobotController::result_callback(
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

void Final::RobotController::odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Grab the robot's initial position and orientation.
    if (!initial_pose_set_)
    {
        initial_pose_set_ = true;
        robot_initial_pose_ = *msg;
        Final::RobotController::set_initial_pose();
    }
}

void Final::RobotController::nav_timer_cb_()
{
    // If all required information has been retrieved
    if ((part_got_cam_1_) && (part_got_cam_2_) && (part_got_cam_3_) && (part_got_cam_4_) && (part_got_cam_5_) && (marker_got_))
    {
        // Sleep for 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));
        // send the goal
        send_goal();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Final::RobotController>("robot_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
}

#include "robot_controller.hpp"

void Final::RobotController::turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{

    // Look Get Aruco Marker ID.
    marker_id_ = "aruco_marker_" + std::to_string(msg->marker_ids[0]);

}
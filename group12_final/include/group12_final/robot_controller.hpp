#pragma once
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2/exceptions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosgraph_msgs/msg/clock.hpp>

/**
 * @brief Namespace Used for Final Project
 */
namespace Final
{
    /**
     * @brief Robot Controller Node
     *
     */
    class RobotController : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor
         * @param node_name
         */
        RobotController(std::string node_name) : Node(node_name)
        {

            // Declare parameters -> Grabs from .yaml
            // this->declare_parameter("aruco_0", "right_90");
            // aruco_marker_0_ = this->get_parameter("aruco_marker_0").as_string();

            // this->declare_parameter("aruco_1", "left_90");
            // aruco_marker_1_ = this->get_parameter("aruco_marker_1").as_string();

            // this->declare_parameter("aruco_2", "end");
            // aruco_marker_2_ = this->get_parameter("aruco_marker_2").as_string();

            // Initialize the transform broadcasters
            part_tf_broadcaster_1_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            part_tf_broadcaster_2_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            part_tf_broadcaster_3_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            part_tf_broadcaster_4_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            part_tf_broadcaster_5_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

            // Load buffers of transforms and associated listeners
            // Camera 1
            part_tf_buffer_1_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            part_tf_buffer_1_->setUsingDedicatedThread(true);
            part_tf_listener_1 = std::make_shared<tf2_ros::TransformListener>(*part_tf_buffer_1_);

            // Camera 2
            part_tf_buffer_2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            part_tf_buffer_2_->setUsingDedicatedThread(true);
            part_tf_listener_2 = std::make_shared<tf2_ros::TransformListener>(*part_tf_buffer_2_);

            // Camera 3
            part_tf_buffer_3_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            part_tf_buffer_3_->setUsingDedicatedThread(true);
            part_tf_listener_3 = std::make_shared<tf2_ros::TransformListener>(*part_tf_buffer_3_);

            // Camera 4
            part_tf_buffer_4_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            part_tf_buffer_4_->setUsingDedicatedThread(true);
            part_tf_listener_4 = std::make_shared<tf2_ros::TransformListener>(*part_tf_buffer_4_);

            // Camera 5
            part_tf_buffer_5_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            part_tf_buffer_5_->setUsingDedicatedThread(true);
            part_tf_listener_5 = std::make_shared<tf2_ros::TransformListener>(*part_tf_buffer_5_);

            // Set up marker subscription and bind it to a callback.
            turtle_camera_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", rclcpp::SensorDataQoS(),
                                                                                                              std::bind(&RobotController::turtle_camera_sub_cb_, this, std::placeholders::_1));

            // Set up logical camera 1 subscription and bind it to a callback.
            advanced_camera_subscription_1_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera1/image", rclcpp::SensorDataQoS(),
                                                                                                                    std::bind(&RobotController::advanced_camera_sub_cb_1_, this, std::placeholders::_1));

            // Set up logical camera 2 subscription and bind it to a callback.
            advanced_camera_subscription_2_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera2/image", rclcpp::SensorDataQoS(),
                                                                                                                    std::bind(&RobotController::advanced_camera_sub_cb_2_, this, std::placeholders::_1));

            // Set up logical camera 3 subscription and bind it to a callback.
            advanced_camera_subscription_3_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera3/image", rclcpp::SensorDataQoS(),
                                                                                                                    std::bind(&RobotController::advanced_camera_sub_cb_3_, this, std::placeholders::_1));

            // Set up logical camera 4 subscription and bind it to a callback.
            advanced_camera_subscription_4_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera4/image", rclcpp::SensorDataQoS(),
                                                                                                                    std::bind(&RobotController::advanced_camera_sub_cb_4_, this, std::placeholders::_1));

            // Set up logical camera 5 subscription and bind it to a callback.
            advanced_camera_subscription_5_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera5/image", rclcpp::SensorDataQoS(),
                                                                                                                    std::bind(&RobotController::advanced_camera_sub_cb_5_, this, std::placeholders::_1));

            // Set up clock subscriptions and bind it to a callback.
            clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS(),
                                                                                       std::bind(&RobotController::clock_sub_cb_, this, std::placeholders::_1));
        }

    private:
        // ======================================== parameters ========================================
        // ======================================== attributes ========================================

        // Subscribers
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr turtle_camera_subscription_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_1_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_2_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_3_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_4_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_5_;
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

        // Broadcasters
        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_1_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_1_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_2_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_2_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_3_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_3_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_4_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_4_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_5_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_5_;

        // Listeners
        // Camera 1
        std::shared_ptr<tf2_ros::TransformListener> part_tf_listener_1{nullptr};
        std::unique_ptr<tf2_ros::Buffer> part_tf_buffer_1_;
        rclcpp::TimerBase::SharedPtr part_listener_timer_1_;

        // Camera 2
        std::shared_ptr<tf2_ros::TransformListener> part_tf_listener_2{nullptr};
        std::unique_ptr<tf2_ros::Buffer> part_tf_buffer_2_;
        rclcpp::TimerBase::SharedPtr part_listener_timer_2_;

        // Camera 3
        std::shared_ptr<tf2_ros::TransformListener> part_tf_listener_3{nullptr};
        std::unique_ptr<tf2_ros::Buffer> part_tf_buffer_3_;
        rclcpp::TimerBase::SharedPtr part_listener_timer_3_;

        // Camera 4
        std::shared_ptr<tf2_ros::TransformListener> part_tf_listener_4{nullptr};
        std::unique_ptr<tf2_ros::Buffer> part_tf_buffer_4_;
        rclcpp::TimerBase::SharedPtr part_listener_timer_4_;

        // Camera 5
        std::shared_ptr<tf2_ros::TransformListener> part_tf_listener_5{nullptr};
        std::unique_ptr<tf2_ros::Buffer> part_tf_buffer_5_;
        rclcpp::TimerBase::SharedPtr part_listener_timer_5_;

        // Marker Attributes
        std::string marker_id_;

        // Sim Attributes
        rclcpp::Time current_time_;
        // ======================================== methods ===========================================

        /**
         * @brief Subscriber callback to update aruco marker location.
         * @param msg
         */
        void turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        /**
         * @brief Subscriber callback to update part location from Camera 1.
         * @param msg
         */
        void advanced_camera_sub_cb_1_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Subscriber callback to update part location from Camera 2.
         * @param msg
         */
        void advanced_camera_sub_cb_2_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Subscriber callback to update part location from Camera 3.
         * @param msg
         */
        void advanced_camera_sub_cb_3_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Subscriber callback to update part location from Camera 4.
         * @param msg
         */
        void advanced_camera_sub_cb_4_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Subscriber callback to update part location from Camera 5.
         * @param msg
         */
        void advanced_camera_sub_cb_5_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Subscriber callback to update current time.
         * @param msg
         */

        void clock_sub_cb_(const rosgraph_msgs::msg::Clock::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 1.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_1_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 1.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_2_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 1.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_3_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 1.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_4_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 1.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_5_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    }; // Class RobotController
} // Namespace Final
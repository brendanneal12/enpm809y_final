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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"


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
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        RobotController(std::string node_name) : Node(node_name)
        {
            // initialize the client
            client_ =
                rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            // initialize the publisher
            initial_pose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "initialpose", 10);

            // Declare parameters -> Grabs from .yaml via launch file.
            // Aruco 0
            this->declare_parameter("aruco_0", "aruco_0");
            aruco_0_ = this->get_parameter("aruco_0").as_string();
            this->declare_parameter("aruco_0.wp1.type", "battery");
            a0_wp1_type_ = this->get_parameter("aruco_0.wp1.type").as_string();
            this->declare_parameter("aruco_0.wp1.color", "green");
            a0_wp1_color_ = this->get_parameter("aruco_0.wp1.color").as_string();
            this->declare_parameter("aruco_0.wp2.type", "battery");
            a0_wp2_type_ = this->get_parameter("aruco_0.wp2.type").as_string();
            this->declare_parameter("aruco_0.wp2.color", "red");
            a0_wp2_color_ = this->get_parameter("aruco_0.wp2.color").as_string();
            this->declare_parameter("aruco_0.wp3.type", "battery");
            a0_wp3_type_ = this->get_parameter("aruco_0.wp3.type").as_string();
            this->declare_parameter("aruco_0.wp3.color", "orange");
            a0_wp3_color_ = this->get_parameter("aruco_0.wp3.color").as_string();
            this->declare_parameter("aruco_0.wp4.type", "battery");
            a0_wp4_type_ = this->get_parameter("aruco_0.wp4.type").as_string();
            this->declare_parameter("aruco_0.wp4.color", "purple");
            a0_wp4_color_ = this->get_parameter("aruco_0.wp4.color").as_string();
            this->declare_parameter("aruco_0.wp5.type", "battery");
            a0_wp5_type_ = this->get_parameter("aruco_0.wp5.type").as_string();
            this->declare_parameter("aruco_0.wp5.color", "blue");
            a0_wp5_color_ = this->get_parameter("aruco_0.wp5.color").as_string();

            // Order waypoints by part color and part type.
            aruco_0_waypoints_.emplace_back(aruco_0_, a0_wp1_color_, a0_wp1_type_);
            aruco_0_waypoints_.emplace_back(aruco_0_, a0_wp2_color_, a0_wp2_type_);
            aruco_0_waypoints_.emplace_back(aruco_0_, a0_wp3_color_, a0_wp3_type_);
            aruco_0_waypoints_.emplace_back(aruco_0_, a0_wp4_color_, a0_wp4_type_);
            aruco_0_waypoints_.emplace_back(aruco_0_, a0_wp5_color_, a0_wp5_type_);

            // Aruco 1
            this->declare_parameter("aruco_1", "aruco_1");
            aruco_1_ = this->get_parameter("aruco_1").as_string();
            this->declare_parameter("aruco_1.wp1.type", "battery");
            a1_wp1_type_ = this->get_parameter("aruco_1.wp1.type").as_string();
            this->declare_parameter("aruco_1.wp1.color", "blue");
            a1_wp1_color_ = this->get_parameter("aruco_1.wp1.color").as_string();
            this->declare_parameter("aruco_1.wp2.type", "battery");
            a1_wp2_type_ = this->get_parameter("aruco_1.wp2.type").as_string();
            this->declare_parameter("aruco_1.wp2.color", "green");
            a1_wp2_color_ = this->get_parameter("aruco_1.wp2.color").as_string();
            this->declare_parameter("aruco_1.wp3.type", "battery");
            a1_wp3_type_ = this->get_parameter("aruco_1.wp3.type").as_string();
            this->declare_parameter("aruco_1.wp3.color", "orange");
            a1_wp3_color_ = this->get_parameter("aruco_1.wp3.color").as_string();
            this->declare_parameter("aruco_1.wp4.type", "red");
            a1_wp4_type_ = this->get_parameter("aruco_1.wp4.type").as_string();
            this->declare_parameter("aruco_1.wp4.color", "red");
            a1_wp4_color_ = this->get_parameter("aruco_1.wp4.color").as_string();
            this->declare_parameter("aruco_1.wp5.type", "battery");
            a1_wp5_type_ = this->get_parameter("aruco_1.wp5.type").as_string();
            this->declare_parameter("aruco_1.wp5.color", "purple");
            a1_wp5_color_ = this->get_parameter("aruco_1.wp5.color").as_string();


            // Order waypoints by part color and type.
            aruco_1_waypoints_.emplace_back(aruco_1_, a1_wp1_color_, a1_wp1_type_);
            aruco_1_waypoints_.emplace_back(aruco_1_, a1_wp2_color_, a1_wp2_type_);
            aruco_1_waypoints_.emplace_back(aruco_1_, a1_wp3_color_, a1_wp3_type_);
            aruco_1_waypoints_.emplace_back(aruco_1_, a1_wp4_color_, a1_wp4_type_);
            aruco_1_waypoints_.emplace_back(aruco_1_, a1_wp5_color_, a1_wp5_type_);

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
            // Set up robot pose subscriptions and bind it to a callback.
            amcl_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::SensorDataQoS(),
                                                                                                          std::bind(&RobotController::amcl_sub_cb_, this, std::placeholders::_1));
            // Listener Timer 1
            part_listener_timer_1_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotController::part_frame_listener_1_, this));

            // Listener Timer 2
            part_listener_timer_2_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotController::part_frame_listener_2_, this));

            // Listener Timer 3
            part_listener_timer_3_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotController::part_frame_listener_3_, this));

            // Listener Timer 4
            part_listener_timer_4_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotController::part_frame_listener_4_, this));

            // Listener Timer 5
            part_listener_timer_5_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotController::part_frame_listener_5_, this));

            parameter_cb_ = this->add_on_set_parameters_callback(std::bind(&RobotController::parameters_cb, this, std::placeholders::_1));

            // Set up odometry subscription and bind it to a callback.
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotController::odom_sub_cb_, this, std::placeholders::_1));

            // Set up navigation timer and bind it to a callback.
            nav_timer_ = this->create_wall_timer(std::chrono::milliseconds(20000), std::bind(&RobotController::nav_timer_cb_, this));
        }
    private:
        // ======================================== parameters ========================================
        //Parameter callback
        OnSetParametersCallbackHandle::SharedPtr parameter_cb_;
        // Aruco 0
        std::string aruco_0_;
        std::string a0_wp1_color_;
        std::string a0_wp1_type_;
        std::string a0_wp2_color_;
        std::string a0_wp2_type_;
        std::string a0_wp3_color_;
        std::string a0_wp3_type_;
        std::string a0_wp4_color_;
        std::string a0_wp4_type_;
        std::string a0_wp5_color_;
        std::string a0_wp5_type_;

        // Aruco 1
        std::string aruco_1_;
        std::string a1_wp1_color_;
        std::string a1_wp1_type_;
        std::string a1_wp2_color_;
        std::string a1_wp2_type_;
        std::string a1_wp3_color_;
        std::string a1_wp3_type_;
        std::string a1_wp4_color_;
        std::string a1_wp4_type_;
        std::string a1_wp5_color_;
        std::string a1_wp5_type_;

        // Aruco 0 Waypoints
        std::vector<std::tuple<std::string, std::string, std::string>> aruco_0_waypoints_;

        // Aruco 1 Waypoints
        std::vector<std::tuple<std::string, std::string, std::string>> aruco_1_waypoints_;
        // ======================================== attributes ========================================
        // Subscribers
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr turtle_camera_subscription_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_1_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_2_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_3_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_4_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_5_;
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

        // Client
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
        //   rclcpp::TimerBase::SharedPtr timer_;

        // Broadcasters
        //Camera 1
        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_1_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_1_;

        //Camera 2
        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_2_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_2_;

        // Camera 3
        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_3_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_3_;

        // Camera 4
        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_4_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_4_;

        // Camera 5
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

        // Robot Attributes
        geometry_msgs::msg::PoseWithCovarianceStamped robot_current_pose_;

        nav_msgs::msg::Odometry robot_initial_pose_;

        // Marker Attributes
        std::string marker_id_;
        bool marker_got_{false};
        std::string marker_instruction_;

        // Sim Attributes
        rclcpp::Time current_time_;

        // Part Storage Attributes
        // Camera 1
        std::string part_type_1_;
        std::string part_color_1_;
        std::vector<std::tuple<std::string, std::string, std::array<double, 2>>> detected_parts_cam_1_;
        bool part_got_cam_1_{false};

        // Camera 2
        std::string part_type_2_;
        std::string part_color_2_;
        std::vector<std::tuple<std::string, std::string, std::array<double, 2>>> detected_parts_cam_2_;
        bool part_got_cam_2_{false};

        // Camera 3
        std::string part_type_3_;
        std::string part_color_3_;
        std::vector<std::tuple<std::string, std::string, std::array<double, 2>>> detected_parts_cam_3_;
        bool part_got_cam_3_{false};

        // Camera 4
        std::string part_type_4_;
        std::string part_color_4_;
        std::vector<std::tuple<std::string, std::string, std::array<double, 2>>> detected_parts_cam_4_;
        bool part_got_cam_4_{false};

        // Camera 5
        std::string part_type_5_;
        std::string part_color_5_;
        std::vector<std::tuple<std::string, std::string, std::array<double, 2>>> detected_parts_cam_5_;
        bool part_got_cam_5_{false};

        // Map to store parts in the world.
        std::map<std::tuple<std::string, std::string>, std::array<double, 2>> parts_in_world_;

        // Generated Waypoints
        std::vector<std::tuple<std::string, std::string, std::string>>  use_waypoints_;

        // Initial Pose Boolean
        bool initial_pose_set_{false};

        // Nav Timer
        rclcpp::TimerBase::SharedPtr nav_timer_;
        int movement_ctr_{0};

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
         * @brief Subscriber callback to update current time.
         * @param msg
         */

        void amcl_sub_cb_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 1.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_1_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 2.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_2_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 3.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_3_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 4.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_4_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf. from camera 5.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_5_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Method to listen for transformation updates for parts from camera 1.
         */
        void part_frame_listener_1_();

        /**
         * @brief Method to listen for transformation updates for parts from camera 2.
         */
        void part_frame_listener_2_();

        /**
         * @brief Method to listen for transformation updates for parts from camera 3.
         */
        void part_frame_listener_3_();

        /**
         * @brief Method to listen for transformation updates for parts from camera 4.
         */
        void part_frame_listener_4_();

        /**
         * @brief Method to listen for transformation updates for parts from camera 5.
         */
        void part_frame_listener_5_();

        /**
         * @brief Method to ensure parts seen are unique.
         * @param new_part_color
         * @param new_part_type
         * @return true
         * @return false
         */
        bool check_duplicate_parts_1(const std::string &new_part_color, const std::string &new_part_type);
        bool check_duplicate_parts_2(const std::string &new_part_color, const std::string &new_part_type);
        bool check_duplicate_parts_3(const std::string &new_part_color, const std::string &new_part_type);
        bool check_duplicate_parts_4(const std::string &new_part_color, const std::string &new_part_type);
        bool check_duplicate_parts_5(const std::string &new_part_color, const std::string &new_part_type);

        /**
         * @brief Method to add seen parts to data structure.
         * @param color
         * @param type
         * @param position
         */
        void add_seen_part_1(const std::string &color, const std::string &type, const std::array<double, 2> &position);
        void add_seen_part_2(const std::string &color, const std::string &type, const std::array<double, 2> &position);
        void add_seen_part_3(const std::string &color, const std::string &type, const std::array<double, 2> &position);
        void add_seen_part_4(const std::string &color, const std::string &type, const std::array<double, 2> &position);
        void add_seen_part_5(const std::string &color, const std::string &type, const std::array<double, 2> &position);

        /**
         * @brief Convert a part type to a string
         *
         * @param part_type
         * @return battery
         * @return regulator
         * @return sensor
         * @return pump
         * @return unknown
         */
        std::string convert_part_type_to_string(uint8_t part_type);

        /**
         * @brief Convert a part color to a string
         *
         * @param part_color
         * @return red
         * @return green
         * @return blue
         * @return purple
         * @return orange
         * @return unknown
         */
        std::string convert_part_color_to_string(uint8_t part_color);

        /**
         * @brief Parameter callback to watch for changes of parameters.
         * @param parameters
         * @return result
         */
        rcl_interfaces::msg::SetParametersResult parameters_cb(const std::vector<rclcpp::Parameter> &parameters);

        /**
         * @brief Generates X-Y waypoints from parameters.
         */
        void generate_waypoints_from_params();

        /**
         * @brief Method to send initial pose of robot using the client
         *
         */
        void set_initial_pose();

        /**
         * @brief Response from the server after sending the goal
         */
        void goal_response_callback(
            std::shared_future<GoalHandleNavigation::SharedPtr> future);
        /**
         * @brief Feedback received while the robot is driving towards the goal
         *
         * @param feedback
         */
        void feedback_callback(
            GoalHandleNavigation::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        /**
         * @brief Result after the action has completed
         *
         * @param result
         */
        void result_callback(const GoalHandleNavigation::WrappedResult &result);
        /**
         * @brief Method to build and send a goal using the client
         *
         */
        void send_goal();

        /**
         * @brief Subscriber callback to update current position of turtlebot.
         * @param msg
         */

        void odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
         * @brief timer callback that updates the next waypoint the robot must go to.
        */

        void nav_timer_cb_();

    }; // Class RobotController
} // Namespace Final
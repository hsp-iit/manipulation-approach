// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

namespace approach_path_planning{

class planner : public rclcpp_lifecycle::LifecycleNode 
{
private:
    // ------------ Params
    std::string _base_frame;
    std::string _costmap_topic_name;
    //TODO change in enum
    int _state;
    // TODO subscribe to robot footprint for extracting this param:
    double _robot_radius;

    // ------------ Action client vars:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr _nav_client;
    rclcpp::CallbackGroup::SharedPtr _nav_callback_group;
    rclcpp::executors::SingleThreadedExecutor _nav_executor;
    std::shared_future<rclcpp_action::ClientGoalHandle
        <nav2_msgs::action::NavigateToPose>::SharedPtr> _future_goal_handle;
    nav2_msgs::action::NavigateToPose::Goal _goal_action;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr _nav_goal_handle;
    // Action subs
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr _nav_feedback_sub;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr _nav_result_sub;
    // The (non-spinning) client node used to invoke the action client
    rclcpp::Node::SharedPtr _client_node;
    const std::chrono::milliseconds  _server_timeout = 100ms;
    // ------------ Subscribers
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr _costmap_sub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr _contours_sub;
    // ------------ TF2
    std::unique_ptr<tf2_ros::Buffer> _buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
    // ------------ Memory
    std::shared_ptr<nav2_msgs::msg::Costmap> _local_costmap;
    // ------------ Callbacks
    void costmap_update(nav2_msgs::msg::Costmap::SharedPtr msg);
    void contours_update(visualization_msgs::msg::Marker::SharedPtr mrk_msg);
public:
    planner(const rclcpp::NodeOptions & options);

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
    
};
}
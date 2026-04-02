// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "surface_detector_interfaces/msg/detection_results.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "Eigen/Core"
#include <mutex>
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

namespace approach_path_planning{

class planner : public rclcpp_lifecycle::LifecycleNode 
{
private:
    class Cell
    {
    public:
        int id;
        double score;
        double path_lenght;
        // We invert the direction because we want the minimum score to be on top of priority_queue
        bool operator>(Cell a) const{
            return this->score > a.score;;
        };
    };
    // ------------ Params
    std::string base_frame_;
    std::string costmap_topic_name_;
    std::string contours_topic_name_;
    //TODO change in enum
    int state_;
    // TODO subscribe to robot footprint for extracting this param:
    double robot_radius_;
    // Costmap value above which we consider it lethal
    unsigned int max_costmap_val_;
    // Maximum distance tolerated between the robot and the object to grasp
    double dist_threshold_;

    // ------------ Action client vars:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::CallbackGroup::SharedPtr nav_callback_group_;
    rclcpp::executors::SingleThreadedExecutor nav_executor_;
    std::shared_future<rclcpp_action::ClientGoalHandle
        <nav2_msgs::action::NavigateToPose>::SharedPtr> future_goal_handle_;
    nav2_msgs::action::NavigateToPose::Goal goal_action_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle_;
    // Action subs
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr nav_feedback_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_result_sub_;
    // The (non-spinning) client node used to invoke the action client
    rclcpp::Node::SharedPtr client_node_;
    const std::chrono::milliseconds  server_timeout_ = 100ms;
    // ------------ Subscribers
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<surface_detector_interfaces::msg::DetectionResults>::SharedPtr contours_sub_;
    // ------------ Publishers
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr candidate_marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr filtered_candidate_marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    // ------------ TF2
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    // ------------ Memory
    nav2_costmap_2d::Costmap2D global_costmap_;
    bool costmap_received_;
    std::mutex costmap_mutex_;
    // ------------ Callbacks
    void costmap_update(nav2_msgs::msg::Costmap::SharedPtr msg);
    void contours_update(surface_detector_interfaces::msg::DetectionResults::SharedPtr msg);
    // ------------ Functions
    bool findNearestFreeCell(int mx, int my, int& out_x, int& out_y, int radius);
    double signedArea(const std::vector<geometry_msgs::msg::PointStamped>& poly);
    bool generateMarkerMsg(std::vector<Eigen::Vector3d> poses,
                            visualization_msgs::msg::Marker &msg_out,
                            builtin_interfaces::msg::Time stamp,
                            Eigen::Vector3d rgb,
                            std::string frame_id = "map",
                            double z_height = 0.2);
    bool isReachableAstar(nav2_costmap_2d::Costmap2D* costmap, 
                     unsigned int start_mx, unsigned int start_my, 
                     unsigned int goal_mx, unsigned int goal_my);
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
// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "surface_detector_interfaces/msg/detection_results.hpp"
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
    // Approach goal output (forwarded to navigation by the object_detector node)
    std::string goal_topic_name_;
    // TODO subscribe to robot footprint for extracting this param:
    double robot_radius_;
    // Costmap value above which we consider it lethal
    unsigned int max_costmap_val_;
    // Maximum distance tolerated between the robot and the object to grasp
    double dist_threshold_;
    bool enable_window_tangent_orientation_;
    int orientation_window_size_;
    double orientation_object_weight_;
    double orientation_contour_normal_weight_;
    double orientation_face_tolerance_deg_;
    double orientation_perp_tolerance_deg_;
    bool enable_goal_clearance_check_;
    int goal_clearance_radius_cells_;

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
    bool isCellNeighborhoodFree(const nav2_costmap_2d::Costmap2D* costmap,
                                unsigned int mx,
                                unsigned int my,
                                int radius_cells) const;
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

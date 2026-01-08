// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "approach_path_planning/planner.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "Eigen/Core"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>

using namespace approach_path_planning;

planner::planner(const rclcpp::NodeOptions & options) : 
rclcpp_lifecycle::LifecycleNode("approach_path_planning_node", options)
{
    // TODO declare parameters
    base_frame_ = "geometric_unicycle";
    costmap_topic_name_ = "/global_costmap/costmap";
    contours_topic_name_ = "/surface_detector/marker";
    robot_radius_ = 0.4;
    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    state_ = 0;
}

void planner::costmap_update(nav2_msgs::msg::Costmap::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    global_costmap_ = nav2_costmap_2d::Costmap2D(msg->metadata.size_x,
                                                msg->metadata.size_y,
                                                msg->metadata.resolution,
                                                msg->metadata.origin.position.x,
                                                msg->metadata.origin.position.y);
    unsigned char * costmap_data = global_costmap_.getCharMap();
    // To change the data, we do a memory copy of the underlying char vector representing the cell values
    std::memcpy(costmap_data,
                msg->data.data(),
                msg->data.size() * sizeof(unsigned char));
}

void planner::contours_update(surface_detector_interfaces::msg::DetectionResults::SharedPtr result_msg)
{
    auto contours_points = result_msg->surface_contours;
    auto object_pose = result_msg->segmented_object;
    // TODO Finalize sanity checks
    if (contours_points.size() < 3)
    {
        return;
    }
    
    // Logic on how to compute the goal
    // 1) we get the pose of the object and transform it in map frame
    // then the height is also extracted for setting the hands position
    // TODO send hand position coordinates
    // Transform to map frame
    auto transformed_contours = contours_points;
    auto transformed_pose = object_pose;
    try
    {
        for (size_t i = 0; i < transformed_contours.size(); i++)
        {
            transformed_contours[i] = buffer_->transform(contours_points[i], "map");
        }
        transformed_pose = buffer_->transform(object_pose, "map");
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Caught exception while transforming frame: " << e.what());
        return;
    }
    
    // 2) we check the position of the contours and find the closest point closer the object.
    // Then we project it on our custom costmap as maximum rejection cost.
    // TODO do we order the points as check?

    std::size_t L = transformed_contours.size();
    // cycle the segment of the contours to find the closest one to the centroid:
    // we save also the list of closer points per segment of the contour
    // we consider these points to be the optimal ones to reach for grasping the object (since it's closer)
    std::vector<Eigen::Vector2f> approach_point_vec(L);
    //approach_point_vec.resize(L);
    float min_dist;
    Eigen::Vector2f P (transformed_pose.point.x, transformed_pose.point.y);
    Eigen::Vector2f closest_point_to_obj;
    for (size_t i = 0; i < L; i++)
    {
        
        // closing condition
        Eigen::Vector2f AB, A, B;
        if (i == L - 1)
        {
            A (transformed_contours[i].point.x, transformed_contours[i].point.y);
            B (transformed_contours[0].point.x, transformed_contours[0].point.y);
            AB = B - A;

        }
        else
        {
            A (transformed_contours[i].point.x, transformed_contours[i].point.y);
            B (transformed_contours[i+1].point.x, transformed_contours[i+1].point.y);
            AB = B - A;
        }
        float segment_length = AB.squaredNorm();
        // Check if A==B
        if (segment_length == 0.0f)
        {
            float distance_squared = (A - P).squaredNorm();
            approach_point_vec[i] = A;  // save the closest point to P (A=B)
            if (i == 0)
            {
                min_dist = distance_squared;
                closest_point_to_obj = A;
            }
            else if (distance_squared < min_dist)
            {
                min_dist = distance_squared;
                closest_point_to_obj = A;
            }
        }
        else
        {
            float xA = A[0], xB = B[0], xP = P[0];
            float yA = A[1], yB = B[1], yP = P[1];
            // parametric line equation: r(t) = r0 + t * m
            // or y = y0 + m * (x - x0) (between two points P and P0)
            // m is the inclination vector of the line AB
            // m = (yB - yA)/(xB - xA)
            // to have the perpendicular line we invert the sign: -m
            // We obtain the line from P (object pose) as: P - m * t -> y = yP - m * (x - xP)
            // The intersection with AB will give us the closest point
            // y = yA + m * (x - xA)
            // Thus:
            // yP - m * (x - xP) = yA + m * (x - xA)
            // yP - yA = m * (2x + xP - xA)
            // x = ((yP - yA) / m - (xP - xA)) / 2
            // and
            // y = yA + m * (((yP - yA) / m - (xP - xA)) / 2 - xA)
            float m = (yB - yA) / (xB - xA);
            float x = ((yP - yA) / m - (xP - xA)) / 2;
            float y = yA + m * (x - xA);
            Eigen::Vector2f C (x, y);   // closest point to P
            approach_point_vec[i] = C;  // save it
            float distance_squared = (C - P).squaredNorm();
            if (i == 0)
            {
                min_dist = distance_squared;
                closest_point_to_obj = C;
            }
            else if (distance_squared < min_dist)
            {
                min_dist = distance_squared;
                closest_point_to_obj = C;
            }
        }
    }
    
    // 3) Inflate the free space near the contours, where the real costmap is free, as desired goal, 
    // in a gradient descent fashion from the closest point of the contours from the object to grasp

    std::lock_guard<std::mutex> lock(costmap_mutex_);   // DO we use the costmap here?

}

CallbackReturn planner::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring...");
    // TODO: get parameters
    robot_radius_ = 0.3;
    state_ = 0; // 0 standby : 1 navigating
    //Action Client
    nav_callback_group_ = create_callback_group(
                            rclcpp::CallbackGroupType::MutuallyExclusive,
                            false);
    nav_executor_.add_callback_group(nav_callback_group_, get_node_base_interface());
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                            get_node_base_interface(),
                            get_node_graph_interface(),
                            get_node_logging_interface(),
                            get_node_waitables_interface(),
                            "navigate_to_pose", nav_callback_group_);

    nav_feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
                "navigate_to_pose/_action/feedback",
                rclcpp::SystemDefaultsQoS(),
                // Write lambda function to what to do with the feedback
                [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg) {   
                    RCLCPP_INFO( this->get_logger(), "Remaining distance from action feedback: %f", msg->feedback.distance_remaining);
                    {
                        if (nav_goal_handle_!=nullptr)
                        {
                            RCLCPP_INFO(client_node_->get_logger(), "FEEDBACK status: %i", nav_goal_handle_->get_status());
                        }
                    }
                });
    nav_result_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
                "navigate_to_pose/_action/status",
                rclcpp::SystemDefaultsQoS(),
                [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {   
                    // TODO write logic on completion???
                    RCLCPP_INFO( this->get_logger(), "GOAL STATUS: %i", msg->status_list.back().status);
                });
    client_node_ = std::make_shared<rclcpp::Node>("nav_action_client_node_planning_approach");

    // Subs
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(costmap_topic_name_,
                                            rclcpp::SensorDataQoS(),
                                            [this](nav2_msgs::msg::Costmap::SharedPtr msg){
                                                costmap_update(msg);
                                            });
    contours_sub_ = this->create_subscription<surface_detector_interfaces::msg::DetectionResults>(contours_topic_name_,
                                            10,
                                            [this](surface_detector_interfaces::msg::DetectionResults::SharedPtr msg){
                                                contours_update(msg);
                                            });
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Activating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Error");
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_cleanup(const rclcpp_lifecycle::State & state)
{
    nav_client_.reset();
    RCLCPP_INFO(get_logger(), "Cleanup");
    return CallbackReturn::SUCCESS;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;
    rclcpp::NodeOptions options;
    std::cout << "Creating planner node..." << std::endl;
    planner::SharedPtr planner_node = std::make_shared<planner>(options);
    exe.add_node(planner_node->get_node_base_interface());
    std::cout << "Spinning planner node..." << std::endl;
    exe.spin();
    std::cout << "Shutting down planner node..." << std::endl;
    rclcpp::shutdown();

    return 0;
}

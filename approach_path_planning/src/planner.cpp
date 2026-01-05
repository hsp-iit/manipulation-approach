// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "approach_path_planning/planner.hpp"


using namespace approach_path_planning;
using std::placeholders::_1;

planner::planner(const rclcpp::NodeOptions & options) : 
rclcpp_lifecycle::LifecycleNode("approach_path_planning_node", options)
{
    // TODO declare parameters
    _buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(_buffer);
}

CallbackReturn planner::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring...");
    // TODO: get parameters
    _robot_radius = 0.3;
    _state = 0; // 0 standby : 1 navigating
    //Action Client
    _nav_callback_group = create_callback_group(
                            rclcpp::CallbackGroupType::MutuallyExclusive,
                            false);
    _nav_executor.add_callback_group(_nav_callback_group, get_node_base_interface());
    _nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                            get_node_base_interface(),
                            get_node_graph_interface(),
                            get_node_logging_interface(),
                            get_node_waitables_interface(),
                            "navigate_to_pose", _nav_callback_group);

    _nav_feedback_sub = this->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
                "navigate_to_pose/_action/feedback",
                rclcpp::SystemDefaultsQoS(),
                // Write lambda function to what to do with the feedback
                [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg) {   
                    RCLCPP_INFO( this->get_logger(), "Remaining distance from action feedback: %f", msg->feedback.distance_remaining);
                    {
                        if (_nav_goal_handle!=nullptr)
                        {
                            RCLCPP_INFO(_client_node->get_logger(), "FEEDBACK status: %i", _nav_goal_handle->get_status());
                        }
                    }
                });
    _nav_result_sub = this->create_subscription<action_msgs::msg::GoalStatusArray>(
                "navigate_to_pose/_action/status",
                rclcpp::SystemDefaultsQoS(),
                [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {   
                    // TODO write logic on completion???
                    RCLCPP_INFO( this->get_logger(), "GOAL STATUS: %i", msg->status_list.back().status);
                });
    _client_node = std::make_shared<rclcpp::Node>("nav_action_client_node_planning_approach");

    // Subs
    _costmap_sub = this->create_subscription<nav2_msgs::msg::Costmap>("/local_costmap",
                                            10,
                                            std::bind(planner::costmap_update, this, _1));
    _contours_sub = this->create_subscription<visualization_msgs::msg::Marker>("/surface_detector/marker",
                                            10,
                                            std::bind(planner::contours_update, this, _1));
                                            
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
    _nav_client.reset();
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

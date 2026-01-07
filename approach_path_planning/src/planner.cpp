// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "approach_path_planning/planner.hpp"


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
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
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

void planner::contours_update(visualization_msgs::msg::Marker::SharedPtr msg)
{

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
    contours_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(contours_topic_name_,
                                            10,
                                            [this](visualization_msgs::msg::Marker::SharedPtr msg){
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

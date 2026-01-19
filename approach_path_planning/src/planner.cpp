// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "approach_path_planning/planner.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>

using namespace approach_path_planning;

planner::planner(const rclcpp::NodeOptions & options) : 
rclcpp_lifecycle::LifecycleNode("approach_planner_node", options)
{
    // TODO declare parameters
    base_frame_ = "geometric_unicycle";
    costmap_topic_name_ = "/costmap_raw";
    contours_topic_name_ = "/surface_detector/results";
    robot_radius_ = 0.2;
    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    state_ = 0;
    costmap_received_ = false;
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
    costmap_received_ = true;
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
    geometry_msgs::msg::PointStamped robot_pose;
    try
    {
        for (size_t i = 0; i < transformed_contours.size(); i++)
        {
            transformed_contours[i] = buffer_->transform(contours_points[i], "map");
        }
        transformed_pose = buffer_->transform(object_pose, "map");
        auto robot_tf = buffer_->lookupTransform("map", object_pose.header.frame_id, object_pose.header.stamp);
        robot_pose.header = robot_tf.header;
        robot_pose.point.x = robot_tf.transform.translation.x;
        robot_pose.point.y = robot_tf.transform.translation.y;
        robot_pose.point.z = robot_tf.transform.translation.z;

    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Caught exception while transforming frame: " << e.what());
        return;
    }
    
    // 2) we check the position of the contours and find the closest point closer the object.
    // Then we project it on our custom costmap as maximum rejection cost.

    std::size_t L = transformed_contours.size();
    RCLCPP_INFO_STREAM(get_logger(), "Got number of transformed contours: " << L);
    // cycle the segment of the contours to find the closest one to the centroid:
    // we save also the list of closer points per segment of the contour
    // we consider these points to be the optimal ones to reach for grasping the object (since it's closer)
    std::vector<Eigen::Vector2d> approach_point_vec(L);
    Eigen::Vector2d robot_pose_eigen (robot_pose.point.x, robot_pose.point.y);
    std::vector<Eigen::Vector2d> candidate_goals(L);
    double min_dist;
    Eigen::Vector2d P;
    P[0] = transformed_pose.point.x;
    P[1] = transformed_pose.point.y;
    RCLCPP_INFO_STREAM(this->get_logger(), "ROBOT POSE x: " << P[0] << " y: " << P[1]);
    Eigen::Vector2d closest_point_to_obj;
    double area = planner::signedArea(transformed_contours);
    
    for (size_t i = 0; i < L; i++)
    {
        // closing condition
        Eigen::Vector2d AB, A, B;
        // We need to manually initialize each component to avoid internal numerical truncation/optimization
        A[0] = transformed_contours[i].point.x;
        A[1] = transformed_contours[i].point.y;
        if (i == L - 1)
        {
            B[0] = transformed_contours[0].point.x;
            B[1] = transformed_contours[0].point.y;
            //AB = B - A;
            AB[0] = transformed_contours[0].point.x - transformed_contours[i].point.x;
            AB[1] = transformed_contours[0].point.y - transformed_contours[i].point.y;
        }
        else
        {
            B[0] = transformed_contours[i+1].point.x;
            B[1] = transformed_contours[i+1].point.y;
            //AB = B - A;
            AB[0] = B[0] - A[0];
            AB[1] = B[1] - A[1];
        }
        RCLCPP_INFO_STREAM(get_logger(), "transformed_contours " << i << " x: " << transformed_contours[i].point.x << " y: " << transformed_contours[i].point.y);
        RCLCPP_INFO_STREAM(get_logger(), "ABx: " << AB[0] << " ABy: " << AB[1]);
        double segment_length_sq = AB.squaredNorm();
        RCLCPP_INFO_STREAM(get_logger(), "segment_length: " << segment_length_sq);
        // Check if A==B
        if (segment_length_sq <= 1e-12)
        {
            double distance_squared = (A - P).squaredNorm();
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
            Eigen::Vector2d C;   // closest point to P
            double t = (P - A).dot(AB) / segment_length_sq;
            t = std::clamp(t, 0.0, 1.0);
            C = A + t * AB;
            approach_point_vec[i] = C;  // save it
            double distance_squared = (C - P).squaredNorm();
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
        // Check outward normal for the segment AB
        AB.normalize();
        // Two candidate normals
        Eigen::Vector2d normal_left(-AB.y(), AB.x());
        Eigen::Vector2d normal_right( AB.y(), -AB.x());
        Eigen::Vector2d outward_normal = (area >= 0 ? normal_right : normal_left);
        // For keeping things simple: we compute points outside the contours by a fixed offset (based on the robot radius)
        candidate_goals[i] = approach_point_vec[i] + outward_normal.normalized() * robot_radius_;
        RCLCPP_INFO_STREAM(get_logger(), "Candidate goal " << i << " x: " << candidate_goals[i][0] << " y: " << candidate_goals[i][1]);

        RCLCPP_INFO_STREAM(get_logger(), "approach_point_vec " << i << " x: " << approach_point_vec[i][0] << " y: " << approach_point_vec[i][1]);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Found possible approach points: " << approach_point_vec.size());

    // Publish these poses for debug
    visualization_msgs::msg::Marker candidate_goals_msg;
    if (generateMarkerMsg(candidate_goals, 
                        candidate_goals_msg, 
                        transformed_pose.header.stamp, 
                        Eigen::Vector3d(0.0, 0.0, 1.0) , 
                        "map", 
                        transformed_pose.point.z))
    {
        candidate_marker_pub_->publish(candidate_goals_msg);
    }
    
    RCLCPP_INFO_STREAM(get_logger(), "Published Markers");
    // 3) Inflate the free space near the contours, where the real costmap is free, based on the robot radius?
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (! costmap_received_)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Costmap not available! Check that the topic: " << costmap_topic_name_ << " is working fine!");
        return;
    }
    
    // 4) we will see if these points are inside a high cost area, or looking for a nearby cell, and eventually exclude them.
    int costmap_search_radius = 2; // Since resolution is 5cm, we look in a neighbourhood of 10 cm of an occupied candidate goal.
    std::vector<Eigen::Vector2d> filtered_goals;
    std::vector<unsigned char> goal_cells_cost;
    RCLCPP_INFO_STREAM(get_logger(), "1");
    for (const auto & it : candidate_goals)
    {
        unsigned int grid_x, grid_y;
        double x = it[0], y = it[1];
        RCLCPP_INFO_STREAM(get_logger(), "before worldToMap x: " << x << " y: " << y);
        global_costmap_.worldToMap(x, y, grid_x, grid_y);
        RCLCPP_INFO_STREAM(get_logger(), "got cell: " << grid_x << " " << grid_y);
        auto cost = global_costmap_.getCost(grid_x, grid_y);
        RCLCPP_INFO_STREAM(get_logger(), "Got cost " << cost);
        if (cost >= 253)    //254 means lethal, 255 unknown, 253 inflated
        {
            int closest_x, closest_y;
            if (findNearestFreeCell(grid_x, grid_y, closest_x, closest_y, costmap_search_radius))
            {
                RCLCPP_INFO_STREAM(get_logger(), "Found nearest cell " << closest_x << " " << closest_y);
                cost = global_costmap_.getCost(closest_x, closest_y);
                double world_x, world_y;
                global_costmap_.mapToWorld(closest_x, closest_y, world_x, world_y);
                // We save the valid candidates
                filtered_goals.push_back(Eigen::Vector2d(world_x, world_y));
            }
        }
        else
        {
            // Save the original
            filtered_goals.push_back(Eigen::Vector2d(x, y));
        }
        goal_cells_cost.push_back(cost);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Filtered " << goal_cells_cost.size() << " coming from " << L);
    // Check if we found at least one valid candidate
    if (filtered_goals.size() < 1)
    {
        RCLCPP_WARN(get_logger(), "Unable to find valid goal candidates");
        return;
    }
    // Debug publish
    visualization_msgs::msg::Marker filtered_goal_msg;
    if (generateMarkerMsg(filtered_goals, 
                        filtered_goal_msg, 
                        transformed_pose.header.stamp, 
                        Eigen::Vector3d(1.0, 1.0, 0.0), 
                        "map", 
                        transformed_pose.point.z))
    {
        filtered_candidate_marker_pub_->publish(filtered_goal_msg);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Published filtered_goal_msg");
    // 5) Score each valid candidate
    // The poses are ordered from the closest to the robot to the furthest.
    // We need to evaluate the costmap values
    int best_score = 254;
    Eigen::Vector2d best_goal;
    bool found = false;
    for(size_t i = 0; i < filtered_goals.size(); ++i)
    {
        unsigned char score = goal_cells_cost[i];
        if (score < best_score)
        {
            best_score = score;
            best_goal = filtered_goals[i];
            found = true;
        }
    }
    if (!found)
    {
        RCLCPP_ERROR(get_logger(), "No valid goal found!");
        return;
    }
    else
    {
        // Debug publish:
        RCLCPP_INFO_STREAM(get_logger(), "Found goal in X: " << best_goal[0] << " Y: " << best_goal[1]);
    }
    
    
    // 6) Compute orientation (facing the object)
    // The orientation is given by the final X, Y goal cell facing the object pose
}

double planner::signedArea(const std::vector<geometry_msgs::msg::PointStamped>& poly)
{
    double area = 0.0;
    for (size_t i = 0; i < poly.size(); i++)
    {
        const auto& p = poly[i];
        const auto& q = poly[(i + 1) % poly.size()];
        area += (p.point.x * q.point.y - q.point.x * p.point.y);
    }
    return 0.5 * area;
}


bool planner::findNearestFreeCell(int map_x, int map_y, int& out_x, int& out_y, int radius)
{
    unsigned char best_cost = 254;    //Maximum value in costmap (lethal)
    bool found = false;

    for (int dx = -radius; dx <= radius; ++dx) 
    {
        for (int dy = -radius; dy <= radius; ++dy) 
        {
            int near_x = map_x + dx;
            int near_y = map_x + dy;
            // Check bounds
            if(near_x > global_costmap_.getSizeInCellsX() or near_x < 0) continue;
            if(near_y > global_costmap_.getSizeInCellsY() or near_y < 0) continue;

            unsigned char c = global_costmap_.getCost(near_x, near_y);
            // find the minimum cost
            if (c < best_cost) {
              best_cost = c;
              // return value in costmap values (int)
              out_x = near_x;
              out_y = near_y;
              found = true;
            }
        }
    }
    return found;
}

bool planner::generateMarkerMsg(std::vector<Eigen::Vector2d> poses, 
                                visualization_msgs::msg::Marker &msg_out, 
                                builtin_interfaces::msg::Time stamp,
                                Eigen::Vector3d rgb,
                                std::string frame_id,
                                double z_height)
{
    if (poses.size() < 1) return false;
    
    visualization_msgs::msg::Marker candidate_goals_msg;
    candidate_goals_msg.header.frame_id = frame_id;
    candidate_goals_msg.header.stamp = stamp;
    candidate_goals_msg.ns = "";
    candidate_goals_msg.id = 0;
    candidate_goals_msg.pose.orientation.w = 1.0;   
    candidate_goals_msg.action = visualization_msgs::msg::Marker::ADD;
    candidate_goals_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    candidate_goals_msg.scale.x = 0.1;
    candidate_goals_msg.scale.y = 0.1;
    candidate_goals_msg.scale.z = 0.1;
    candidate_goals_msg.color.r = rgb[0];
    candidate_goals_msg.color.g = rgb[1];
    candidate_goals_msg.color.b = rgb[2];
    candidate_goals_msg.color.a = 1.0;
    candidate_goals_msg.points.reserve(poses.size());
    for (const auto & iter : poses)
    {
        geometry_msgs::msg::Point tmp;
        tmp.x = iter[0];
        tmp.y = iter[1];
        tmp.z = z_height;
        candidate_goals_msg.points.push_back(tmp);
    }
    msg_out = candidate_goals_msg;
    return true;
}

CallbackReturn planner::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring...");
    // TODO: get parameters
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
    // Pubs
    candidate_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(std::string(this->get_name()) + "/candidate_goals_marker", 10);
    filtered_candidate_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(std::string(this->get_name()) + "/filtered_candidate_marker", 10);
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(std::string(this->get_name()) + "/goal_pose", 10);
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_activate(const rclcpp_lifecycle::State & state)
{
    candidate_marker_pub_->on_activate();
    filtered_candidate_marker_pub_->on_activate();
    RCLCPP_INFO(get_logger(), "Activating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_deactivate(const rclcpp_lifecycle::State & state)
{
    candidate_marker_pub_->on_deactivate();
    filtered_candidate_marker_pub_->on_deactivate();
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
    candidate_marker_pub_.reset();
    filtered_candidate_marker_pub_.reset();
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

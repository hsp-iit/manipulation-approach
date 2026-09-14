// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Simone Micheletti
#include "approach_path_planning/planner.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <queue>
#include <limits>
#include <cmath>

using namespace approach_path_planning;

planner::planner(const rclcpp::NodeOptions & options) :
rclcpp_lifecycle::LifecycleNode("approach_planner_node", options)
{
    // TODO declare parameters
    this->declare_parameter<std::string>("base_frame", "geometric_unicycle");
    this->declare_parameter<std::string>("costmap_topic_name", "/global_costmap/costmap_raw");
    this->declare_parameter<std::string>("contours_topic_name", "/surface_detector/results");
    this->declare_parameter<std::string>("goal_topic_name", "/approach_planner/goal_pose");
    this->declare_parameter<double>("robot_radius", 0.2);
    this->declare_parameter<double>("goal_edge_margin", 0.05);
    this->declare_parameter<int>("max_costmap_val", 253);
    this->declare_parameter<double>("dist_threshold", 0.6);
    this->declare_parameter<bool>("enable_window_tangent_orientation", true);
    this->declare_parameter<int>("orientation_window_size", 2);
    this->declare_parameter<double>("orientation_object_weight", 0.7);
    this->declare_parameter<double>("orientation_contour_normal_weight", 0.3);
    this->declare_parameter<double>("orientation_face_tolerance_deg", 40.0);
    this->declare_parameter<double>("orientation_perp_tolerance_deg", 40.0);
    this->declare_parameter<bool>("enable_goal_clearance_check", true);
    this->declare_parameter<int>("goal_clearance_radius_cells", -1);

    base_frame_ = "geometric_unicycle";
    costmap_topic_name_ = "/global_costmap/costmap_raw";
    contours_topic_name_ = "/surface_detector/results";
    goal_topic_name_ = "/approach_planner/goal_pose";
    robot_radius_ = 0.2;
    goal_edge_margin_ = 0.05;
    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    costmap_received_ = false;
    max_costmap_val_ = 253;
    dist_threshold_ = 0.6;
    enable_window_tangent_orientation_ = true;
    orientation_window_size_ = 2;
    orientation_object_weight_ = 0.7;
    orientation_contour_normal_weight_ = 0.3;
    orientation_face_tolerance_deg_ = 40.0;
    orientation_perp_tolerance_deg_ = 40.0;
    enable_goal_clearance_check_ = true;
    goal_clearance_radius_cells_ = -1;
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
    auto loop_start = this->get_clock()->now();
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
        auto robot_tf = buffer_->lookupTransform("map", base_frame_, object_pose.header.stamp);
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
    std::vector<Eigen::Vector3d> candidate_goals;    // X, Y, Theta
    candidate_goals.reserve(L);
    Eigen::Vector2d P;
    P[0] = transformed_pose.point.x;
    P[1] = transformed_pose.point.y;
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
        double segment_length_sq = AB.squaredNorm();
        // Skip degenerate segments (A==B), e.g. the closing point of the contour: they have no normal
        if (segment_length_sq <= 1e-12)
        {
            continue;
        }
        // Closest point to P on the segment
        double t = (P - A).dot(AB) / segment_length_sq;
        t = std::clamp(t, 0.0, 1.0);
        approach_point_vec[i] = A + t * AB;  // save it
        // Check outward normal for the segment AB
        AB.normalize();
        // Two candidate normals
        Eigen::Vector2d normal_left(-AB.y(), AB.x());
        Eigen::Vector2d normal_right( AB.y(), -AB.x());
        Eigen::Vector2d outward_normal = (area >= 0 ? normal_right : normal_left);
        // For keeping things simple: we compute points outside the contours by a fixed offset (based on the robot radius)
        Eigen::Vector2d pose_xy = approach_point_vec[i] + outward_normal.normalized() * (robot_radius_ + goal_edge_margin_);

        // Orientation logic: face object while staying mostly perpendicular to local contour.
        double theta = std::atan2(-outward_normal.y(), -outward_normal.x());
        bool valid_orientation = true;

        if (enable_window_tangent_orientation_)
        {
            // 1) Estimate local tangent around i using a symmetric window to make it robust on jagged contours.
            int radius = std::max(1, orientation_window_size_);
            Eigen::Vector2d smooth_tangent(0.0, 0.0);
            for (int r = 1; r <= radius; ++r)
            {
                int prev_idx = (static_cast<int>(i) - r + static_cast<int>(L)) % static_cast<int>(L);
                int next_idx = (static_cast<int>(i) + r) % static_cast<int>(L);
                Eigen::Vector2d prev_pt(transformed_contours[prev_idx].point.x, transformed_contours[prev_idx].point.y);
                Eigen::Vector2d next_pt(transformed_contours[next_idx].point.x, transformed_contours[next_idx].point.y);
                smooth_tangent += (next_pt - prev_pt);
            }
            if (smooth_tangent.norm() < 1e-8)
            {
                smooth_tangent = AB;
            }
            smooth_tangent.normalize();

            // 2) Build inward normal from smoothed tangent.
            Eigen::Vector2d smooth_left(-smooth_tangent.y(), smooth_tangent.x());
            Eigen::Vector2d smooth_right(smooth_tangent.y(), -smooth_tangent.x());
            Eigen::Vector2d smooth_outward = (area >= 0 ? smooth_right : smooth_left);
            Eigen::Vector2d inward_normal = -smooth_outward.normalized();

            // 3) Direction to object from candidate pose.
            Eigen::Vector2d to_object = P - pose_xy;
            if (to_object.norm() < 1e-8)
            {
                valid_orientation = false;
            }
            else
            {
                to_object.normalize();

                // 4) Blend objective: look at object + stay close to inward normal.
                Eigen::Vector2d blended_dir = orientation_object_weight_ * to_object +
                                              orientation_contour_normal_weight_ * inward_normal;
                if (blended_dir.norm() < 1e-8)
                {
                    blended_dir = to_object;
                }
                blended_dir.normalize();
                theta = std::atan2(blended_dir.y(), blended_dir.x());

                // 5) Orientation checks (configurable):
                //    - must face object within tolerance
                //    - must be roughly perpendicular to local contour tangent
                constexpr double kPi = 3.14159265358979323846;
                double face_tol_rad = orientation_face_tolerance_deg_ * kPi / 180.0;
                double perp_tol_rad = orientation_perp_tolerance_deg_ * kPi / 180.0;
                double face_dot = blended_dir.dot(to_object);
                double perp_dot = std::abs(blended_dir.dot(smooth_tangent));

                valid_orientation = (face_dot >= std::cos(face_tol_rad)) &&
                                    (perp_dot <= std::sin(perp_tol_rad));
            }
        }

        if (valid_orientation)
        {
            candidate_goals.push_back(Eigen::Vector3d(pose_xy[0], pose_xy[1], theta));
        }
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

    // 3) Inflate the free space near the contours, where the real costmap is free, based on the robot radius?
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (! costmap_received_)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Costmap not available! Check that the topic: " << costmap_topic_name_ << " is working fine!");
        return;
    }

    // 4) we will see if these points are inside a high cost area, or looking for a nearby cell, and eventually exclude them.
    auto start = this->get_clock()->now();
    int costmap_search_radius = 2; // Since resolution is 5cm, we look in a neighbourhood of 10 cm of an occupied candidate goal.
    std::vector<Eigen::Vector3d> filtered_goals;
    // Candidates passing the local checks, and their costmap cells (reachability is checked later for all of them)
    std::vector<Eigen::Vector3d> local_goals;
    std::vector<unsigned int> local_goal_cells;
    //std::vector<unsigned char> goal_cells_cost;
    // Robot pose in grid coords
    unsigned int r_grid_x, r_grid_y;
    if (!global_costmap_.worldToMap(robot_pose.point.x, robot_pose.point.y, r_grid_x, r_grid_y))
    {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Robot pose is outside costmap bounds. x=" << robot_pose.point.x << " y=" << robot_pose.point.y);
        return;
    }
    for (const auto & it : candidate_goals)
    {
        unsigned int grid_x, grid_y;
        double x = it[0], y = it[1], theta = it[2];
        if (!global_costmap_.worldToMap(x, y, grid_x, grid_y))
        {
            RCLCPP_WARN_STREAM(get_logger(), "Cell outside costmap: x: " << x << " y: " << y);
            continue;
        }
        auto cost = global_costmap_.getCost(grid_x, grid_y);
        if (cost >= max_costmap_val_)    //254 means lethal, 255 unknown, 253 inflated
        {
            int closest_x, closest_y;
            if (findNearestFreeCell(grid_x, grid_y, closest_x, closest_y, costmap_search_radius))
            {
                if (enable_goal_clearance_check_ &&
                    !isCellNeighborhoodFree(&global_costmap_,
                                            static_cast<unsigned int>(closest_x),
                                            static_cast<unsigned int>(closest_y),
                                            goal_clearance_radius_cells_))
                {
                    continue;
                }
                cost = global_costmap_.getCost(closest_x, closest_y);
                double world_x, world_y;
                global_costmap_.mapToWorld(closest_x, closest_y, world_x, world_y);
                // Filter the poses that cannot reach the object to grasp:
                double dx = world_x - P[0];
                double dy = world_y - P[1];
                if (dx*dx + dy*dy > dist_threshold_ * dist_threshold_)
                {
                    continue;
                }
                // We save the valid candidates
                local_goals.push_back(Eigen::Vector3d(world_x, world_y, theta));
                local_goal_cells.push_back(global_costmap_.getIndex(static_cast<unsigned int>(closest_x),
                                                                    static_cast<unsigned int>(closest_y)));
            }
        }
        else
        {
            if (enable_goal_clearance_check_ &&
                !isCellNeighborhoodFree(&global_costmap_, grid_x, grid_y, goal_clearance_radius_cells_))
            {
                continue;
            }
            // Filter the poses that cannot reach the object to grasp:
            double dx = x - P[0];
            double dy = y - P[1];
            if (dx*dx + dy*dy > dist_threshold_ * dist_threshold_)
            {
                continue;
            }
            // Save the original
            local_goals.push_back(Eigen::Vector3d(x, y, theta));
            local_goal_cells.push_back(global_costmap_.getIndex(grid_x, grid_y));
        }
        //goal_cells_cost.push_back(cost); // We could need it in the future for heuristic expansion
    }
    // Check if the robot can reach them: a single search from the robot cell for all the candidates
    std::vector<bool> reachable = computeReachableCells(&global_costmap_, r_grid_x, r_grid_y, local_goal_cells);
    for (size_t i = 0; i < local_goals.size(); ++i)
    {
        if (reachable[local_goal_cells[i]])
        {
            filtered_goals.push_back(local_goals[i]);
        }
    }
    if (filtered_goals.size() < local_goals.size())
    {
        RCLCPP_INFO_STREAM(get_logger(), "Skipped " << local_goals.size() - filtered_goals.size() << " unreachable goal candidates");
    }
    // Check if we found at least one valid candidate
    if (filtered_goals.size() < 1)
    {
        RCLCPP_WARN(get_logger(), "Unable to find valid goal candidates");
        return;
    }
    auto duration = rclcpp::Duration(this->get_clock()->now() - start);
    RCLCPP_INFO_STREAM(this->get_logger(), "Time elapsed for pose candidates eval" << duration.seconds());
    // Debug publish
    visualization_msgs::msg::Marker filtered_goal_msg;
    if (generateMarkerMsg(filtered_goals,
                        filtered_goal_msg,
                        transformed_pose.header.stamp,
                        Eigen::Vector3d(1.0, 1.0, 0.0),
                        "map",
                        transformed_pose.point.z - 0.2))
    {
        filtered_candidate_marker_pub_->publish(filtered_goal_msg);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Published filtered_goal_msg");
    // 5) Score each valid candidate
    // The poses are ordered from the closest to the robot to the furthest.
    // We need to evaluate the costmap values
    Eigen::Vector3d best_goal;
    bool found = false;
    double best_dist_sq = 0;
    for(size_t i = 0; i < filtered_goals.size(); ++i)
    {
        // Chose as best pose the one closer to the robot
        double robot_dist_dx = filtered_goals[i][0] - robot_pose_eigen[0];
        double robot_dist_dy = filtered_goals[i][1] - robot_pose_eigen[1];
        double robot_dist_sq = robot_dist_dx * robot_dist_dx + robot_dist_dy * robot_dist_dy;
        if (!found) // just for the first time
        {
            best_goal = filtered_goals[i];
            found = true;
            best_dist_sq = robot_dist_sq;
        }
        if (robot_dist_sq < best_dist_sq)
        {
            best_goal = filtered_goals[i];
            found = true;
            best_dist_sq = robot_dist_sq;
        }
    }
    auto loop_duration = rclcpp::Duration(this->get_clock()->now() - loop_start);
    RCLCPP_INFO_STREAM(this->get_logger(), "Loop duration: " << loop_duration.seconds());
    if (!found)
    {
        RCLCPP_ERROR(get_logger(), "No valid goal found!");
        return;
    }
    else
    {
        // Debug publish:
        RCLCPP_INFO_STREAM(get_logger(), "Found goal in X: " << best_goal[0] << " Y: " << best_goal[1]);
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = filtered_goal_msg.header.stamp;
        goal_msg.pose.position.x = best_goal[0];
        goal_msg.pose.position.y = best_goal[1];
        goal_msg.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, best_goal[2]);
        goal_msg.pose.orientation.x = q.x();
        goal_msg.pose.orientation.y = q.y();
        goal_msg.pose.orientation.z = q.z();
        goal_msg.pose.orientation.w = q.w();
        goal_pose_pub_->publish(goal_msg);
    }
}

std::vector<bool> planner::computeReachableCells(const nav2_costmap_2d::Costmap2D* costmap,
                                                 unsigned int start_mx, unsigned int start_my,
                                                 const std::vector<unsigned int>& target_cells) const
{
    const unsigned int width = costmap->getSizeInCellsX();
    const unsigned int height = costmap->getSizeInCellsY();
    const size_t map_size = static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<bool> reached(map_size, false);

    // Basic bounds sanity checks
    if (start_mx >= width || start_my >= height)
    {
        return reached;
    }

    // Count the targets, to stop the search once they are all reached
    std::vector<bool> is_target(map_size, false);
    size_t targets_left = 0;
    for (const auto & cell : target_cells)
    {
        if (cell < map_size && !is_target[cell])
        {
            is_target[cell] = true;
            ++targets_left;
        }
    }

    // Near the start, inflated and unknown cells are traversable too (lethal ones are not):
    // the robot must be able to leave the inflation when it's already close to an obstacle
    const double escape_radius = robot_radius_ / costmap->getResolution();
    const double escape_radius_sq = escape_radius * escape_radius;

    const int dx[] = {0, 0, 1, -1};
    const int dy[] = {-1, 1, 0, 0};

    const unsigned int start_id = costmap->getIndex(start_mx, start_my);
    reached[start_id] = true;
    if (is_target[start_id])
    {
        --targets_left;
    }
    std::queue<unsigned int> q;
    q.push(start_id);

    // Breadth first search (4-connected)
    while (!q.empty() && targets_left > 0)
    {
        unsigned int mx, my;
        costmap->indexToCells(q.front(), mx, my);
        q.pop();

        // Expand neighbors
        for (size_t i = 0; i < 4; ++i)
        {
            const int next_x = static_cast<int>(mx) + dx[i];
            const int next_y = static_cast<int>(my) + dy[i];

            if (next_x < 0 || next_y < 0 ||
                next_x >= static_cast<int>(width) || next_y >= static_cast<int>(height))
            {
                continue;
            }

            const unsigned int next_id = costmap->getIndex(static_cast<unsigned int>(next_x),
                                                           static_cast<unsigned int>(next_y));
            if (reached[next_id])
            {
                continue;
            }

            const unsigned int cost = costmap->getCost(next_id);
            bool traversable = cost < max_costmap_val_;
            if (!traversable && cost != nav2_costmap_2d::LETHAL_OBSTACLE)
            {
                const double start_dx = next_x - static_cast<int>(start_mx);
                const double start_dy = next_y - static_cast<int>(start_my);
                traversable = start_dx * start_dx + start_dy * start_dy <= escape_radius_sq;
            }
            if (!traversable)
            {
                continue;
            }

            reached[next_id] = true;
            if (is_target[next_id])
            {
                --targets_left;
            }
            q.push(next_id);
        }
    }

    return reached;
}

bool planner::isCellNeighborhoodFree(const nav2_costmap_2d::Costmap2D* costmap,
                                    unsigned int mx,
                                    unsigned int my,
                                    int radius_cells) const
{
    const unsigned int width = costmap->getSizeInCellsX();
    const unsigned int height = costmap->getSizeInCellsY();
    if (mx >= width || my >= height)
    {
        return false;
    }

    // Circular footprint, radius in cells
    double radius = radius_cells;
    if (radius < 0)
    {
        radius = std::max(1.0, robot_radius_ / costmap->getResolution());
    }
    const int max_offset = static_cast<int>(std::ceil(radius));

    for (int dx = -max_offset; dx <= max_offset; ++dx)
    {
        for (int dy = -max_offset; dy <= max_offset; ++dy)
        {
            if (dx * dx + dy * dy > radius * radius)
            {
                continue;
            }
            const int nx = static_cast<int>(mx) + dx;
            const int ny = static_cast<int>(my) + dy;

            if (nx < 0 || ny < 0 ||
                nx >= static_cast<int>(width) || ny >= static_cast<int>(height))
            {
                return false;
            }

            const unsigned int cost = costmap->getCost(static_cast<unsigned int>(nx),
                                                       static_cast<unsigned int>(ny));
            // Only obstacles and unknown cells: inflated costs already include the (costmap) robot radius,
            // rejecting them here would count the robot size twice
            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
            {
                return false;
            }
        }
    }

    return true;
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
    // Check if costmap present
    if (! costmap_received_)
        return false;

    unsigned char best_cost = max_costmap_val_;    //Maximum value in costmap (lethal) TODO parameterize
    bool found = false;

    for (int dx = -radius; dx <= radius; ++dx)
    {
        for (int dy = -radius; dy <= radius; ++dy)
        {
            int near_x = map_x + dx;
            int near_y = map_y + dy;
            // Check bounds
            if(near_x >= (int)global_costmap_.getSizeInCellsX() or near_x < 0) continue;
            if(near_y >= (int)global_costmap_.getSizeInCellsY() or near_y < 0) continue;

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

bool planner::generateMarkerMsg(std::vector<Eigen::Vector3d> poses,
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
    RCLCPP_INFO(get_logger(), "Configuring... %s", state.label().c_str());
    // TODO: get parameters
    base_frame_ = this->get_parameter("base_frame").as_string();
    costmap_topic_name_ = this->get_parameter("costmap_topic_name").as_string();
    contours_topic_name_ = this->get_parameter("contours_topic_name").as_string();
    goal_topic_name_ = this->get_parameter("goal_topic_name").as_string();
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    goal_edge_margin_ = this->get_parameter("goal_edge_margin").as_double();
    max_costmap_val_ = static_cast<unsigned int>(this->get_parameter("max_costmap_val").as_int());
    dist_threshold_ = this->get_parameter("dist_threshold").as_double();
    enable_window_tangent_orientation_ = this->get_parameter("enable_window_tangent_orientation").as_bool();
    orientation_window_size_ = this->get_parameter("orientation_window_size").as_int();
    orientation_object_weight_ = this->get_parameter("orientation_object_weight").as_double();
    orientation_contour_normal_weight_ = this->get_parameter("orientation_contour_normal_weight").as_double();
    orientation_face_tolerance_deg_ = this->get_parameter("orientation_face_tolerance_deg").as_double();
    orientation_perp_tolerance_deg_ = this->get_parameter("orientation_perp_tolerance_deg").as_double();
    enable_goal_clearance_check_ = this->get_parameter("enable_goal_clearance_check").as_bool();
    goal_clearance_radius_cells_ = this->get_parameter("goal_clearance_radius_cells").as_int();

    if (orientation_window_size_ < 1)
    {
        orientation_window_size_ = 1;
        RCLCPP_WARN(this->get_logger(), "orientation_window_size must be >= 1. Clamping to 1.");
    }
    if (goal_clearance_radius_cells_ == 0)
    {
        goal_clearance_radius_cells_ = 1;
        RCLCPP_WARN(this->get_logger(), "goal_clearance_radius_cells cannot be 0. Clamping to 1.");
    }

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
    // The goal is forwarded to navigation by the object_detector node
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_name_, 10);
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_activate(const rclcpp_lifecycle::State & state)
{
    candidate_marker_pub_->on_activate();
    filtered_candidate_marker_pub_->on_activate();
    goal_pose_pub_->on_activate();
    RCLCPP_INFO(get_logger(), "Activating %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_deactivate(const rclcpp_lifecycle::State & state)
{
    candidate_marker_pub_->on_deactivate();
    filtered_candidate_marker_pub_->on_deactivate();
    goal_pose_pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "Deactivating: %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Error state: %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn planner::on_cleanup(const rclcpp_lifecycle::State & state)
{
    candidate_marker_pub_.reset();
    filtered_candidate_marker_pub_.reset();
    goal_pose_pub_.reset();
    RCLCPP_INFO(get_logger(), "Cleanup: %s", state.label().c_str());
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

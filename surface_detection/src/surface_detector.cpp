// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Vignesh Sushrutha Raghavan
#include "surface_detector/surface_detector.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/angles.hpp>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/distances.h>
#include <pcl/common/common.h>


#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using namespace std::chrono_literals;

SurfaceDetector::SurfaceDetector(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("desk_detector_node", options)
{
    declare_parameter("pointcloud_topic", "/object_detector/segmented_pointcloud");
    declare_parameter("reference_frame", "geometric_unicycle");
    declare_parameter("min_cluster_size", 100);
    declare_parameter("cluster_tolerance", 0.02);
    declare_parameter("height_offset", 0.01);
    declare_parameter("ransac_eps", 0.02);
    declare_parameter("ransac_distance_threshold", 0.01);
    declare_parameter("thicken_ransac", true);
    declare_parameter("delta_ransac_height", 0.02);
    declare_parameter("enable_visualization", true);
    m_kd_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>> ();

    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
}

void SurfaceDetector::cloud_callback(surface_detector_interfaces::msg::SegmentedPointcloud::ConstSharedPtr coupled_pc_in)
{
    // Let's separate the two pointclouds:
    sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_in = std::make_shared<sensor_msgs::msg::PointCloud2>(coupled_pc_in->full_pointcloud);
    sensor_msgs::msg::PointCloud2::ConstSharedPtr object_pc = std::make_shared<sensor_msgs::msg::PointCloud2>(coupled_pc_in->segmented_object);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_pre_height_filter (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::msg::PointCloud2 full_cloud, object_cloud;
    geometry_msgs::msg::TransformStamped realsense_to_foot_tf, realsense_frame_to_XYZ;
    // Transform form camera frame to robot ground frame (reference frame)
    if (m_tf_buffer_in->canTransform(
                pc_in->header.frame_id,
                m_reference_frame,
                pc_in->header.stamp
                ))
    {
        try
        {
            full_cloud = m_tf_buffer_in->transform(*pc_in, m_reference_frame);
            object_cloud = m_tf_buffer_in->transform(*object_pc, m_reference_frame);
        }
        catch(const std::exception& e)
        {
           RCLCPP_WARN(this->get_logger(), "Transform exception: %s \n", e.what());
           return;
        }
    }
    else
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Cannot transfor: " << m_reference_frame << " to: " << pc_in->header.frame_id);
        return;
    }
    auto start = this->get_clock()->now();
    // Convert from ros2
    pcl::fromROSMsg(full_cloud, *in_cloud_pre_height_filter);
    pcl::fromROSMsg(object_cloud, *object_pcl_cloud);
    // Sanity check
    if (object_pcl_cloud->points.size()==0 || in_cloud_pre_height_filter->points.size()==0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Empty pointcloud received. object_pcl_cloud: " << object_pcl_cloud->points.size() << " in_cloud_pre_height_filter: " << in_cloud_pre_height_filter->points.size());
        return;
    }
    
    // Find height extremes and avg of the object (in reference frame):
    Eigen::Vector4f object_centre = Eigen::Vector4f::Zero();
    pcl::PointXYZ min_pt, max_pt;
    if (object_pcl_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Object cloud is empty, skipping.");
        return;
    }
    pcl::compute3DCentroid(*object_pcl_cloud, object_centre);
    pcl::getMinMax3D(*object_pcl_cloud, min_pt, max_pt);
    double object_min_h = min_pt.z;
    double object_max_h = max_pt.z;
    double object_avg_h = object_centre[2];

    RCLCPP_INFO_STREAM(this->get_logger(), "Object height values: avg Z: " << object_avg_h << " max Z: " << object_max_h << " min Z: " << object_min_h);
    // Create return msg for object pose
    geometry_msgs::msg::PointStamped object_pose_msg;
    object_pose_msg.header.frame_id = m_reference_frame;
    object_pose_msg.header.stamp = pc_in->header.stamp;
    object_pose_msg.point.z = object_max_h; // TODO: check whether to use max H or avg H
    object_pose_msg.point.x = object_centre[0];
    object_pose_msg.point.y = object_centre[1];
    // Filter height of the plane underneath the object
    m_pass.setInputCloud(in_cloud_pre_height_filter);
    double delta = std::abs(object_max_h - object_min_h) / 2;
    double min_limit = object_avg_h - delta - m_height_offset;
    if (min_limit >= object_avg_h) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "PassThrough limits inverted. min: " << min_limit << " max: " << object_avg_h);
        return;
    }
    m_pass.setFilterLimits(min_limit, object_avg_h);
    m_pass.filter(*in_cloud_filtered);
    if (in_cloud_filtered->points.size() == 0)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Obtained an empty pc after heigh filtering with extremes: max: " << object_min_h << " min: " << object_min_h - 0.2);
        return;
    }
    // Voxel grid filtering (downsampling)
    m_grid.setInputCloud(in_cloud_filtered);
    m_grid.filter(*in_cloud_filtered);
    // RANSAC
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    m_seg.setInputCloud(in_cloud_filtered);
    m_seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        RCLCPP_ERROR(get_logger(), "Could not estimate a planar model for the given cloud.");
        return;
    }

    // Extract plane points
    sensor_msgs::msg::PointCloud2 plane_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZ>);
    m_extract.setInputCloud(in_cloud_filtered);
    m_extract.setIndices(inliers);
    m_extract.filter(*plane_points);
    pcl::toROSMsg(*plane_points, plane_cloud);
    plane_cloud.header.stamp = full_cloud.header.stamp;
    plane_cloud.header.frame_id = full_cloud.header.frame_id;
    // Find plane avg height
    double plane_height;
    Eigen::Vector4f plane_centre = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*plane_points, plane_centre);
    plane_height = plane_centre[2];
    if (!m_thicken_ransac)
    {
        if (m_enable_vis) m_plane_pub->publish(plane_cloud);
    }
    else
    {
        // Filter full pc based on avg height:
        m_height_pass.setInputCloud(in_cloud_pre_height_filter);
        m_height_pass.setFilterLimits(plane_height - m_delta_ransac_height , plane_height + m_delta_ransac_height);
        m_height_pass.filter(*plane_points);
        if (plane_points->points.size() == 0)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Obtained an empty pc after heigh filtering with extremes: max: " << plane_height + m_delta_ransac_height << " min: " << plane_height - m_delta_ransac_height);
            return;
        }
        if (m_enable_vis)
        {
            pcl::toROSMsg(*plane_points, plane_cloud);
            plane_cloud.header.stamp = full_cloud.header.stamp;
            plane_cloud.header.frame_id = full_cloud.header.frame_id;
            m_plane_pub->publish(plane_cloud);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Clustering plane points..");
    // Create a cluster and find the one closer to the given object pc
    m_kd_tree->setInputCloud(plane_points);
    std::vector<pcl::PointIndices> cluster_ids;
    m_cluster.setInputCloud(plane_points);
    m_cluster.extract(cluster_ids);
    //Sanity Check
    if(cluster_ids.size() <=1)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to cluster. Got only number of ids: " << cluster_ids.size());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Iterating clusters..");
    // Convert clusters: find the one closer to the object
    auto min_dist = std::numeric_limits<double>::infinity();
    pcl::Indices closer_cluster_ids;
    for (const auto& cluster : cluster_ids)
    {
        Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
        pcl::compute3DCentroid(*plane_points, cluster.indices, centroid);
        // Scoring closer clusters based on eucledian distance
        double dist = (centroid.head<3>() - object_centre.head<3>()).norm();
        if(dist < min_dist)
        {
            min_dist = dist;
            closer_cluster_ids = cluster.indices;
        }
    }
    // Convert to PC from ids
    pcl::PointCloud<pcl::PointXYZ>::Ptr closer_cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(*plane_points, closer_cluster_ids, *closer_cluster);

    // Check if cloud is empty
    if (closer_cluster->empty())
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to find a cluster");
        return;
    }

    // To ensure that we don't achieve a degenerate concave hull we must ensure perfect planarity
    // We project the points on the XY plane -> we set a fixed Z and set points Z to this value
    for (auto & p: closer_cluster->points)
    {
        p.z = plane_height;
    }
    if (m_enable_vis)
    {
        sensor_msgs::msg::PointCloud2 debug_msg;
        pcl::toROSMsg(*closer_cluster, debug_msg);
        debug_msg.header.frame_id = full_cloud.header.frame_id;
        debug_msg.header.stamp = full_cloud.header.stamp;
        m_closer_cluster_pub->publish(debug_msg);
    }
    
    // Eliminate interior points to avoid degenerates hulls
    // TODO (now it's working fine, maybe we can avoid this passage)
    RCLCPP_INFO(this->get_logger(), "Computing chull..");
    // Create a Concave Hull representation of the projected inliers of the cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    std::vector<pcl::Vertices> polygons;
    if (closer_cluster->size() > 0)
    {
        concave_hull.setInputCloud(closer_cluster);
    }
    else
    {   // Use the whole plane instead
        concave_hull.setInputCloud(plane_points);
    }
    concave_hull.setAlpha(0.2);    // lower alpha means a more refined and tight contour (we don't need)
    concave_hull.reconstruct(*cloud_hull, polygons);
    // Find the outermost polygon, we exclude the holes
    pcl::Vertices biggest_poly;
    double max_area = -1.0;
    if (!polygons.empty())
    {
        for (const auto& poly : polygons)
        {
            double area = computePolygonArea(cloud_hull, poly);
            
            if (area > max_area)
            {
                max_area = area;
                biggest_poly = poly;
            }
        }
    }

    if (max_area <= 0 || biggest_poly.vertices.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No valid surface contour found.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Selected biggest polygon: Area %.3f, Vertices %zu", 
                max_area, biggest_poly.vertices.size());
    
    // TODO publish only in debug
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing markers.. with number of chulls: " << cloud_hull->points.size());
    visualization_msgs::msg::Marker marker_msg = create_chull_marker(cloud_hull, biggest_poly, full_cloud.header, 1);
    if (m_enable_vis) m_marker_pub->publish(marker_msg);
    
    // Publish the custom message for the planner
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing result... ");
    surface_detector_interfaces::msg::DetectionResults result_msg;
    result_msg.segmented_object = object_pose_msg;
    std::vector<geometry_msgs::msg::PointStamped> points_array(marker_msg.points.size());
    geometry_msgs::msg::PointStamped tmp;
    tmp.header = marker_msg.header;
    for (size_t i = 0; i < marker_msg.points.size(); i++)
    {
        tmp.point = marker_msg.points[i];
        points_array[i] = tmp;
    }
    
    result_msg.surface_contours = points_array;
    m_results_pub->publish(result_msg);
    auto duration = rclcpp::Duration(this->get_clock()->now() - start);
    RCLCPP_INFO_STREAM(this->get_logger(), "Loop duration: " << duration.seconds());
}

double SurfaceDetector::computePolygonArea(const pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud, 
                                           const pcl::Vertices& polygon)
{
    if (polygon.vertices.size() < 3) {
        return 0.0;
    }

    double area = 0.0;
    size_t n = polygon.vertices.size();

    for (size_t i = 0; i < n; ++i)
    {
        const auto& p1 = hull_cloud->points[polygon.vertices[i]];
        const auto& p2 = hull_cloud->points[polygon.vertices[(i + 1) % n]]; // Use module to wrap around to the first point at the end
        area += (p1.x * p2.y - p2.x * p1.y);
    }

    return std::abs(area) / 2.0;
}

visualization_msgs::msg::Marker SurfaceDetector::create_chull_marker(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>hull_points, 
                                                                    pcl::Vertices polygon, 
                                                                    std_msgs::msg::Header header, 
                                                                    int plane_id = 0)
{
    visualization_msgs::msg::Marker hull_marker;
    hull_marker.header = header;
    hull_marker.id = plane_id;
    hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    hull_marker.action = visualization_msgs::msg::Marker::ADD;
    hull_marker.scale.x = 0.01;
    hull_marker.color.r = 0.0f;
    hull_marker.color.g = 1.0f;
    hull_marker.color.b = 0.0f;
    hull_marker.color.a = 1.0f;

    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = hull_points->points[polygon.vertices[i]].x;
        p.y = hull_points->points[polygon.vertices[i]].y;
        p.z = hull_points->points[polygon.vertices[i]].z;
        hull_marker.points.push_back(p);
    }
    // close the hull
    if (!hull_marker.points.empty())
        hull_marker.points.push_back(hull_marker.points.front());
    
    return hull_marker;
}

CallbackReturn SurfaceDetector::on_configure(const rclcpp_lifecycle::State &)
{
    //Param Init
    m_pointcloud_topic_name = this->get_parameter("pointcloud_topic").as_string();
    m_cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    m_min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    m_height_offset = this->get_parameter("height_offset").as_double();
    m_ransac_eps = this->get_parameter("ransac_eps").as_double();
    m_ransac_distance_threshold = this->get_parameter("ransac_distance_threshold").as_double();
    m_thicken_ransac = this->get_parameter("thicken_ransac").as_bool();
    m_delta_ransac_height = this->get_parameter("delta_ransac_height").as_double();
    m_reference_frame = this->get_parameter("reference_frame").as_string();
    m_enable_vis = this->get_parameter("enable_visualization").as_bool();

    RCLCPP_INFO(this->get_logger(), "Configuring with: pointcloud topic: %s cluster_tolerance: %f & min_cluster_size %i ",
                    m_pointcloud_topic_name.c_str(), m_cluster_tolerance, m_min_cluster_size);
    
    //Subscribers
    m_pc_sub = this->create_subscription<surface_detector_interfaces::msg::SegmentedPointcloud> (
        m_pointcloud_topic_name,
        10,
        std::bind(&SurfaceDetector::cloud_callback, this, _1)
    );
    
    //Publisher
    m_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/surface_detector/marker", 10);
    m_plane_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surface_detector/detected_plane", 10);
    m_results_pub = this->create_publisher<surface_detector_interfaces::msg::DetectionResults>("/surface_detector/results", 10);
    m_closer_cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surface_detector/closer_cluster", 10);
    // Filters
    m_pass.setFilterFieldName("z");
    m_height_pass.setFilterFieldName("z");
    m_grid.setLeafSize(m_voxel_size, m_voxel_size, m_voxel_size);
    m_seg.setOptimizeCoefficients(true);
    m_seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    m_seg.setAxis(Eigen::Vector3f::UnitX());  // Should be Z, but here we are in the camera frame
    m_seg.setEpsAngle(m_ransac_eps);  //0.087 -> 5deg
    m_seg.setMethodType(pcl::SAC_RANSAC);
    m_seg.setMaxIterations(200);
    m_seg.setDistanceThreshold(m_ransac_distance_threshold);
    m_cluster.setClusterTolerance(m_cluster_tolerance);  
    m_cluster.setMinClusterSize(m_min_cluster_size);
    m_cluster.setSearchMethod(m_kd_tree);
    m_extract.setNegative(false);
    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    m_marker_pub->on_activate();
    m_plane_pub->on_activate();
    m_closer_cluster_pub->on_activate();
    m_results_pub->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    //m_horizontal_surfaces_pub->on_deactivate();
    m_marker_pub->on_deactivate();
    m_plane_pub->on_deactivate();
    m_closer_cluster_pub->on_deactivate();
    m_results_pub->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    //m_horizontal_surfaces_pub.reset();
    m_marker_pub.reset();
    m_plane_pub.reset();
    m_closer_cluster_pub.reset();
    m_pc_sub.reset();
    m_results_pub.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::NodeOptions options;
        auto node = std::make_shared<SurfaceDetector>(options);
        executor.add_node(node->get_node_base_interface());
        std::cout << "Starting up node. \n";
        executor.spin();
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    else
    {
        std::cout << "ROS2 not available. Shutting down node. \n";
    }
    
    return 0;
}


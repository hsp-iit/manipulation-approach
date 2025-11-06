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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/distances.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using namespace std::chrono_literals;

SurfaceDetector::SurfaceDetector(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("desk_detector_node", options)
{
    declare_parameter("pointcloud_topic", "/camera/depth/color/points");
    declare_parameter("reference_frame", "geometric_unicycle");
    declare_parameter("min_cluster_size", 100);
    declare_parameter("cluster_tolerance", 0.02);
    declare_parameter("height_offset", 0.1);
    declare_parameter("ransac_eps", 0.02);
    declare_parameter("ransac_distance_threshold", 0.01);

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

    RCLCPP_INFO(this->get_logger(), "TF OK");
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
    pcl::CentroidPoint<pcl::PointXYZ> object_centre;
    pcl::PointXYZ obj_center_xyz;
    double object_min_h = 1000, object_max_h = -1000, object_avg_h = 0;
    for (const auto& point : object_pcl_cloud->points)
    {
        object_centre.add(point);
        if (point.z > object_max_h){object_max_h = point.z;}
        if (point.z < object_min_h){object_min_h = point.z;}
    }
    object_centre.get(obj_center_xyz);
    object_avg_h = obj_center_xyz.z;
    RCLCPP_INFO_STREAM(this->get_logger(), "Object height values: avg Z: " << object_avg_h << " max Z: " << object_max_h << " min Z: " << object_min_h);

    // Filter height of the plane underneath the object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_cloud_pre_height_filter);
    pass.setFilterFieldName("z");
    //pass.setFilterLimits(object_min_h - 0.2, object_min_h); // TODO parameterize
    double delta = std::abs(object_max_h - object_min_h) / 2;
    pass.setFilterLimits(object_avg_h - delta - m_height_offset, object_avg_h);
    pass.filter(*in_cloud_filtered);
    if (in_cloud_filtered->points.size() == 0)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Obtained an empty pc after heigh filtering with extremes: max: " << object_min_h << " min: " << object_min_h - 0.2);
        return;
    }

    // RANSAC
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f::UnitX());  // Should be Z, but here we are in the camera frame
    seg.setEpsAngle(m_ransac_eps);  //0.087 -> 5deg
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(m_ransac_distance_threshold);
    seg.setInputCloud(in_cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        RCLCPP_ERROR(get_logger(), "Could not estimate a planar model for the given cloud.");
        return;
    }

    // Extract plane points
    sensor_msgs::msg::PointCloud2 plane_cloud;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*plane_points);
    pcl::toROSMsg(*plane_points, plane_cloud);
    plane_cloud.header.stamp = full_cloud.header.stamp;
    plane_cloud.header.frame_id = full_cloud.header.frame_id;
    m_plane_pub->publish(plane_cloud);

    RCLCPP_INFO(this->get_logger(), "Clustering plane points..");
    // Create a cluster and find the one closer to the given object pc
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(plane_points);
    std::vector<pcl::PointIndices> cluster_ids;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
    cluster.setClusterTolerance(m_cluster_tolerance);  
    cluster.setMinClusterSize(m_min_cluster_size);
    cluster.setSearchMethod(kd_tree);
    cluster.setInputCloud(plane_points);
    cluster.extract(cluster_ids);
    //Sanity Check
    if(cluster_ids.size() <=1)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to cluster. Got only number of ids: " << cluster_ids.size());
        return;
    }

    // Convert clusters: find the one closer to the object
    double min_dist = 1000;
    pcl::PointCloud<pcl::PointXYZ>::Ptr closer_cluster;
    for (const auto& cluster : cluster_ids)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>); 
        pcl::CentroidPoint<pcl::PointXYZ> cluster_centre;
        // Build pontcloud by iterating cluster indicies
        for (const auto& idx : cluster.indices) {   
            cloud_cluster->push_back((*plane_points)[idx]);
            cluster_centre.add((*plane_points)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();  
        cloud_cluster->height = 1;  
        cloud_cluster->is_dense = true;
        pcl::PointXYZ cluster_center_xyz;
        cluster_centre.get(cluster_center_xyz);
        double dist = pcl::euclideanDistance(cluster_center_xyz, obj_center_xyz);
        if(dist < min_dist)
        {
            min_dist = dist;
            closer_cluster = cloud_cluster;
        } // TODO check == case
        
        RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points.");
    }
    

    RCLCPP_INFO(this->get_logger(), "Computing chull..");
    // Create a Concave Hull representation of the projected inliers of the cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    if (closer_cluster->size()>0)
    {
        concave_hull.setInputCloud (closer_cluster);
    }
    else
    {   // Use the whole plane instead
        concave_hull.setInputCloud (plane_points);
    }
    concave_hull.setAlpha (0.1);
    concave_hull.reconstruct (*cloud_hull);
    // TODO publish only in debug
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing markers.. with number of chulls: " << cloud_hull->points.size());
    visualization_msgs::msg::MarkerArray marker_msg = create_chull_marker(cloud_hull, full_cloud.header, 1);
    // TODO remove
    m_horizontal_surfaces_pub->publish(marker_msg);

    return;
}

visualization_msgs::msg::MarkerArray SurfaceDetector::create_chull_marker(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>hull_points, std_msgs::msg::Header header, int plane_id = 0)
{
    visualization_msgs::msg::MarkerArray marker_array;
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

    for (size_t i = 0; i < hull_points->points.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = hull_points->points[i].x;
        p.y = hull_points->points[i].y;
        p.z = hull_points->points[i].z;
        hull_marker.points.push_back(p);
    }
    // close the hull
    if (!hull_marker.points.empty())
        hull_marker.points.push_back(hull_marker.points.front());
    
    m_marker_pub->publish(hull_marker);
    return marker_array;
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


    RCLCPP_INFO(this->get_logger(), "Configuring with: pointcloud topic: %s cluster_tolerance: %f & min_cluster_size %i ",
                    m_pointcloud_topic_name.c_str(), m_cluster_tolerance, m_min_cluster_size);
    
    //Subscribers
    m_pc_sub = this->create_subscription<surface_detector_interfaces::msg::SegmentedPointcloud> (
        m_pointcloud_topic_name,
        10,
        std::bind(&SurfaceDetector::cloud_callback, this, _1)
    );
    
    //Publisher
    m_horizontal_surfaces_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/surface_detector/horizontal_surfaces", 10);   //TODO use node name to smart naming of the topics
    m_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/surface_detector/marker", 10);
    m_plane_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_plane", 10);

    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    m_horizontal_surfaces_pub->on_activate();
    m_marker_pub->on_activate();
    m_plane_pub->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    m_horizontal_surfaces_pub->on_deactivate();
    m_marker_pub->on_deactivate();
    m_plane_pub->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn SurfaceDetector::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    m_horizontal_surfaces_pub.reset();
    m_marker_pub.reset();
    m_plane_pub.reset();
    m_pc_sub.reset();
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


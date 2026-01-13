// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
// SPDX-License-Identifier: BSD-3-Clause
// Author: Vignesh Sushrutha Raghavan
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <std_msgs/msg/header.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "surface_detector_interfaces/msg/segmented_pointcloud.hpp"
#include "surface_detector_interfaces/msg/detection_results.hpp"
#include <pcl/Vertices.h>


#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <string>

class SurfaceDetector : public rclcpp_lifecycle::LifecycleNode
{
    private:
        // Parameters
        std::string m_pointcloud_topic_name;
        double m_cluster_tolerance = 0.02;
        int m_min_cluster_size = 50;
        std::string m_reference_frame = "geometric_unicycle";
        double m_height_offset = 0.1;
        double m_ransac_eps = 0.02;
        double m_ransac_distance_threshold = 0.01;
        bool m_thicken_ransac = true;
        double m_delta_ransac_height = 0.02;
        bool m_enable_vis = true;    // TODO implement
        // TFs
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
        
        // Publishers
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_closer_cluster_pub;
        rclcpp_lifecycle::LifecyclePublisher<surface_detector_interfaces::msg::DetectionResults>::SharedPtr m_results_pub;
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr m_marker_pub;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_plane_pub;
        // Subscribers
        rclcpp::Subscription<surface_detector_interfaces::msg::SegmentedPointcloud>::SharedPtr m_pc_sub;
        // Functions
        void cloud_callback(surface_detector_interfaces::msg::SegmentedPointcloud::ConstSharedPtr pc_in);
    public:
        SurfaceDetector(const rclcpp::NodeOptions & options);

        visualization_msgs::msg::Marker create_chull_marker(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>hull_points, 
                                                            pcl::Vertices polygon, 
                                                            std_msgs::msg::Header header, 
                                                            int plane_id);

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
        CallbackReturn on_error(const rclcpp_lifecycle::State & state);
};

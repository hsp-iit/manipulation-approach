#include "segment_task_surfaces/segment_closest_desk.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <tf2_eigen/tf2_eigen.hpp>

//TODO: Cite Simone's Plane Detector Code
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using namespace std::chrono_literals;

DeskDetector::DeskDetector(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("desk_detector_node", options)
{
    declare_parameter("compensated_above_floor_pcloud", "above_floor_pcloud");
    declare_parameter("minimum_desk_height", 0.7);
    declare_parameter("maximum_desk_height", 1.5);

    m_tf_buffer_in_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in_);
    m_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void DeskDetector::compensated_cloudCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pc_in, *in_cloud);
    

}


CallbackReturn DeskDetector::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    m_pointcloud_pub->on_activate();
    m_plane_pub->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn DeskDetector::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    m_pointcloud_pub->on_deactivate();
    m_plane_pub->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn DeskDetector::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    m_pointcloud_pub.reset();
    m_plane_pub.reset();
    m_pc_sub.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn DeskDetector::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn DeskDetector::on_error(const rclcpp_lifecycle::State & state)
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
        auto node = std::make_shared<PlaneDetector>(options);
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


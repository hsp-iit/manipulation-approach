#include "segment_task_surfaces/segment_closest_desk.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/angles.hpp>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <tf2_eigen/tf2_eigen.hpp>

//TODO: Cite Simone's Plane Detector Code
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using namespace std::chrono_literals;

DeskDetector::DeskDetector(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("desk_detector_node", options)
{
    declare_parameter("compensated_pointcloud_topic", "/camera/depth/color/points");
    declare_parameter("min_desk_height", 0.7);
    declare_parameter("max_desk_height", 1.5);

    m_tf_buffer_in_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in_);
    m_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void DeskDetector::compensated_cloudCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_pre_filter (new pcl::PointCloud<pcl::PointXYZ>);


    sensor_msgs::msg::PointCloud2 realsense_cloud;
    std::string transform_error;
    geometry_msgs::msg::TransformStamped realsense_to_foot_tf, realsense_frame_to_XYZ;
    if (m_tf_buffer_in_->canTransform(
                pc_in->header.frame_id,
                "l_sole",
                pc_in->header.stamp,
                tf2::durationFromSec(0.02),  
                & transform_error ))
    {
        try
        {
            realsense_cloud = m_tf_buffer_in_->transform(*pc_in, "l_sole", tf2::durationFromSec(0.0));
        }
        catch(const std::exception& e)
        {
           RCLCPP_WARN(this->get_logger(), "Transform exception: %s \n", e.what());
           return;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "GOTT HERE %s ==================================\n",realsense_cloud.header.frame_id.c_str());
    pcl::fromROSMsg(realsense_cloud, *in_cloud_pre_filter);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_cloud_pre_filter);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_desk_height_, max_desk_height_);
    pass.filter(*in_cloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    // Accept surfaces within X degrees of horizontal
    seg.setEpsAngle(pcl::deg2rad(10.0f)); // 10° tolerance
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
    *remaining = *in_cloud;
    int plane_id = 0;
    int iter = 0;

    visualization_msgs::msg::MarkerArray marker_array;

    int max_iter = 1000;
    while(remaining->size() > 1000 && iter < max_iter)
    {
        RCLCPP_INFO(this->get_logger(), "GOTT HERE2222222==================================\n");

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.setInputCloud(remaining);
        seg.segment(*inliers, *coefficients);

        
        if (inliers->indices.empty()) {
            RCLCPP_INFO(this->get_logger(), "No more planes found.");
            break;
        }

        // Normal vector (coefficients[0..2]), plane eq: ax+by+cz+d=0
        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        normal.normalize();

        if (fabs(normal.dot(Eigen::Vector3f::UnitZ())) > 0.9) 
        {

            RCLCPP_INFO(this->get_logger(), "Got something close to horizontal.");
    
            // remove ground points but don’t publish them
            extract.setInputCloud(remaining);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*plane);
                // 3️⃣ If we extracted almost the same cloud (rare edge case)
            if (plane->size() == remaining->size())
            {
                RCLCPP_WARN(this->get_logger(), "Plane extraction did not reduce point cloud — breaking.");
                break;
            }

            remaining.swap(plane);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*plane, centroid);

            // ---- Compute convex hull ----
            pcl::ConvexHull<pcl::PointXYZ> chull;
            pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZ>);
            chull.setInputCloud(plane);
            chull.setDimension(2);
            chull.reconstruct(*hull_points);

            // ---- Create centroid marker ----
            visualization_msgs::msg::Marker centroid_marker;
            centroid_marker.header = realsense_cloud.header;
            centroid_marker.ns = "plane_centroids";
            centroid_marker.id = plane_id;
            centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::msg::Marker::ADD;
            centroid_marker.pose.position.x = centroid[0];
            centroid_marker.pose.position.y = centroid[1];
            centroid_marker.pose.position.z = centroid[2];
            centroid_marker.scale.x = 0.08;
            centroid_marker.scale.y = 0.08;
            centroid_marker.scale.z = 0.08;
            centroid_marker.color.r = 1.0f;
            centroid_marker.color.g = 0.0f;
            centroid_marker.color.b = 0.0f;
            centroid_marker.color.a = 1.0f;
            marker_array.markers.push_back(centroid_marker);

            // ---- Create hull line marker ----
            visualization_msgs::msg::Marker hull_marker;
            hull_marker.header = realsense_cloud.header;
            hull_marker.ns = "plane_hulls";
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
            // close the loop
            if (!hull_marker.points.empty())
                hull_marker.points.push_back(hull_marker.points.front());

            marker_array.markers.push_back(hull_marker);
            iter++;
            plane_id++;
            continue;
    
        }


        // Extract non-horizontal plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*non_horizontal_plane);
        remaining.swap(non_horizontal_plane);
        iter++;

    }

        // Clear old markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.insert(marker_array.markers.begin(), clear_marker);

        horizontal_surfaces_pub_->publish(marker_array);



}

CallbackReturn DeskDetector::on_configure(const rclcpp_lifecycle::State &)
{
    //Param Init
    pointcloud_topic_name_ = this->get_parameter("compensated_pointcloud_topic").as_string();
    min_desk_height_ = this->get_parameter("min_desk_height").as_double();
    max_desk_height_ = this->get_parameter("max_desk_height").as_double();

    RCLCPP_INFO(this->get_logger(), "Configuring with: pointcloud topic: %s minimum desk height: %f & maximum desk height %f ",
                    pointcloud_topic_name_.c_str(), min_desk_height_, max_desk_height_);
    
    //Subscribers
    m_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2> (
        pointcloud_topic_name_,
        10,
        std::bind(&DeskDetector::compensated_cloudCB, this, _1)
    );
    
    //Publisher
    horizontal_surfaces_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/desk_detector/horizontal_surfaces", 10);

    return CallbackReturn::SUCCESS;
}



CallbackReturn DeskDetector::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    horizontal_surfaces_pub_->on_activate();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn DeskDetector::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    horizontal_surfaces_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn DeskDetector::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    horizontal_surfaces_pub_.reset();
    m_pc_sub_.reset();
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
        auto node = std::make_shared<DeskDetector>(options);
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


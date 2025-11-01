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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>   // optional, only if you want to visualize the contour
#include <opencv2/opencv.hpp>  

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_pre_voxelized (new pcl::PointCloud<pcl::PointXYZ>);



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
    
    RCLCPP_INFO(this->get_logger(), "GOTT HERE %s ==================================\n",pc_in->header.frame_id.c_str());
    pcl::fromROSMsg(realsense_cloud, *in_cloud_pre_filter);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_cloud_pre_filter);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_desk_height_, max_desk_height_);
    pass.filter(*in_cloud_pre_voxelized);

    pass.setInputCloud(in_cloud_pre_voxelized);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-8.0, 8.0);
    pass.filter(*in_cloud_pre_voxelized);


    pass.setInputCloud(in_cloud_pre_voxelized);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0,6.0);
    pass.filter(*in_cloud_pre_voxelized);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in_cloud_pre_voxelized);
    vg.setLeafSize(0.01f, 0.01f, 0.01f); // 2 cm voxel grid
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*in_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);  
    ec.setMinClusterSize(500);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud);
    ec.extract(cluster_indices);




    int cluster_id = 0;
    visualization_msgs::msg::MarkerArray marker_array;
    int plane_id = 0;
    int iter = 0;

    for (const auto& indices : cluster_indices) 
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : indices.indices)
            cluster->push_back((*in_cloud)[idx]);

        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        RCLCPP_INFO(this->get_logger(), "Read Cluster number %d", cluster_id);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setSearchMethod(tree);
        ne.setInputCloud(cluster);
        ne.setKSearch(30); // neighborhood for normal estimation
        ne.compute(*normals);


        // 2️⃣ Region growing based on normals + smoothness
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr reg_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(reg_tree);
        reg.setMinClusterSize(100);
        reg.setMaxClusterSize(25000);
        reg.setSearchMethod(reg_tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cluster);
        reg.setInputNormals(normals);

        // Allow small smooth curvature transitions
        reg.setSmoothnessThreshold(pcl::deg2rad(10.0f)); // max angle between normals (10°)
        reg.setCurvatureThreshold(1.0);

        std::vector<pcl::PointIndices> sub_cluster;
        reg.extract(sub_cluster);

        for (const auto& sub : sub_cluster)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : sub.indices)
                subcloud->push_back((*cluster)[idx]);

            // Compute normals again for filtering (small cluster = faster)
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_sub;
            ne_sub.setInputCloud(subcloud);
            ne_sub.setSearchMethod(reg_tree);
            ne_sub.setKSearch(20);
            pcl::PointCloud<pcl::Normal>::Ptr sub_normals(new pcl::PointCloud<pcl::Normal>);
            ne_sub.compute(*sub_normals);

                // 2️⃣ Combine XYZ + normals
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*subcloud, *sub_normals, *cloud_with_normals);

            // 3️⃣ Filter out points whose normals are not horizontal
            pcl::PointCloud<pcl::PointNormal>::Ptr horizontal_points(new pcl::PointCloud<pcl::PointNormal>);
            for (const auto& pt : cloud_with_normals->points)
            {
                Eigen::Vector3f n(pt.normal_x, pt.normal_y, pt.normal_z);
                float cos_angle = n.normalized().dot(Eigen::Vector3f(0.0, 0.0, 1.0)); // z-axis = "up"
                if (std::abs(cos_angle) > std::cos(pcl::deg2rad(15.0f))) {
                    horizontal_points->push_back(pt);
                }
            }      

            // Optional: if too few points, skip this cluster
            if (horizontal_points->size() < 100)
                continue;


            /*pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.05);
            seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
            // Accept surfaces within X degrees of horizontal
            seg.setEpsAngle(pcl::deg2rad(3.0f)); // 10° tolerance
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            seg.setInputCloud(horizontal_points);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) continue;
            

        // Normal vector (coefficients[0..2]), plane eq: ax+by+cz+d=0
            Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            normal.normalize();

            if (fabs(normal.dot(Eigen::Vector3f::UnitZ())) > 0.90) 
            {*/

            RCLCPP_INFO(this->get_logger(), "Got something close to horizontal with size %d.",cluster->size());
    
            // remove ground points but don’t publish them
            /*extract.setInputCloud(cluster);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*plane);*/
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*horizontal_points, *plane);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*plane, centroid);

            // ---- Compute convex hull ----
            /*pcl::ConvexHull<pcl::PointXYZ> chull;
            pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZ>);
            chull.setInputCloud(plane);
            chull.setDimension(2);
            chull.reconstruct(*hull_points);*/

            Eigen::Vector4f min_plane, max_plane;
            pcl::getMinMax3D(*plane,min_plane,max_plane);

            std::vector<cv::Point2f> projected_2d;
            for (const auto& p : plane->points) 
            {
                Eigen::Vector3f pt(p.x, p.y, p.z);
                projected_2d.emplace_back(pt(1)-min_plane(1), pt(0)-min_plane(0));
            }
            
            float scale = 50.0; // pixels per meter (adjust)
            cv::Rect2f bbox = cv::boundingRect(projected_2d);  // compute limits

            cv::Mat img = cv::Mat::zeros(bbox.height * scale + 20, bbox.width * scale + 20, CV_8UC1);

            // Draw projected points
            for (const auto& p : projected_2d) {
                int u = static_cast<int>((p.x - bbox.x) * scale);
                int v = static_cast<int>((p.y - bbox.y) * scale);
                if (u >= 0 && v >= 0 && u < img.cols && v < img.rows)
                    img.at<uchar>(v, u) = 255;
            }

            cv::GaussianBlur(img, img, cv::Size(3,3), 0);

            // Contour extraction
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

            pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZ>);

            for(auto &contour : contours)
            {
                for (auto& pt : contour) {
                    float y_local = pt.x / scale + bbox.x;
                    float x_local = pt.y / scale + bbox.y;

                    Eigen::Vector3f p3 (x_local ,y_local,centroid(2));
                    edge_points->push_back(pcl::PointXYZ(p3.x()+min_plane(0), p3.y()+min_plane(1), p3.z()));
                }
            }



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
            centroid_marker.scale.x = 0.1;
            centroid_marker.scale.y = 0.1;
            centroid_marker.scale.z = 0.1;
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

            for (size_t i = 0; i < edge_points->points.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = edge_points->points[i].x;
                p.y = edge_points->points[i].y;
                p.z = edge_points->points[i].z;
                hull_marker.points.push_back(p);
            }
            // close the loop
            if (!hull_marker.points.empty())
                hull_marker.points.push_back(hull_marker.points.front());

            marker_array.markers.push_back(hull_marker);
            iter++;
            plane_id++;
        }
        cluster_id++;

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


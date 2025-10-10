/**
 * 
 */
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"


class DeskDetector : public rclcpp_lifecycle::LifecycleNode
{
    private:
        
        std::string pointcloud_topic_name_;
        double min_desk_height_ = 0.3;
        double max_desk_height_ = 1.5;

        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster_;

        void compensated_cloudCB(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in);

        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markers_to_edge_pub_;
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr horizontal_surfaces_pub_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc_sub_;
    
    public:

        DeskDetector(const rclcpp::NodeOptions & options);

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
        CallbackReturn on_error(const rclcpp_lifecycle::State & state);




};

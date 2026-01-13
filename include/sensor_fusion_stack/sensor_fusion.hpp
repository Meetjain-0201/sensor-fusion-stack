#ifndef SENSOR_FUSION_HPP
#define SENSOR_FUSION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

struct FusedDetection {
    // 3D information from LiDAR
    pcl::PointXYZ centroid_3d;
    float width, length, height;
    
    // 2D information from camera
    bool has_camera_detection;
    std::string class_name;
    float confidence;
    cv::Rect2f bbox_2d;
    
    // Metadata
    int cluster_size;
    int detection_id;
};

class SensorFusion : public rclcpp::Node {
public:
    SensorFusion();

private:
    void syncCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections_msg
    );
    
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    std::vector<FusedDetection> processLidarClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );
    
    bool projectPointToImage(
        const pcl::PointXYZ& point_3d,
        cv::Point2f& point_2d
    );
    
    void associateDetections(
        std::vector<FusedDetection>& lidar_objects,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& camera_detections
    );
    
    void publishFusedMarkers(
        const std::vector<FusedDetection>& fused_objects,
        const std_msgs::msg::Header& header
    );
    
    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // Synchronizer - FIXED: Complete typedef on one line
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fused_marker_pub_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Camera calibration
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_;
    
    // Parameters
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double association_threshold_;
};

#endif

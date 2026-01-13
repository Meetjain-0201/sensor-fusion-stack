#ifndef LIDAR_CLUSTERING_HPP
#define LIDAR_CLUSTERING_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>

struct DetectedObject {
    pcl::PointXYZ centroid;
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    float width, length, height;
    int cluster_size;
};

class LidarClustering : public rclcpp::Node {
public:
    LidarClustering();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    void preprocessCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out
    );
    
    void removeGround(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out
    );
    
    std::vector<DetectedObject> clusterObjects(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );
    
    void publishMarkers(
        const std::vector<DetectedObject>& objects,
        const std_msgs::msg::Header& header
    );
    
    // ROS2 communication
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    
    // Parameters
    double voxel_leaf_size_;
    double ground_threshold_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double min_height_;
    double max_height_;
};

#endif

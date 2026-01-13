#include "sensor_fusion_stack/lidar_clustering.hpp"

LidarClustering::LidarClustering() : Node("lidar_clustering") {
    
    // Declare parameters
    this->declare_parameter<double>("voxel_leaf_size", 0.05);
    this->declare_parameter<double>("ground_threshold", 0.2);
    this->declare_parameter<double>("cluster_tolerance", 0.5);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<int>("max_cluster_size", 5000);
    this->declare_parameter<double>("min_height", 0.1);
    this->declare_parameter<double>("max_height", 3.0);
    
    // Get parameters
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("ground_threshold", ground_threshold_);
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("min_height", min_height_);
    this->get_parameter("max_height", max_height_);
    
    RCLCPP_INFO(this->get_logger(), "LiDAR Clustering Parameters:");
    RCLCPP_INFO(this->get_logger(), "  Voxel size: %.2f", voxel_leaf_size_);
    RCLCPP_INFO(this->get_logger(), "  Ground threshold: %.2f", ground_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Cluster tolerance: %.2f", cluster_tolerance_);
    RCLCPP_INFO(this->get_logger(), "  Cluster size: %d - %d", min_cluster_size_, max_cluster_size_);
    
    // Subscribers
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/scan/points", 10,
        std::bind(&LidarClustering::cloudCallback, this, std::placeholders::_1)
    );
    
    // Publishers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lidar/clusters", 10
    );
    
    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar/filtered_cloud", 10
    );
    
    RCLCPP_INFO(this->get_logger(), "LiDAR clustering node initialized");
}

void LidarClustering::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    
    // Convert ROS message to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_raw);
    
    if (cloud_raw->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
        return;
    }
    
    // Step 1: Preprocess (downsample)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    preprocessCloud(cloud_raw, cloud_filtered);
    
    // Step 2: Remove ground plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    removeGround(cloud_filtered, cloud_no_ground);
    
    // Publish filtered cloud for visualization
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_no_ground, filtered_msg);
    filtered_msg.header = msg->header;
    filtered_cloud_pub_->publish(filtered_msg);
    
    // Step 3: Cluster objects
    std::vector<DetectedObject> objects = clusterObjects(cloud_no_ground);
    
    // Step 4: Publish visualization
    publishMarkers(objects, msg->header);
    
    RCLCPP_INFO(this->get_logger(), "Detected %zu objects", objects.size());
}

void LidarClustering::preprocessCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out
) {
    // Downsample using voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_in);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*cloud_out);
}

void LidarClustering::removeGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out
) {
    // RANSAC plane segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_threshold_);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    
    // Extract non-ground points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Keep everything except ground
    extract.filter(*cloud_out);
}

std::vector<DetectedObject> LidarClustering::clusterObjects(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) {
    std::vector<DetectedObject> detected_objects;
    
    if (cloud->points.size() == 0) {
        return detected_objects;
    }
    
    // KD-Tree for nearest neighbor search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    // Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    // Process each cluster
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        
        // Get bounding box
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        
        // Filter by height
        float height = max_pt.z - min_pt.z;
        if (height < min_height_ || height > max_height_) {
            continue;
        }
        
        // Calculate centroid
        pcl::PointXYZ centroid;
        centroid.x = (min_pt.x + max_pt.x) / 2.0;
        centroid.y = (min_pt.y + max_pt.y) / 2.0;
        centroid.z = (min_pt.z + max_pt.z) / 2.0;
        
        // Create detected object
        DetectedObject obj;
        obj.centroid = centroid;
        obj.min_point = min_pt;
        obj.max_point = max_pt;
        obj.width = max_pt.x - min_pt.x;
        obj.length = max_pt.y - min_pt.y;
        obj.height = height;
        obj.cluster_size = cluster->points.size();
        
        detected_objects.push_back(obj);
    }
    
    return detected_objects;
}

void LidarClustering::publishMarkers(
    const std::vector<DetectedObject>& objects,
    const std_msgs::msg::Header& header
) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete old markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    int id = 0;
    for (const auto& obj : objects) {
        
        // Bounding box marker
        visualization_msgs::msg::Marker box_marker;
        box_marker.header = header;
        box_marker.ns = "clusters";
        box_marker.id = id++;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.action = visualization_msgs::msg::Marker::ADD;
        
        box_marker.pose.position.x = obj.centroid.x;
        box_marker.pose.position.y = obj.centroid.y;
        box_marker.pose.position.z = obj.centroid.z;
        box_marker.pose.orientation.w = 1.0;
        
        box_marker.scale.x = obj.width;
        box_marker.scale.y = obj.length;
        box_marker.scale.z = obj.height;
        
        box_marker.color.r = 0.0;
        box_marker.color.g = 1.0;
        box_marker.color.b = 0.0;
        box_marker.color.a = 0.5;
        
        box_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        
        marker_array.markers.push_back(box_marker);
        
        // Text marker with size info
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = header;
        text_marker.ns = "cluster_info";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        text_marker.pose.position.x = obj.centroid.x;
        text_marker.pose.position.y = obj.centroid.y;
        text_marker.pose.position.z = obj.max_point.z + 0.2;
        
        text_marker.scale.z = 0.3;
        
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        char text[100];
        snprintf(text, sizeof(text), "%.1fx%.1fx%.1f\n%d pts", 
                 obj.width, obj.length, obj.height, obj.cluster_size);
        text_marker.text = text;
        
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        
        marker_array.markers.push_back(text_marker);
    }
    
    marker_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarClustering>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

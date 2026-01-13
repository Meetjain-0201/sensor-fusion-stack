#include "sensor_fusion_stack/sensor_fusion.hpp"

SensorFusion::SensorFusion() : Node("sensor_fusion"), camera_info_received_(false) {
    
    // Parameters
    this->declare_parameter<double>("cluster_tolerance", 0.5);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<int>("max_cluster_size", 5000);
    this->declare_parameter<double>("association_threshold", 0.3);
    
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("association_threshold", association_threshold_);
    
    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node Parameters:");
    RCLCPP_INFO(this->get_logger(), "  Cluster tolerance: %.2f", cluster_tolerance_);
    RCLCPP_INFO(this->get_logger(), "  Association threshold: %.2f", association_threshold_);
    
    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Camera info subscriber
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10,
        std::bind(&SensorFusion::cameraInfoCallback, this, std::placeholders::_1)
    );
    
    // Synchronized subscribers
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, "/scan/points"
    );
    
    detection_sub_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(
        this, "/camera/detections"
    );
    
    // Synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *cloud_sub_, *detection_sub_
    );
    
    sync_->registerCallback(
        std::bind(&SensorFusion::syncCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    // Publishers
    fused_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fusion/objects", 10
    );
    
    RCLCPP_INFO(this->get_logger(), "Sensor fusion node initialized");
    RCLCPP_INFO(this->get_logger(), "Waiting for camera calibration...");
}

void SensorFusion::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_received_) {
        // Extract camera matrix
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = msg->k[0];  // fx
        camera_matrix_.at<double>(1, 1) = msg->k[4];  // fy
        camera_matrix_.at<double>(0, 2) = msg->k[2];  // cx
        camera_matrix_.at<double>(1, 2) = msg->k[5];  // cy
        
        // Distortion coefficients
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        if (msg->d.size() >= 5) {
            for (int i = 0; i < 5; i++) {
                dist_coeffs_.at<double>(i, 0) = msg->d[i];
            }
        }
        
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera calibration received");
        RCLCPP_INFO(this->get_logger(), "  fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                    camera_matrix_.at<double>(0, 0),
                    camera_matrix_.at<double>(1, 1),
                    camera_matrix_.at<double>(0, 2),
                    camera_matrix_.at<double>(1, 2));
    }
}

void SensorFusion::syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections_msg
) {
    if (!camera_info_received_) {
        return;
    }
    
    // Convert point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    if (cloud->points.empty()) {
        return;
    }
    
    // Process LiDAR clusters
    std::vector<FusedDetection> fused_objects = processLidarClusters(cloud);
    
    // Associate with camera detections
    associateDetections(fused_objects, detections_msg);
    
    // Publish fused markers
    publishFusedMarkers(fused_objects, cloud_msg->header);
    
    RCLCPP_INFO(this->get_logger(), "Fused %zu objects (%zu with camera detections)",
                fused_objects.size(),
                std::count_if(fused_objects.begin(), fused_objects.end(),
                             [](const FusedDetection& obj) { return obj.has_camera_detection; }));
}

std::vector<FusedDetection> SensorFusion::processLidarClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) {
    std::vector<FusedDetection> objects;
    
    // KD-Tree for clustering
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
    
    int id = 0;
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        
        // Get bounding box
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        
        // Calculate centroid
        pcl::PointXYZ centroid;
        centroid.x = (min_pt.x + max_pt.x) / 2.0;
        centroid.y = (min_pt.y + max_pt.y) / 2.0;
        centroid.z = (min_pt.z + max_pt.z) / 2.0;
        
        FusedDetection obj;
        obj.centroid_3d = centroid;
        obj.width = max_pt.x - min_pt.x;
        obj.length = max_pt.y - min_pt.y;
        obj.height = max_pt.z - min_pt.z;
        obj.cluster_size = cluster->points.size();
        obj.detection_id = id++;
        obj.has_camera_detection = false;
        
        objects.push_back(obj);
    }
    
    return objects;
}

bool SensorFusion::projectPointToImage(
    const pcl::PointXYZ& point_3d,
    cv::Point2f& point_2d
) {
    try {
        // Transform from lidar_link to camera_optical_frame
        geometry_msgs::msg::PointStamped point_lidar, point_camera;
        point_lidar.header.frame_id = "lidar_link";
        point_lidar.header.stamp = this->get_clock()->now();
        point_lidar.point.x = point_3d.x;
        point_lidar.point.y = point_3d.y;
        point_lidar.point.z = point_3d.z;
        
        // FIXED: Correct transform call for ROS2 Humble
        point_camera = tf_buffer_->transform(point_lidar, "camera_optical_frame", tf2::durationFromSec(0.1));
        
        // Check if point is in front of camera
        if (point_camera.point.z <= 0) {
            return false;
        }
        
        // Project to image plane
        cv::Mat point_3d_cv = (cv::Mat_<double>(3, 1) << 
            point_camera.point.x,
            point_camera.point.y,
            point_camera.point.z);
        
        cv::Mat point_2d_cv = camera_matrix_ * point_3d_cv;
        
        point_2d.x = point_2d_cv.at<double>(0, 0) / point_2d_cv.at<double>(2, 0);
        point_2d.y = point_2d_cv.at<double>(1, 0) / point_2d_cv.at<double>(2, 0);
        
        // Check if projection is within image bounds
        if (point_2d.x < 0 || point_2d.x >= 640 || point_2d.y < 0 || point_2d.y >= 480) {
            return false;
        }
        
        return true;
        
    } catch (const tf2::TransformException& ex) {
        return false;
    }
}

void SensorFusion::associateDetections(
    std::vector<FusedDetection>& lidar_objects,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& camera_detections
) {
    for (auto& obj : lidar_objects) {
        cv::Point2f projected_point;
        
        if (!projectPointToImage(obj.centroid_3d, projected_point)) {
            continue;
        }
        
        // Find best matching camera detection
        float best_overlap = 0.0;
        int best_detection_idx = -1;
        
        for (size_t i = 0; i < camera_detections->detections.size(); i++) {
            const auto& det = camera_detections->detections[i];
            
            // Get bounding box
            float x1 = det.bbox.center.position.x - det.bbox.size_x / 2.0;
            float y1 = det.bbox.center.position.y - det.bbox.size_y / 2.0;
            float x2 = det.bbox.center.position.x + det.bbox.size_x / 2.0;
            float y2 = det.bbox.center.position.y + det.bbox.size_y / 2.0;
            
            // Check if projected point is inside bbox
            if (projected_point.x >= x1 && projected_point.x <= x2 &&
                projected_point.y >= y1 && projected_point.y <= y2) {
                
                // Calculate overlap score (simple: 1.0 if inside)
                float overlap = 1.0;
                
                if (overlap > best_overlap) {
                    best_overlap = overlap;
                    best_detection_idx = i;
                }
            }
        }
        
        // Associate if overlap exceeds threshold
        if (best_overlap > association_threshold_ && best_detection_idx >= 0) {
            const auto& det = camera_detections->detections[best_detection_idx];
            
            obj.has_camera_detection = true;
            obj.class_name = det.results[0].hypothesis.class_id;
            obj.confidence = det.results[0].hypothesis.score;
            
            obj.bbox_2d.x = det.bbox.center.position.x - det.bbox.size_x / 2.0;
            obj.bbox_2d.y = det.bbox.center.position.y - det.bbox.size_y / 2.0;
            obj.bbox_2d.width = det.bbox.size_x;
            obj.bbox_2d.height = det.bbox.size_y;
        }
    }
}

void SensorFusion::publishFusedMarkers(
    const std::vector<FusedDetection>& fused_objects,
    const std_msgs::msg::Header& header
) {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete old markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    int id = 0;
    for (const auto& obj : fused_objects) {
        
        // Bounding box marker - color based on camera detection
        visualization_msgs::msg::Marker box_marker;
        box_marker.header = header;
        box_marker.ns = "fused_objects";
        box_marker.id = id++;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.action = visualization_msgs::msg::Marker::ADD;
        
        box_marker.pose.position.x = obj.centroid_3d.x;
        box_marker.pose.position.y = obj.centroid_3d.y;
        box_marker.pose.position.z = obj.centroid_3d.z;
        box_marker.pose.orientation.w = 1.0;
        
        box_marker.scale.x = obj.width;
        box_marker.scale.y = obj.length;
        box_marker.scale.z = obj.height;
        
        if (obj.has_camera_detection) {
            // Blue for fused detections
            box_marker.color.r = 0.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 1.0;
            box_marker.color.a = 0.7;
        } else {
            // Yellow for LiDAR-only
            box_marker.color.r = 1.0;
            box_marker.color.g = 1.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 0.5;
        }
        
        box_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(box_marker);
        
        // Text marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = header;
        text_marker.ns = "fused_info";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        text_marker.pose.position.x = obj.centroid_3d.x;
        text_marker.pose.position.y = obj.centroid_3d.y;
        text_marker.pose.position.z = obj.centroid_3d.z + obj.height / 2.0 + 0.3;
        
        text_marker.scale.z = 0.3;
        
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        char text[200];
        if (obj.has_camera_detection) {
            snprintf(text, sizeof(text), "%s (%.0f%%)\n%.1fx%.1fx%.1f",
                     obj.class_name.c_str(),
                     obj.confidence * 100.0,
                     obj.width, obj.length, obj.height);
        } else {
            snprintf(text, sizeof(text), "LiDAR Only\n%.1fx%.1fx%.1f",
                     obj.width, obj.length, obj.height);
        }
        text_marker.text = text;
        
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(text_marker);
    }
    
    fused_marker_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

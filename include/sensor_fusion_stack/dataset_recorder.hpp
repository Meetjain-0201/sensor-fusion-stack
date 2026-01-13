#ifndef DATASET_RECORDER_HPP
#define DATASET_RECORDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>

class DatasetRecorder : public rclcpp::Node {
public:
    DatasetRecorder();

private:
    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg
    );
    
    void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
    
    void saveDataPoint(
        const cv::Mat& image,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int frame_id
    );
    
    bool createDirectories();
    
    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    
    // Synchronizer - all on one line
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Dataset info
    std::string dataset_path_;
    int frame_counter_;
    std::ofstream labels_file_;
    
    // Ground truth storage
    struct ObjectInfo {
        std::string name;
        double x, y, z;
        double vx, vy, vz;
    };
    std::vector<ObjectInfo> current_objects_;
};

#endif

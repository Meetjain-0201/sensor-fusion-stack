#include "sensor_fusion_stack/dataset_recorder.hpp"

DatasetRecorder::DatasetRecorder() : Node("dataset_recorder"), frame_counter_(0) {
    
    // Get parameters
    this->declare_parameter<std::string>("dataset_path", "/home/meet/datasets/sensor_fusion");
    this->get_parameter("dataset_path", dataset_path_);
    
    RCLCPP_INFO(this->get_logger(), "Dataset path: %s", dataset_path_.c_str());
    
    // Create directories
    if (!createDirectories()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create dataset directories");
        return;
    }
    
    // Open labels file
    std::string labels_path = dataset_path_ + "/labels.txt";
    labels_file_.open(labels_path);
    if (!labels_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open labels file");
        return;
    }
    
    // Write header
    labels_file_ << "frame_id,timestamp,object_name,x,y,z,vx,vy,vz\n";
    
    // Create synchronized subscribers
    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/image_raw"
    );
    
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, "/scan/points"
    );
    
    // Synchronizer with 10ms tolerance
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *image_sub_, *cloud_sub_
    );
    
    sync_->registerCallback(
        std::bind(&DatasetRecorder::syncCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    // Subscribe to model states for ground truth
    model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 10,
        std::bind(&DatasetRecorder::modelStatesCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Dataset recorder initialized");
    RCLCPP_INFO(this->get_logger(), "Recording to: %s", dataset_path_.c_str());
}

bool DatasetRecorder::createDirectories() {
    // Create main dataset directory
    mkdir(dataset_path_.c_str(), 0777);
    
    // Create subdirectories
    std::string images_dir = dataset_path_ + "/images";
    std::string clouds_dir = dataset_path_ + "/point_clouds";
    
    mkdir(images_dir.c_str(), 0777);
    mkdir(clouds_dir.c_str(), 0777);
    
    return true;
}

void DatasetRecorder::modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    current_objects_.clear();
    
    for (size_t i = 0; i < msg->name.size(); i++) {
        const std::string& name = msg->name[i];
        
        // Filter for obstacle objects (skip ground plane, robot, sun)
        if (name.find("box") != std::string::npos || 
            name.find("cylinder") != std::string::npos ||
            name.find("sphere") != std::string::npos) {
            
            ObjectInfo obj;
            obj.name = name;
            obj.x = msg->pose[i].position.x;
            obj.y = msg->pose[i].position.y;
            obj.z = msg->pose[i].position.z;
            obj.vx = msg->twist[i].linear.x;
            obj.vy = msg->twist[i].linear.y;
            obj.vz = msg->twist[i].linear.z;
            
            current_objects_.push_back(obj);
        }
    }
}

void DatasetRecorder::syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg
) {
    try {
        // Convert image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
        cv::Mat image = cv_ptr->image;
        
        // Convert point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // Save data
        saveDataPoint(image, cloud, frame_counter_);
        
        // Save ground truth labels
        double timestamp = image_msg->header.stamp.sec + image_msg->header.stamp.nanosec * 1e-9;
        for (const auto& obj : current_objects_) {
            labels_file_ << frame_counter_ << ","
                        << std::fixed << std::setprecision(6) << timestamp << ","
                        << obj.name << ","
                        << obj.x << "," << obj.y << "," << obj.z << ","
                        << obj.vx << "," << obj.vy << "," << obj.vz << "\n";
        }
        labels_file_.flush();
        
        frame_counter_++;
        
        if (frame_counter_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Recorded %d frames", frame_counter_);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in sync callback: %s", e.what());
    }
}

void DatasetRecorder::saveDataPoint(
    const cv::Mat& image,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int frame_id
) {
    // Save image
    std::stringstream img_name;
    img_name << dataset_path_ << "/images/frame_" 
             << std::setfill('0') << std::setw(6) << frame_id << ".png";
    cv::imwrite(img_name.str(), image);
    
    // Save point cloud
    std::stringstream cloud_name;
    cloud_name << dataset_path_ << "/point_clouds/frame_" 
               << std::setfill('0') << std::setw(6) << frame_id << ".pcd";
    pcl::io::savePCDFileBinary(cloud_name.str(), *cloud);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DatasetRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

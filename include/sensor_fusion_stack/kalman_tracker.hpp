#ifndef KALMAN_TRACKER_HPP
#define KALMAN_TRACKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <map>

struct Detection {
    Eigen::Vector3d position;  // x, y, z
    Eigen::Vector3d size;      // width, length, height
    std::string class_name;
    float confidence;
    int id;
};

class KalmanFilter {
public:
    KalmanFilter();
    
    void predict(double dt);
    void update(const Eigen::Vector3d& measurement);
    
    Eigen::VectorXd getState() const { return state_; }
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    double getAge() const { return age_; }
    
private:
    Eigen::VectorXd state_;        // [x, y, z, vx, vy, vz]
    Eigen::MatrixXd covariance_;   // State covariance
    Eigen::MatrixXd F_;            // State transition matrix
    Eigen::MatrixXd H_;            // Measurement matrix
    Eigen::MatrixXd Q_;            // Process noise
    Eigen::MatrixXd R_;            // Measurement noise
    
    double age_;
};

struct Track {
    int id;
    KalmanFilter kf;
    Eigen::Vector3d size;
    std::string class_name;
    float confidence;
    int hits;
    int misses;
    bool active;
    
    Track(int track_id, const Detection& det);
};

class KalmanTracker : public rclcpp::Node {
public:
    KalmanTracker();

private:
    void detectionCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    std::vector<Detection> extractDetections(
        const visualization_msgs::msg::MarkerArray::SharedPtr& markers
    );
    
    void predictTracks(double dt);
    void associateDetections(const std::vector<Detection>& detections);
    void updateTracks(const std::vector<Detection>& detections,
                     const std::vector<std::pair<int, int>>& associations);
    void manageTracks();
    void publishTracks();
    
    double calculateDistance(const Track& track, const Detection& det);
    std::vector<std::pair<int, int>> hungarianAssociation(
        const std::vector<Track>& tracks,
        const std::vector<Detection>& detections
    );
    
    // Subscribers and Publishers
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_pub_;
    
    // Tracking state
    std::vector<Track> tracks_;
    int next_track_id_;
    rclcpp::Time last_update_time_;
    
    // Parameters
    double max_association_distance_;
    int max_misses_;
    int min_hits_;
};

#endif

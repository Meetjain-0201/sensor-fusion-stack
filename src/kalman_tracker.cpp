#include "sensor_fusion_stack/kalman_tracker.hpp"
#include <algorithm>
#include <limits>

// ============ KalmanFilter Implementation ============

KalmanFilter::KalmanFilter() : age_(0.0) {
    // State: [x, y, z, vx, vy, vz]
    state_ = Eigen::VectorXd::Zero(6);
    
    // State covariance
    covariance_ = Eigen::MatrixXd::Identity(6, 6) * 10.0;
    
    // State transition matrix (will be updated with dt)
    F_ = Eigen::MatrixXd::Identity(6, 6);
    
    // Measurement matrix (observe position only)
    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    H_(2, 2) = 1.0;
    
    // Process noise
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    
    // Measurement noise
    R_ = Eigen::MatrixXd::Identity(3, 3) * 0.5;
}

void KalmanFilter::predict(double dt) {
    // Update state transition matrix with time step
    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;
    
    // Predict state
    state_ = F_ * state_;
    
    // Predict covariance
    covariance_ = F_ * covariance_ * F_.transpose() + Q_;
    
    age_ += dt;
}

void KalmanFilter::update(const Eigen::Vector3d& measurement) {
    // Innovation
    Eigen::Vector3d y = measurement - H_ * state_;
    
    // Innovation covariance
    Eigen::Matrix3d S = H_ * covariance_ * H_.transpose() + R_;
    
    // Kalman gain
    Eigen::MatrixXd K = covariance_ * H_.transpose() * S.inverse();
    
    // Update state
    state_ = state_ + K * y;
    
    // Update covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    covariance_ = (I - K * H_) * covariance_;
    
    age_ = 0.0;
}

Eigen::Vector3d KalmanFilter::getPosition() const {
    return Eigen::Vector3d(state_(0), state_(1), state_(2));
}

Eigen::Vector3d KalmanFilter::getVelocity() const {
    return Eigen::Vector3d(state_(3), state_(4), state_(5));
}

// ============ Track Implementation ============

Track::Track(int track_id, const Detection& det) 
    : id(track_id), size(det.size), class_name(det.class_name),
      confidence(det.confidence), hits(1), misses(0), active(false) {
    
    // Initialize Kalman filter state
    Eigen::VectorXd initial_state(6);
    initial_state << det.position(0), det.position(1), det.position(2),
                     0.0, 0.0, 0.0;  // Zero initial velocity
    kf.getState() = initial_state;
}

// ============ KalmanTracker Implementation ============

KalmanTracker::KalmanTracker() 
    : Node("kalman_tracker"), next_track_id_(0) {
    
    // Parameters
    this->declare_parameter<double>("max_association_distance", 2.0);
    this->declare_parameter<int>("max_misses", 5);
    this->declare_parameter<int>("min_hits", 3);
    
    this->get_parameter("max_association_distance", max_association_distance_);
    this->get_parameter("max_misses", max_misses_);
    this->get_parameter("min_hits", min_hits_);
    
    RCLCPP_INFO(this->get_logger(), "Kalman Tracker Parameters:");
    RCLCPP_INFO(this->get_logger(), "  Max association distance: %.2f", max_association_distance_);
    RCLCPP_INFO(this->get_logger(), "  Max misses: %d", max_misses_);
    RCLCPP_INFO(this->get_logger(), "  Min hits: %d", min_hits_);
    
    // Subscriber - listen to fusion output
    detection_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/fusion/objects", 10,
        std::bind(&KalmanTracker::detectionCallback, this, std::placeholders::_1)
    );
    
    // Publisher
    track_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/tracking/objects", 10
    );
    
    last_update_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Kalman tracker initialized");
}

void KalmanTracker::detectionCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg
) {
    // Calculate time delta
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.1;  // Default if time seems invalid
    }
    
    // Extract detections from markers
    std::vector<Detection> detections = extractDetections(msg);
    
    // Predict all tracks forward
    predictTracks(dt);
    
    // Associate detections with tracks
    associateDetections(detections);
    
    // Manage track lifecycle
    manageTracks();
    
    // Publish tracked objects
    publishTracks();
    
    RCLCPP_INFO(this->get_logger(), "Tracking %zu objects (%zu active tracks)",
                detections.size(), 
                std::count_if(tracks_.begin(), tracks_.end(),
                             [](const Track& t) { return t.active; }));
}

std::vector<Detection> KalmanTracker::extractDetections(
    const visualization_msgs::msg::MarkerArray::SharedPtr& markers
) {
    std::vector<Detection> detections;
    
    for (const auto& marker : markers->markers) {
        if (marker.ns == "fused_objects" && marker.action == visualization_msgs::msg::Marker::ADD) {
            Detection det;
            det.position = Eigen::Vector3d(
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            );
            det.size = Eigen::Vector3d(
                marker.scale.x,
                marker.scale.y,
                marker.scale.z
            );
            det.id = marker.id;
            
            detections.push_back(det);
        }
    }
    
    return detections;
}

void KalmanTracker::predictTracks(double dt) {
    for (auto& track : tracks_) {
        track.kf.predict(dt);
    }
}

double KalmanTracker::calculateDistance(const Track& track, const Detection& det) {
    Eigen::Vector3d predicted_pos = track.kf.getPosition();
    return (predicted_pos - det.position).norm();
}

std::vector<std::pair<int, int>> KalmanTracker::hungarianAssociation(
    const std::vector<Track>& tracks,
    const std::vector<Detection>& detections
) {
    // Simple greedy association (not true Hungarian, but works well)
    std::vector<std::pair<int, int>> associations;
    std::vector<bool> track_matched(tracks.size(), false);
    std::vector<bool> det_matched(detections.size(), false);
    
    // Create cost matrix
    for (size_t i = 0; i < detections.size(); i++) {
        double min_dist = std::numeric_limits<double>::max();
        int best_track_idx = -1;
        
        for (size_t j = 0; j < tracks.size(); j++) {
            if (track_matched[j]) continue;
            
            double dist = calculateDistance(tracks[j], detections[i]);
            
            if (dist < min_dist && dist < max_association_distance_) {
                min_dist = dist;
                best_track_idx = j;
            }
        }
        
        if (best_track_idx >= 0) {
            associations.push_back({best_track_idx, i});
            track_matched[best_track_idx] = true;
            det_matched[i] = true;
        }
    }
    
    return associations;
}

void KalmanTracker::associateDetections(const std::vector<Detection>& detections) {
    if (tracks_.empty()) {
        // Create new tracks for all detections
        for (const auto& det : detections) {
            tracks_.emplace_back(next_track_id_++, det);
        }
        return;
    }
    
    // Associate detections with existing tracks
    auto associations = hungarianAssociation(tracks_, detections);
    
    // Update matched tracks
    std::vector<bool> track_updated(tracks_.size(), false);
    std::vector<bool> det_used(detections.size(), false);
    
    for (const auto& [track_idx, det_idx] : associations) {
        tracks_[track_idx].kf.update(detections[det_idx].position);
        tracks_[track_idx].hits++;
        tracks_[track_idx].misses = 0;
        tracks_[track_idx].size = detections[det_idx].size;
        
        if (tracks_[track_idx].hits >= min_hits_) {
            tracks_[track_idx].active = true;
        }
        
        track_updated[track_idx] = true;
        det_used[det_idx] = true;
    }
    
    // Handle unmatched tracks
    for (size_t i = 0; i < tracks_.size(); i++) {
        if (!track_updated[i]) {
            tracks_[i].misses++;
        }
    }
    
    // Create new tracks for unmatched detections
    for (size_t i = 0; i < detections.size(); i++) {
        if (!det_used[i]) {
            tracks_.emplace_back(next_track_id_++, detections[i]);
        }
    }
}

void KalmanTracker::manageTracks() {
    // Remove dead tracks
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                      [this](const Track& t) { return t.misses > max_misses_; }),
        tracks_.end()
    );
}

void KalmanTracker::publishTracks() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete old markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    int marker_id = 0;
    for (const auto& track : tracks_) {
        if (!track.active) continue;
        
        Eigen::Vector3d pos = track.kf.getPosition();
        Eigen::Vector3d vel = track.kf.getVelocity();
        
        // Bounding box
        visualization_msgs::msg::Marker box;
        box.header.frame_id = "odom";
        box.header.stamp = this->now();
        box.ns = "tracked_objects";
        box.id = marker_id++;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;
        
        box.pose.position.x = pos(0);
        box.pose.position.y = pos(1);
        box.pose.position.z = pos(2);
        box.pose.orientation.w = 1.0;
        
        box.scale.x = track.size(0);
        box.scale.y = track.size(1);
        box.scale.z = track.size(2);
        
        // Cyan color for tracked objects
        box.color.r = 0.0;
        box.color.g = 1.0;
        box.color.b = 1.0;
        box.color.a = 0.8;
        
        box.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(box);
        
        // Velocity arrow
        if (vel.norm() > 0.1) {
            visualization_msgs::msg::Marker arrow;
            arrow.header = box.header;
            arrow.ns = "velocities";
            arrow.id = marker_id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            
            geometry_msgs::msg::Point start, end;
            start.x = pos(0);
            start.y = pos(1);
            start.z = pos(2);
            
            end.x = pos(0) + vel(0);
            end.y = pos(1) + vel(1);
            end.z = pos(2) + vel(2);
            
            arrow.points.push_back(start);
            arrow.points.push_back(end);
            
            arrow.scale.x = 0.1;  // Shaft diameter
            arrow.scale.y = 0.2;  // Head diameter
            
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;
            
            arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(arrow);
        }
        
        // Text label
        visualization_msgs::msg::Marker text;
        text.header = box.header;
        text.ns = "track_info";
        text.id = marker_id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        
        text.pose.position.x = pos(0);
        text.pose.position.y = pos(1);
        text.pose.position.z = pos(2) + track.size(2) / 2.0 + 0.5;
        
        text.scale.z = 0.3;
        
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.g = 1.0;
        text.color.a = 1.0;
        
        char label[200];
        snprintf(label, sizeof(label), "ID:%d\nVel:%.1fm/s\n%.1fx%.1fx%.1f",
                 track.id,
                 vel.norm(),
                 track.size(0), track.size(1), track.size(2));
        text.text = label;
        
        text.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(text);
    }
    
    track_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>  
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <deque>
#include <chrono>
#include <thread>  

struct GoodPose {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    double score;
    rclcpp::Time timestamp;
};

class LocalizationScoreWithRecovery : public rclcpp::Node
{
public:
    LocalizationScoreWithRecovery() : Node("localization_score"), 
                                      tf_buffer_(this->get_clock()), 
                                      tf_listener_(tf_buffer_)
    {
        declareParameters();
        loadParameters();
        
        // Subscriptions
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&LocalizationScoreWithRecovery::mapCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LocalizationScoreWithRecovery::scanCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&LocalizationScoreWithRecovery::poseCallback, this, std::placeholders::_1));

        // Publishers
        score_pub_ = this->create_publisher<std_msgs::msg::Float32>("/localization_score", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/localization_status", 10);
        recovery_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

        // Initialize state
        current_score_ = 0.0;
        consecutive_bad_count_ = 0;
        startup_time_ = this->now();
        last_recovery_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        
        RCLCPP_INFO(this->get_logger(), "=== Localization Score with Recovery Started ===");
        RCLCPP_INFO(this->get_logger(), "Recovery thresholds - Critical: %.2f, Good: %.2f", 
                   critical_threshold_, good_threshold_);
        RCLCPP_INFO(this->get_logger(), "Waiting %.1fs for score normalization before recovery checks", 
                   normalization_wait_sec_);
    }

private:
    // TF components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Subscriptions & Publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr score_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr recovery_pose_pub_;

    // Localization Score Parameters
    double error_threshold_;
    int decimation_;
    int occupied_thresh_;

    // Recovery Parameters  
    double critical_threshold_;
    double good_threshold_;
    int consecutive_bad_limit_;
    int max_pose_history_;
    double pose_timeout_sec_;
    double normalization_wait_sec_;

    // State
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
    std::deque<GoodPose> good_poses_;
    std::deque<double> recent_scores_;
    static constexpr size_t MAX_RECENT_SCORES = 5;
    
    double current_score_;
    int consecutive_bad_count_;
    rclcpp::Time startup_time_;
    rclcpp::Time last_recovery_time_;

    void declareParameters() {
        // Localization score parameters
        this->declare_parameter("error_threshold", 0.3);
        this->declare_parameter("sample_decimation", 5);
        this->declare_parameter("occupied_threshold", 50);
        
        // Recovery parameters
        this->declare_parameter("critical_threshold", 0.3);
        this->declare_parameter("good_threshold", 0.7);
        this->declare_parameter("consecutive_bad_limit", 3);
        this->declare_parameter("max_pose_history", 10);
        this->declare_parameter("pose_timeout_sec", 30.0);
        this->declare_parameter("normalization_wait_sec", 5.0);
    }

    void loadParameters() {
        error_threshold_ = this->get_parameter("error_threshold").as_double();
        decimation_ = this->get_parameter("sample_decimation").as_int();
        occupied_thresh_ = this->get_parameter("occupied_threshold").as_int();
        
        critical_threshold_ = this->get_parameter("critical_threshold").as_double();
        good_threshold_ = this->get_parameter("good_threshold").as_double();
        consecutive_bad_limit_ = this->get_parameter("consecutive_bad_limit").as_int();
        max_pose_history_ = this->get_parameter("max_pose_history").as_int();
        pose_timeout_sec_ = this->get_parameter("pose_timeout_sec").as_double();
        normalization_wait_sec_ = this->get_parameter("normalization_wait_sec").as_double();
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = msg;
        RCLCPP_INFO(this->get_logger(), "Map loaded: %dx%d, resolution: %.3fm", 
                   msg->info.width, msg->info.height, msg->info.resolution);
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Store good poses for recovery
        if (current_score_ >= good_threshold_) {
            storeGoodPose(*msg, current_score_);
        }
        cleanOldPoses();
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        current_scan_ = msg;
        
        if (!map_) return;

        // Get robot pose using TF lookup
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", msg->header.frame_id, msg->header.stamp, 
                rclcpp::Duration::from_seconds(0.1));
            
            // Convert to pose
            auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
            pose_msg->header = msg->header;
            pose_msg->header.frame_id = "map";
            pose_msg->pose.pose.position.x = transform.transform.translation.x;
            pose_msg->pose.pose.position.y = transform.transform.translation.y;
            pose_msg->pose.pose.position.z = transform.transform.translation.z;
            pose_msg->pose.pose.orientation = transform.transform.rotation;
            
            // Compute and publish score
            double score = computeLocalizationScore(pose_msg, msg);
            publishScoreAndCheckRecovery(score);
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
        }
    }

    double computeLocalizationScore(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose_msg,
                                   const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg) {
        double robot_x = pose_msg->pose.pose.position.x;
        double robot_y = pose_msg->pose.pose.position.y;
        double robot_yaw = tf2::getYaw(pose_msg->pose.pose.orientation);

        double total_score = 0.0;
        int valid_rays = 0;
        
        for (size_t i = 0; i < scan_msg->ranges.size(); i += decimation_) {
            double actual_range = scan_msg->ranges[i];
            
            if (std::isnan(actual_range) || std::isinf(actual_range) ||
                actual_range < scan_msg->range_min || 
                actual_range > scan_msg->range_max) {
                continue;
            }
            
            double ray_angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double world_angle = robot_yaw + ray_angle;
            
            double expected_range = raycastToObstacle(robot_x, robot_y, world_angle, 
                                                     scan_msg->range_max);
            
            double range_error = std::abs(actual_range - expected_range);
            double ray_score = std::max(0.0, 1.0 - (range_error / error_threshold_));
            
            total_score += ray_score;
            valid_rays++;
        }

        return (valid_rays > 0) ? (total_score / valid_rays) : 0.0;
    }

    void publishScoreAndCheckRecovery(double score) {
        current_score_ = score;
        
        // Apply smoothing
        recent_scores_.push_back(score);
        if (recent_scores_.size() > MAX_RECENT_SCORES) {
            recent_scores_.pop_front();
        }
        
        double smoothed_score = 0.0;
        for (double s : recent_scores_) {
            smoothed_score += s;
        }
        smoothed_score /= recent_scores_.size();
        
        // Publish only the raw score to /localization_score
        std_msgs::msg::Float32 score_msg;
        score_msg.data = static_cast<float>(smoothed_score);
        score_pub_->publish(score_msg);

        // Print score to console
        static int print_counter = 0;
        if (++print_counter % 10 == 0) { // Print every 10th score
            RCLCPP_INFO(this->get_logger(), "Score: %.3f | Good poses: %zu", 
                       smoothed_score, good_poses_.size());
        }

        // Publish status to /localization_status
        std_msgs::msg::String status_msg;
        if (consecutive_bad_count_ == 0) {
            status_msg.data = "Score: " + std::to_string(smoothed_score) + 
                             " | Good poses: " + std::to_string(good_poses_.size()) + 
                             " | Status: HEALTHY";
        } else {
            status_msg.data = "Score: " + std::to_string(smoothed_score) + 
                             " | Good poses: " + std::to_string(good_poses_.size()) + 
                             " | Status: WARNING";
        }
        status_pub_->publish(status_msg);

        // Check for recovery (only after normalization period)
        checkRecoveryConditions(smoothed_score);
    }

    void checkRecoveryConditions(double score) {
        auto now = this->now();
        
        // Wait for normalization after startup
        if ((now - startup_time_).seconds() < normalization_wait_sec_) {
            return;
        }
        
        // Wait for normalization after recovery
        if ((now - last_recovery_time_).seconds() < normalization_wait_sec_) {
            return;
        }
        
        // Check score condition
        if (score < critical_threshold_) {
            consecutive_bad_count_++;
            
            RCLCPP_WARN(this->get_logger(), 
                       "LOW SCORE: %.3f (%d/%d consecutive)", 
                       score, consecutive_bad_count_, consecutive_bad_limit_);
            
            // Publish warning status
            std_msgs::msg::String status_msg;
            status_msg.data = "Score: " + std::to_string(score) + 
                             " | Status: LOW_SCORE (" + std::to_string(consecutive_bad_count_) + 
                             "/" + std::to_string(consecutive_bad_limit_) + " consecutive)";
            status_pub_->publish(status_msg);
            
            if (consecutive_bad_count_ >= consecutive_bad_limit_) {
                attemptRecovery();
            }
        } else {
            if (consecutive_bad_count_ > 0) {
                RCLCPP_INFO(this->get_logger(), "Score recovered: %.3f", score);
                
                // Publish recovery success status
                std_msgs::msg::String status_msg;
                status_msg.data = "Score: " + std::to_string(score) + " | Status: RECOVERED";
                status_pub_->publish(status_msg);
                
                consecutive_bad_count_ = 0;
            }
        }
    }

    void storeGoodPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose, double score) {
        GoodPose good_pose;
        good_pose.pose = pose;
        good_pose.score = score;
        good_pose.timestamp = this->now();
        
        good_poses_.push_back(good_pose);
        
        if (good_poses_.size() > static_cast<size_t>(max_pose_history_)) {
            good_poses_.pop_front();
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Stored good pose (score: %.3f). Total: %zu", 
                    score, good_poses_.size());
    }

    void cleanOldPoses() {
        auto now = this->now();
        auto timeout = rclcpp::Duration::from_seconds(pose_timeout_sec_);
        
        good_poses_.erase(
            std::remove_if(good_poses_.begin(), good_poses_.end(),
                          [&](const GoodPose& pose) {
                              return (now - pose.timestamp) > timeout;
                          }),
            good_poses_.end());
    }

    void attemptRecovery() {
        if (good_poses_.empty()) {
            RCLCPP_ERROR(this->get_logger(), 
                        "RECOVERY FAILED: No good poses available!");
            
            // Publish failed recovery status
            std_msgs::msg::String status_msg;
            status_msg.data = "Score: " + std::to_string(current_score_) + 
                             " | Status: RECOVERY_FAILED - No good poses available";
            status_pub_->publish(status_msg);
            return;
        }

        // Find best pose (highest score)
        auto best_pose_it = std::max_element(good_poses_.begin(), good_poses_.end(),
            [](const GoodPose& a, const GoodPose& b) {
                return a.score < b.score;
            });

        // Prepare recovery pose - use simple timestamp approach
        auto recovery_pose = best_pose_it->pose;
        
        // For initial pose, we can use a simple timestamp strategy:
        // Option 1: Use zero timestamp (let AMCL handle it)
        // Option 2: Use current time
        recovery_pose.header.stamp = rclcpp::Time(0);  // Zero timestamp - simplest approach
        recovery_pose.header.frame_id = "map";
        
        // Alternative: if zero timestamp causes issues, use current time
        // recovery_pose.header.stamp = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Using zero timestamp for initial pose");
        recovery_pose.header.frame_id = "map";
        
        // Increase covariance for recovery - AMCL needs higher uncertainty for relocalization
        std::fill(recovery_pose.pose.covariance.begin(), recovery_pose.pose.covariance.end(), 0.0);
        recovery_pose.pose.covariance[0] = 0.5;   // x variance (increased)
        recovery_pose.pose.covariance[7] = 0.5;   // y variance (increased)  
        recovery_pose.pose.covariance[35] = 0.3;  // yaw variance (increased)
        
        // Add some correlation terms for better AMCL behavior
        recovery_pose.pose.covariance[1] = 0.1;   // xy correlation
        recovery_pose.pose.covariance[6] = 0.1;   // yx correlation
        
        // Wait a brief moment to ensure TF is available
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        
        // Publish recovery pose
        recovery_pose_pub_->publish(recovery_pose);
        
        // Update state
        last_recovery_time_ = this->now();
        consecutive_bad_count_ = 0;
        
        RCLCPP_WARN(this->get_logger(), 
                   "RECOVERY TRIGGERED: Published pose (%.2f, %.2f, %.1f°) with score %.3f", 
                   recovery_pose.pose.pose.position.x,
                   recovery_pose.pose.pose.position.y,
                   tf2::getYaw(recovery_pose.pose.pose.orientation) * 180.0 / M_PI,
                   best_pose_it->score);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Waiting %.1fs for recovery normalization...", normalization_wait_sec_);
        
        // Publish recovery status
        std_msgs::msg::String status_msg;
        status_msg.data = "Score: " + std::to_string(current_score_) + 
                         " | Status: RECOVERY_TRIGGERED | Pose: (" + 
                         std::to_string(recovery_pose.pose.pose.position.x) + ", " +
                         std::to_string(recovery_pose.pose.pose.position.y) + ")";
        status_pub_->publish(status_msg);
    }

    // Ray casting methods
    bool worldToMap(double wx, double wy, int& mx, int& my) const {
        if (!map_) return false;
        
        mx = static_cast<int>((wx - map_->info.origin.position.x) / map_->info.resolution);
        my = static_cast<int>((wy - map_->info.origin.position.y) / map_->info.resolution);
        
        return (mx >= 0 && my >= 0 && mx < static_cast<int>(map_->info.width) && 
                my < static_cast<int>(map_->info.height));
    }

    bool isOccupied(int mx, int my) const {
        if (mx < 0 || my < 0 || mx >= static_cast<int>(map_->info.width) || 
            my >= static_cast<int>(map_->info.height)) {
            return true;
        }
        
        int index = my * map_->info.width + mx;
        return map_->data[index] > occupied_thresh_;
    }

    double raycastToObstacle(double start_x, double start_y, double angle, double max_range) {
        if (!map_) return max_range;

        double resolution = map_->info.resolution;
        double dx = std::cos(angle);
        double dy = std::sin(angle);
        double step_size = resolution * 0.5;
        double current_range = 0.0;
        
        while (current_range < max_range) {
            double test_x = start_x + current_range * dx;
            double test_y = start_y + current_range * dy;
            
            int mx, my;
            if (!worldToMap(test_x, test_y, mx, my)) {
                return current_range;
            }
            
            if (isOccupied(mx, my)) {
                return bresenhamRaycast(start_x, start_y, angle, current_range);
            }
            
            current_range += step_size;
        }
        
        return max_range;
    }

    double bresenhamRaycast(double start_x, double start_y, double angle, double approx_range) {
        int start_mx, start_my;
        if (!worldToMap(start_x, start_y, start_mx, start_my)) return approx_range;
        
        double end_x = start_x + (approx_range + map_->info.resolution) * std::cos(angle);
        double end_y = start_y + (approx_range + map_->info.resolution) * std::sin(angle);
        
        int end_mx, end_my;
        if (!worldToMap(end_x, end_y, end_mx, end_my)) {
            end_mx = std::max(0, std::min(static_cast<int>(map_->info.width) - 1, end_mx));
            end_my = std::max(0, std::min(static_cast<int>(map_->info.height) - 1, end_my));
        }
        
        int dx = std::abs(end_mx - start_mx);
        int dy = std::abs(end_my - start_my);
        int x_step = (start_mx < end_mx) ? 1 : -1;
        int y_step = (start_my < end_my) ? 1 : -1;
        int error = dx - dy;
        
        int x = start_mx;
        int y = start_my;
        
        while (true) {
            if (isOccupied(x, y)) {
                double world_x = map_->info.origin.position.x + x * map_->info.resolution;
                double world_y = map_->info.origin.position.y + y * map_->info.resolution;
                
                double distance = std::sqrt(std::pow(world_x - start_x, 2) + std::pow(world_y - start_y, 2));
                return distance;
            }
            
            if (x == end_mx && y == end_my) break;
            
            int error2 = 2 * error;
            if (error2 > -dy) {
                error -= dy;
                x += x_step;
            }
            if (error2 < dx) {
                error += dx;
                y += y_step;
            }
        }
        
        return approx_range;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationScoreWithRecovery>());
    rclcpp::shutdown();
    return 0;
}
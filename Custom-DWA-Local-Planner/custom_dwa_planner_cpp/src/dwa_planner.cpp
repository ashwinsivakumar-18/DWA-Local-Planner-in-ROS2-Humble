#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <memory>

namespace angles {
double shortest_angular_distance(double from, double to) {
    double diff = to - from;
    while (diff > M_PI) diff -= 2.0*M_PI;
    while (diff < -M_PI) diff += 2.0*M_PI;
    return diff;
}
}

using namespace std::chrono_literals;

class DWALocalPlanner : public rclcpp::Node {
public:
    struct Trajectory {
        std::vector<std::pair<double, double>> path;
        double speed;
        double turn_rate;
        double score;
    };

    DWALocalPlanner() : Node("dwa_planner") {
        // Parameters optimized for TurtleBot Burger
        this->declare_parameter("max_speed", 0.15);        // Reduced max speed for better control
        this->declare_parameter("min_speed", 0.05);
        this->declare_parameter("max_turn_rate", 2.84);    // ~163 deg/s (TurtleBot Burger max)
        this->declare_parameter("step_time", 0.2);         // Simulation step time
        this->declare_parameter("safety_margin", 0.15);    // Reduced safety margin
        this->declare_parameter("goal_tolerance", 0.35);   // Larger goal tolerance area
        this->declare_parameter("sim_samples", 15);        // Reduced for performance
        this->declare_parameter("eval_samples", 80);       // Reduced for performance
        this->declare_parameter("robot_radius", 0.12);     // Smaller radius for Burger
        this->declare_parameter("goal_x", 2.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("slow_down_distance", 0.75); // Start slowing down earlier
        this->declare_parameter("max_acceleration", 0.1);   // Gentle acceleration
        this->declare_parameter("max_deceleration", 0.2);   // Stronger deceleration
        this->declare_parameter("max_turn_acceleration", 1.0); // Faster turning response
        this->declare_parameter("goal_weight", 3.0);        // Stronger goal attraction
        this->declare_parameter("heading_weight", 1.5);     // Balanced heading importance
        this->declare_parameter("obstacle_weight", 3.0);    // Strong obstacle avoidance
        this->declare_parameter("stopping_buffer", 0.1);    // Additional stopping buffer

        // Load parameters
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_turn_ = this->get_parameter("max_turn_rate").as_double();
        step_time_ = this->get_parameter("step_time").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        sim_samples_ = this->get_parameter("sim_samples").as_int();
        eval_samples_ = this->get_parameter("eval_samples").as_int();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        slow_down_distance_ = this->get_parameter("slow_down_distance").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        max_deceleration_ = this->get_parameter("max_deceleration").as_double();
        max_turn_acceleration_ = this->get_parameter("max_turn_acceleration").as_double();
        goal_weight_ = this->get_parameter("goal_weight").as_double();
        heading_weight_ = this->get_parameter("heading_weight").as_double();
        obstacle_weight_ = this->get_parameter("obstacle_weight").as_double();
        stopping_buffer_ = this->get_parameter("stopping_buffer").as_double();

        // Subscribers and Publishers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_data_ = msg;
            });
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_data_ = msg;
            });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

        // Initialize with parameterized goal
        goal_x_ = this->get_parameter("goal_x").as_double();
        goal_y_ = this->get_parameter("goal_y").as_double();
        RCLCPP_INFO(this->get_logger(), "Initial goal set to (%.2f, %.2f)", goal_x_, goal_y_);

        // Initialize random number generator
        gen_.seed(rd_());
        
        // Initialize with zero velocity
        current_speed_ = 0.0;
        current_turn_rate_ = 0.0;
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(step_time_),
            [this]() { control_loop(); });
    }

private:
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data storage
    nav_msgs::msg::Odometry::SharedPtr odom_data_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_data_;

    // Configuration parameters (TurtleBot Burger optimized)
    double max_speed_{0.22};
    double min_speed_{0.05};
    double max_turn_{2.84};
    double step_time_{0.2};
    double safety_margin_{0.15};
    double goal_tolerance_{0.25};
    double robot_radius_{0.12};
    double slow_down_distance_{0.75};
    double max_acceleration_{0.1};
    double max_deceleration_{0.2};
    double max_turn_acceleration_{1.0};
    double goal_weight_{2.0};
    double heading_weight_{1.5};
    double obstacle_weight_{3.0};
    double stopping_buffer_{0.1};
    int sim_samples_{15};
    int eval_samples_{80};

    // Current state
    double current_speed_{0.0};
    double current_turn_rate_{0.0};

    // Goal state
    double goal_x_{2.0};
    double goal_y_{0.0};
    bool goal_reached_{false};

    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;

    double get_yaw_from_odom() {
        if (!odom_data_) return 0.0;
        
        tf2::Quaternion q(
            odom_data_->pose.pose.orientation.x,
            odom_data_->pose.pose.orientation.y,
            odom_data_->pose.pose.orientation.z,
            odom_data_->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    Trajectory simulate_trajectory(double speed, double turn_rate) {
        Trajectory traj;
        traj.speed = speed;
        traj.turn_rate = turn_rate;
        
        if (!odom_data_) return traj;

        double x = odom_data_->pose.pose.position.x;
        double y = odom_data_->pose.pose.position.y;
        double yaw = get_yaw_from_odom();

        for (int i = 0; i < sim_samples_; ++i) {
            yaw += turn_rate * step_time_;
            x += speed * cos(yaw) * step_time_;
            y += speed * sin(yaw) * step_time_;
            traj.path.emplace_back(x, y);
        }
        
        return traj;
    }

    double check_collision(const std::vector<std::pair<double, double>>& path) {
        if (!scan_data_ || !odom_data_) return -INFINITY;

        double collision_penalty = 0.0;
        double min_distance = INFINITY;
        const double total_margin = safety_margin_ + robot_radius_;

        for (const auto& point : path) {
            double dx = point.first - odom_data_->pose.pose.position.x;
            double dy = point.second - odom_data_->pose.pose.position.y;
            double dist = hypot(dx, dy);
            double angle = atan2(dy, dx) - get_yaw_from_odom();
            angle = angles::shortest_angular_distance(0.0, angle);

            if (abs(angle) > scan_data_->angle_max) continue;

            size_t idx = static_cast<size_t>((angle - scan_data_->angle_min) / scan_data_->angle_increment);
            idx = std::clamp(idx, size_t(0), scan_data_->ranges.size() - 1);

            double range = static_cast<double>(scan_data_->ranges[idx]);
            if (std::isfinite(range) && range < dist + total_margin) {
                double penetration = (dist + total_margin) - range;
                collision_penalty += -obstacle_weight_ * 1000.0 * (1.0 + penetration);
                min_distance = std::min(min_distance, range);
            }
        }

        if (min_distance < total_margin * 1.5) {
            collision_penalty += -obstacle_weight_ * 5000.0 * (1.5 - min_distance/total_margin);
        }

        return collision_penalty;
    }

    double evaluate_trajectory(const Trajectory& traj) {
        if (!odom_data_ || !scan_data_) return -INFINITY;

        double current_x = odom_data_->pose.pose.position.x;
        double current_y = odom_data_->pose.pose.position.y;
        double yaw = get_yaw_from_odom();

        // Goal distance scoring
        double goal_dist = hypot(traj.path.back().first - goal_x_, 
                               traj.path.back().second - goal_y_);
        double goal_score = -goal_weight_ * 20.0 * goal_dist;

        // Heading scoring
        double target_angle = atan2(goal_y_ - current_y, goal_x_ - current_x);
        double angle_diff = angles::shortest_angular_distance(yaw, target_angle);
        double heading_score = -heading_weight_ * 5.0 * abs(angle_diff) * (1.0 + 5.0/goal_dist);

        // Collision scoring
        double collision_score = check_collision(traj.path);

        // Smoothness scoring (prefer similar speeds/turns to current)
        double speed_diff = abs(traj.speed - current_speed_);
        double turn_diff = abs(traj.turn_rate - current_turn_rate_);
        double smoothness_score = -0.5 * speed_diff - 0.8 * turn_diff;

        // Speed scoring (prefer moderate speeds)
        double speed_score = 0.0;
        if (traj.speed > max_speed_ * 0.6) {
            speed_score = 1.0;
        } else if (traj.speed > max_speed_ * 0.3) {
            speed_score = 0.5;
        }

        // Slow down when approaching goal
        double slow_down_factor = 1.0;
        if (goal_dist < slow_down_distance_) {
            slow_down_factor = goal_dist / slow_down_distance_;
            speed_score *= slow_down_factor;
        }

        return goal_score + heading_score + collision_score + smoothness_score + speed_score;
    }

    void visualize_trajectories(const std::vector<Trajectory>& trajectories, const Trajectory& best_traj) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Add safety margin visualization
        if (odom_data_) {
            visualization_msgs::msg::Marker safety_marker;
            safety_marker.header.frame_id = odom_data_->header.frame_id;
            safety_marker.header.stamp = now();
            safety_marker.ns = "safety_margin";
            safety_marker.id = 0;
            safety_marker.type = visualization_msgs::msg::Marker::SPHERE;
            safety_marker.action = visualization_msgs::msg::Marker::ADD;
            safety_marker.scale.x = (safety_margin_ + robot_radius_) * 2.0;
            safety_marker.scale.y = (safety_margin_ + robot_radius_) * 2.0;
            safety_marker.scale.z = 0.1;
            safety_marker.color.a = 0.2;
            safety_marker.color.r = 1.0;
            safety_marker.color.g = 0.0;
            safety_marker.color.b = 0.0;
            safety_marker.pose.position.x = odom_data_->pose.pose.position.x;
            safety_marker.pose.position.y = odom_data_->pose.pose.position.y;
            marker_array.markers.push_back(safety_marker);

            // Add goal marker (larger for visibility)
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = odom_data_->header.frame_id;
            goal_marker.header.stamp = now();
            goal_marker.ns = "goal";
            goal_marker.id = 0;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            goal_marker.scale.x = goal_tolerance_ * 2.2;  // Slightly larger than actual tolerance
            goal_marker.scale.y = goal_tolerance_ * 2.2;
            goal_marker.scale.z = 0.1;
            goal_marker.color.a = 0.5;
            goal_marker.color.r = 0.0;
            goal_marker.color.g = 1.0;
            goal_marker.color.b = 0.0;
            goal_marker.pose.position.x = goal_x_;
            goal_marker.pose.position.y = goal_y_;
            marker_array.markers.push_back(goal_marker);

            // Add stopping buffer visualization
            visualization_msgs::msg::Marker buffer_marker;
            buffer_marker.header.frame_id = odom_data_->header.frame_id;
            buffer_marker.header.stamp = now();
            buffer_marker.ns = "stopping_buffer";
            buffer_marker.id = 0;
            buffer_marker.type = visualization_msgs::msg::Marker::SPHERE;
            buffer_marker.action = visualization_msgs::msg::Marker::ADD;
            buffer_marker.scale.x = (stopping_buffer_) * 2.0;
            buffer_marker.scale.y = (stopping_buffer_) * 2.0;
            buffer_marker.scale.z = 0.1;
            buffer_marker.color.a = 0.3;
            buffer_marker.color.r = 1.0;
            buffer_marker.color.g = 1.0;
            buffer_marker.color.b = 0.0;
            buffer_marker.pose.position.x = goal_x_;
            buffer_marker.pose.position.y = goal_y_;
            marker_array.markers.push_back(buffer_marker);
        }

        // Visualize all trajectories
        int id = 0;
        for (const auto& traj : trajectories) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = odom_data_ ? odom_data_->header.frame_id : "map";
            marker.header.stamp = now();
            marker.ns = "trajectories";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.02;
            marker.color.a = 0.3;
            marker.color.r = 0.0;
            marker.color.g = 0.5;
            marker.color.b = 1.0;

            for (const auto& point : traj.path) {
                geometry_msgs::msg::Point p;
                p.x = point.first;
                p.y = point.second;
                p.z = 0.0;
                marker.points.push_back(p);
            }
            marker_array.markers.push_back(marker);
        }

        // Highlight best trajectory
        if (!best_traj.path.empty()) {
            visualization_msgs::msg::Marker best_marker;
            best_marker.header.frame_id = odom_data_ ? odom_data_->header.frame_id : "map";
            best_marker.header.stamp = now();
            best_marker.ns = "best_trajectory";
            best_marker.id = 0;
            best_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            best_marker.action = visualization_msgs::msg::Marker::ADD;
            best_marker.scale.x = 0.05;
            best_marker.color.a = 1.0;
            best_marker.color.r = 0.0;
            best_marker.color.g = 1.0;
            best_marker.color.b = 0.0;

            for (const auto& point : best_traj.path) {
                geometry_msgs::msg::Point p;
                p.x = point.first;
                p.y = point.second;
                p.z = 0.0;
                best_marker.points.push_back(p);
            }
            marker_array.markers.push_back(best_marker);
        }

        marker_pub_->publish(marker_array);
    }

    void control_loop() {
        if (!odom_data_ || !scan_data_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "Waiting for odometry and laser scan data...");
            return;
        }

        // Check if goal reached
        double dist_to_goal = hypot(goal_x_ - odom_data_->pose.pose.position.x,
                                  goal_y_ - odom_data_->pose.pose.position.y);
        
        // Define stopping zones
        const double stopping_distance = stopping_buffer_;
        const double slow_down_zone = goal_tolerance_;
        
        if (dist_to_goal < stopping_distance) {
            // In the stopping zone - force stop
            if (!goal_reached_) {
                goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping.");
            }
            
            // Stop the robot completely
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_pub_->publish(stop_cmd);
            current_speed_ = 0.0;
            current_turn_rate_ = 0.0;
            return;
        } 
        else if (dist_to_goal < slow_down_zone) {
            // In the slow down zone - reduce speed significantly
            double slow_down_factor = dist_to_goal / slow_down_zone;
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = std::min(current_speed_, max_speed_ * 0.2 * slow_down_factor);
            cmd.angular.z = current_turn_rate_ * 0.3;  // Reduce turning more aggressively
            cmd_pub_->publish(cmd);
            current_speed_ = cmd.linear.x;
            current_turn_rate_ = cmd.angular.z;
            return;
        } 
        else {
            goal_reached_ = false;  // Reset if we moved away from goal
        }

        // Generate and evaluate trajectories
        std::vector<Trajectory> trajectories;
        Trajectory best_traj;
        double best_score = -INFINITY;

        // Generate samples considering acceleration limits
        std::uniform_real_distribution<> speed_dist(
            std::max(min_speed_, current_speed_ - max_acceleration_ * step_time_),
            std::min(max_speed_, current_speed_ + max_acceleration_ * step_time_));
        
        std::uniform_real_distribution<> turn_dist(
            std::max(-max_turn_, current_turn_rate_ - max_turn_acceleration_ * step_time_),
            std::min(max_turn_, current_turn_rate_ + max_turn_acceleration_ * step_time_));

        for (int i = 0; i < eval_samples_; ++i) {
            double speed = speed_dist(gen_);
            double turn = turn_dist(gen_);
            
            // Ensure we don't generate negative speeds
            speed = std::max(min_speed_, speed);
            
            Trajectory traj = simulate_trajectory(speed, turn);
            traj.score = evaluate_trajectory(traj);
            
            trajectories.push_back(traj);
            
            if (traj.score > best_score) {
                best_score = traj.score;
                best_traj = traj;
            }
        }

        // Visualize trajectories
        visualize_trajectories(trajectories, best_traj);

        // Execute best trajectory if valid
        if (best_score > -INFINITY) {
            // Apply acceleration limits
            double speed_diff = best_traj.speed - current_speed_;
            double max_speed_change = (speed_diff > 0) ? 
                max_acceleration_ * step_time_ : 
                max_deceleration_ * step_time_;
            
            double new_speed = current_speed_ + std::copysign(
                std::min(abs(speed_diff), max_speed_change), speed_diff);
            
            // Apply turn rate limits
            double turn_diff = best_traj.turn_rate - current_turn_rate_;
            double max_turn_change = max_turn_acceleration_ * step_time_;
            double new_turn = current_turn_rate_ + std::copysign(
                std::min(abs(turn_diff), max_turn_change), turn_diff);

            // Slow down when approaching goal
            if (dist_to_goal < slow_down_distance_) {
                double slow_down_factor = dist_to_goal / slow_down_distance_;
                new_speed = std::min(new_speed, max_speed_ * slow_down_factor);
                
                // More aggressive slowing as we get closer
                if (dist_to_goal < slow_down_distance_/2) {
                    new_speed = std::min(new_speed, max_speed_ * 0.3 * slow_down_factor);
                }
            }

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = new_speed;
            cmd.angular.z = new_turn;
            cmd_pub_->publish(cmd);

            // Update current state
            current_speed_ = new_speed;
            current_turn_rate_ = new_turn;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "No valid trajectory found! Stopping.");
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_pub_->publish(stop_cmd);
            current_speed_ = 0.0;
            current_turn_rate_ = 0.0;
        }
    }

    void set_goal(double x, double y) {
        goal_x_ = x;
        goal_y_ = y;
        goal_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "New goal set to (%.2f, %.2f)", goal_x_, goal_y_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWALocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
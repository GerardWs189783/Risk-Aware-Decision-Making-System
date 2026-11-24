#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <limits>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rover_interfaces/msg/perception_result_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "action_msgs/msg/goal_status.hpp"
#include <std_srvs/srv/empty.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

// Check Mission Objective (first action node to set the goal)

class CheckMissionObjective : public BT::SyncActionNode {
private:
    std::vector<std::string> waypoints_ = {"Waypoint_Alpha", "Waypoint_Beta"};
    size_t current_index_ = 0;
    std::string current_goal_ = "";
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    std_msgs::msg::String msg;

public:
    CheckMissionObjective(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
        : BT::SyncActionNode(name, config), ros_node_(ros_node) 
    {
        // Initialize the publisher
        status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/bt/mission_status", 10);
    }
    
    static BT::PortsList providedPorts() { 
        return { BT::OutputPort<std::string>("goal_output") }; 
    }
    
    BT::NodeStatus tick() override {
        bool advance = false;
        // Check if FollowTrajectory successfully reached the previous goal
        if (!config().blackboard->get("waypoint_reached", advance)) {
            advance = false; // Default to false on the very first tick before the flag exists
        }

        // If first tick, or the previous goal was reached, advance the queue
        if (advance || current_goal_.empty()) {
            if (current_index_ < waypoints_.size()) {
                current_goal_ = waypoints_[current_index_];
                current_index_++;
                
                // Reset the flag so we don't accidentally skip waypoints
                config().blackboard->set("waypoint_reached", false);
                std::cout << "\033[1;34m[BT] Mission Objective Updated: " << current_goal_ << "\033[0m\n";
            } else {
                std::cout << "\033[1;32m[BT] ALL WAYPOINTS COMPLETED! Mission Success.\033[0m\n";
                // Return FAILURE to stop the tree from looping once the mission is completely done
                
                msg.data = "SUCCESS";
                status_pub_->publish(msg);
                return BT::NodeStatus::FAILURE; 
            }
        }
        
        setOutput("goal_output", current_goal_);
        return BT::NodeStatus::SUCCESS;
    }
};

// Execute Task (only ticking this node just to tell if the BT goes correclty)
class ExecuteTask : public BT::SyncActionNode {
public:
    explicit ExecuteTask(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "[BT] Task execution..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }


}; 

// ========================================================
// Risk Assessment Node -> part of formal extension to handle uncertainty as in thesis paper derived
// ========================================================

class RiskAssessment : public BT::SyncActionNode {
private:
    rclcpp::Node::SharedPtr ros_node_;
    
    // Perception Subs
    rclcpp::Subscription<rover_interfaces::msg::PerceptionResultArray>::SharedPtr sub_;
    rover_interfaces::msg::PerceptionResultArray::SharedPtr last_msg_;
    rclcpp::Time last_msg_time_;
    
    // Odometry Subscription for dynamic v_t
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double current_velocity_ = 0.0;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Path sub for true Cross-Track Error (msg::Path is array of point on the map that belong to the path)
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav_msgs::msg::Path::SharedPtr current_path_;
    
    // List of rocks already handled by IsDanger/IsPlanning
    std::shared_ptr<std::vector<std::pair<double, double>>> confirmed_obstacles_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    
    int publish_counter_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr risk_pub_;
    std_msgs::msg::Float64 risk_msg_;

public:
    RiskAssessment(const std::string& name, const BT::NodeConfig& config, 
                   rclcpp::Node::SharedPtr ros_node, 
                   std::shared_ptr<std::vector<std::pair<double, double>>> shared_memory) 
        : BT::SyncActionNode(name, config), ros_node_(ros_node), confirmed_obstacles_(shared_memory) 
    {
        sub_ = ros_node_->create_subscription<rover_interfaces::msg::PerceptionResultArray>(
            "/perception/obstacle_info", 10,
            [this](const rover_interfaces::msg::PerceptionResultArray::SharedPtr msg) {
                this->last_msg_ = msg;
                this->last_msg_time_ = ros_node_->now();
            });

        // Odom sub needed to get dynamic velocity (v_t) for the C_index
        odom_sub_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Get absolute forward speed
                this->current_velocity_ = std::abs(msg->twist.twist.linear.x); 
            });

        // Path sub needed for the path to handle
        path_sub_ = ros_node_->create_subscription<nav_msgs::msg::Path>(
            "/plan",10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                // get the map array
                this->current_path_ = msg;
            }
        );    
        // clear_global_client_ = ros_node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
        // clear_local_client_ = ros_node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        point_cloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_obstacles", 10);

        risk_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>("/bt/current_risk", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<double>("target_x"),
            BT::OutputPort<double>("target_y"),
            BT::OutputPort<double>("target_tilt"),
            BT::OutputPort<double>("target_yaw"),
            BT::OutputPort<double>("target_distance"),
            BT::OutputPort<double>("risk_value"),
        };
    }

    BT::NodeStatus tick() override {
        //Prevention of missing the data
       if (!confirmed_obstacles_->empty()) {
            publish_counter_++;
            if (publish_counter_ >= 100) { 
                auto cloud = sensor_msgs::msg::PointCloud2();
                cloud.header.frame_id = "odom";
                cloud.header.stamp = ros_node_->now();

                sensor_msgs::PointCloud2Modifier modifier(cloud);
                modifier.setPointCloud2FieldsByString(1, "xyz");
                modifier.resize(confirmed_obstacles_->size());

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

                for (const auto& obs : *confirmed_obstacles_) {
                    *iter_x = static_cast<float>(obs.first);
                    *iter_y = static_cast<float>(obs.second);
                    *iter_z = 0.0f;
                    ++iter_x; ++iter_y; ++iter_z;
                }
                point_cloud_pub_->publish(cloud);
                publish_counter_ = 0; 
            }
        }


        // No data or no detection
        if (!last_msg_ || !last_msg_->obstacle_detected || last_msg_->detections.empty()) {
            setOutput("risk_value", 0.0);
            risk_msg_.data = 0.0;
            risk_pub_->publish(risk_msg_);
            return BT::NodeStatus::SUCCESS;
        }
        auto current_time = ros_node_->now();
        if ((current_time - last_msg_time_).seconds() > 1.0) {
            RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 1000, 
                                 "Perception data is stale! Ignoring old rocks.");
            setOutput("risk_value", 0.0);
            risk_msg_.data = 0.0;
            risk_pub_->publish(risk_msg_);
            return BT::NodeStatus::SUCCESS;
        }

        double max_risk = 0.0;
        bool threat_found = false;
        
        // holding the highest risk rock
        struct ThreatData {
            double x, y, tilt, yaw, dist;
        } highest_threat;

        for (const auto& rock : last_msg_->detections) {
            
            // local coordinates
            double rel_x = rock.distance_meters * std::cos(rock.theta_angle);
            double rel_y = rock.distance_meters * std::sin(rock.theta_angle);

            geometry_msgs::msg::PointStamped point_sensor;
            point_sensor.header.frame_id = "camera_link"; 
            point_sensor.header.stamp = last_msg_time_;
            point_sensor.point.x = rel_x;
            point_sensor.point.y = rel_y;
            point_sensor.point.z = 0.0;

            try {

                // ========================================================
                // 1. FOOTPRINT MASK
                // ========================================================
                // Transform to base_link to check if the rock is physically touching the rover
                auto point_base = tf_buffer_->transform(point_sensor, "base_link", tf2::durationFromSec(0.1));
                
                // Your footprint limits from YAML, plus a 0.2m margin for arm overhang/shadows
                double min_x = -1.841 - 0.15;
                double max_x =  2.341 + 0.15;
                double min_y = -1.469 - 0.15;
                double max_y =  1.469 + 0.15;

                if (point_base.point.x >= min_x && point_base.point.x <= max_x &&
                    point_base.point.y >= min_y && point_base.point.y <= max_y) {
                    // The detection is inside the rover's physical body. Ignore it completely!
                    continue; 
                }
                //global coordinates
                auto point_odom = tf_buffer_->transform(point_sensor, "odom", tf2::durationFromSec(0.1));
                double global_x = point_odom.point.x;
                double global_y = point_odom.point.y;
                
                // Is this rock already handled?
                bool already_known = false;
                for (const auto& known : *confirmed_obstacles_) {
                    if (std::hypot(global_x - known.first, global_y - known.second) < 0.75) { 
                        already_known = true; 
                        break; 
                    }
                }

                // If handled, SKIP 
                if (already_known) {
                    continue; 
                }

                // ========================================================
                // PLATT SCALING (CALIBRATED PROBABILITY)
                // ========================================================
                double raw_confidence = rock.confidence; 
                double eps = 1e-7;
                double conf_clipped = std::max(eps, std::min(1.0 - eps, raw_confidence));
                double logit = std::log(conf_clipped / (1.0 - conf_clipped));
                double A = 1.68825; 
                double B = 0.02879;
                double p_obstacle = 1.0 / (1.0 + std::exp(-(A * logit + B))); 

                // ========================================================
                // PROBABILITY OF COLLISION
                // ========================================================
                double chassis_width = 1.53774;
                double polygon_stop_margin = 0.75;
                chassis_width = chassis_width + polygon_stop_margin;
                double rover_radius = chassis_width / 2.0;
                double rock_radius = 0.50;  // Slightly larger estimate for "giant boulders"
                double safety_buffer = 0.15; // Margin for Nav2 steering error
                
                double clearance_margin = rover_radius + rock_radius + safety_buffer;
                
                // Calculate delta_y using nav_msgs Path
                double delta_y = std::abs(rel_y); // only if path missing then using relative to the rover position
                if (current_path_ && !current_path_->poses.empty()) {
                    try
                    {
                        // avoid frames mismatch
                        auto point_path_frame = tf_buffer_->transform(point_sensor, current_path_->header.frame_id, tf2::durationFromSec(0.0));
                        double rock_path_x = point_path_frame.point.x;
                        double rock_path_y = point_path_frame.point.y;

                        double min_dist = std::numeric_limits<double>::max();

                        for (const auto& pose_stamped : current_path_->poses) {

                        double path_x = pose_stamped.pose.position.x;
                        double path_y = pose_stamped.pose.position.y;
                        
                        // Euclidean distance
                        double dist = std::hypot(rock_path_x - path_x, rock_path_y - path_y);
                        if (dist < min_dist) {
                            min_dist = dist;
                        }
                    }
                    delta_y = min_dist; // the closest path point to the rock is the delta_y
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_WARN(ros_node_->get_logger(), "TF2 Error transforming rock to path frame: %s", ex.what());
                        // If fail delta_y = std::abs(rel_y)
                    }
                }
                double effective_delta_y = std::max(0.0, delta_y - clearance_margin);
                double var_traj = 0.01; 
                double var_control = 0.01;
                double var_track = var_traj + var_control; 
                ros_node_->get_parameter("var_track", var_track);
                
                double p_cond_col_obs = std::exp(-(effective_delta_y * effective_delta_y) / (2 * var_track));
                double p_collision = p_obstacle * p_cond_col_obs;

                // ========================================================
                // COLLISION SEVERITY INDEX
                // ========================================================
                double t_react = 0.5; // system reaction time
                ros_node_->get_parameter("t_react", t_react);
                // velocity readings from "/odom"
                double v_t = std::max(0.1, current_velocity_); 
                double a_max = 2.5;   // from yaml
                // d_stop calculation
                double kinematic_d_stop = (v_t * t_react) + (v_t * v_t) / (2 * a_max);
                
                // d_safe buffer for coping with low c value
                // so the rover never creeps too close at low speeds.
                double d_safe = 1.5;
                ros_node_->get_parameter("d_safe", d_safe);
                double d_stop = std::max(d_safe, kinematic_d_stop);
                
                double d_obstacle = std::max(0.1, (double)rock.distance_meters);
                
                double c_index = std::min(1.0, (d_stop / d_obstacle));

                // ========================================================
                // EXPECTED COLLISION RISK
                // ========================================================
                double risk = p_collision * c_index;

                // Filtering the rocks to found the one with highest risk -> one to take
                if (risk > max_risk) {
                    max_risk = risk;
                    highest_threat.x = global_x;
                    highest_threat.y = global_y;
                    highest_threat.tilt = rock.phi_angle;
                    highest_threat.yaw = rock.theta_angle;
                    highest_threat.dist = rock.distance_meters;
                    threat_found = true;
                }

            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(ros_node_->get_logger(), "TF2 Error in Risk Assessment: %s", ex.what());
                // Do not return failure here, just let it skip this specific rock and check the others
            }
        }
        
        // if already known
        if (!threat_found) {
            setOutput("risk_value", 0.0);
            risk_msg_.data = 0.0;
            risk_pub_->publish(risk_msg_);
            return BT::NodeStatus::SUCCESS;
        }

        // The highest RISK rock to the Blackboard
        setOutput("target_x", highest_threat.x);
        setOutput("target_y", highest_threat.y);
        setOutput("target_tilt", highest_threat.tilt); 
        setOutput("target_yaw", highest_threat.yaw);
        setOutput("target_distance", highest_threat.dist);
        setOutput("risk_value", max_risk);
        risk_msg_.data = max_risk;
        risk_pub_->publish(risk_msg_);
        return BT::NodeStatus::SUCCESS; 
    }        
};

class IsDanger : public BT::ConditionNode{
private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    // to handle the rock saved in blackboard
    std::shared_ptr<std::vector<std::pair<double, double>>> confirmed_obstacles_;
    // needed for point cloud publish
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    std_msgs::msg::String msg;
    // threshold
    double tau_2_ = 0.75; 
    // PointCloud publishing
    void publishSemanticMap() {
        auto cloud = sensor_msgs::msg::PointCloud2();
        cloud.header.frame_id = "odom";
        cloud.header.stamp = ros_node_->now();

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(confirmed_obstacles_->size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto& obs : *confirmed_obstacles_) {
            *iter_x = static_cast<float>(obs.first);
            *iter_y = static_cast<float>(obs.second);
            *iter_z = 0.0f;
            ++iter_x; ++iter_y; ++iter_z;
        }
        cloud.header.stamp = ros_node_->now();
        point_cloud_pub_->publish(cloud);
    }

public:
    IsDanger(const std::string& name, const BT::NodeConfig& config, 
             rclcpp::Node::SharedPtr ros_node, 
             std::shared_ptr<std::vector<std::pair<double, double>>> shared_memory)
        : BT::ConditionNode(name, config), ros_node_(ros_node), confirmed_obstacles_(shared_memory) 
    {
        // Init pub for the costmap update
        point_cloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_obstacles", 10);

        state_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/bt/state_transition", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("risk_value"),
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }

    BT::NodeStatus tick() override {
        double risk = 0.0;
        ros_node_->get_parameter("tau_2", tau_2_);
        if (!getInput("risk_value", risk)) {
            return BT::NodeStatus::FAILURE;
        }

        // Check the treshold       
        if (risk >= tau_2_) {
            double x = 0.0, y = 0.0;
            if (getInput("target_x", x) && getInput("target_y", y)) {
                
                std::cout << "\033[1;31m[BT] DANGER THRESHOLD MET! Risk: " << risk 
                          << ". Triggering Stop for LiDAR inspection...\033[0m\n";

                msg.data = "DANGER";
                state_pub_->publish(msg);          
                
                // Add to shared memory so RiskAssessment skips it on the next tick
                //confirmed_obstacles_->push_back({x, y});
                
                // Publish to Nav2 so the planner routes around it when it resumes
                //publishSemanticMap();
            }
            // Return SUCCESS to trigger the sequence (EmergencyStop -> ReactToObstacle)
            return BT::NodeStatus::SUCCESS;
        }

        // Risk is lower than tau_2, move to the Planning branch
        return BT::NodeStatus::FAILURE;
    }
};

class IsPlanning : public BT::ConditionNode {
private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    // confirmed rock handle
    std::shared_ptr<std::vector<std::pair<double, double>>> confirmed_obstacles_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    std_msgs::msg::String msg;
    // treshold lower bound for the planning zone
    double tau_1_ = 0.3; 

    void publishSemanticMap() {
        auto cloud = sensor_msgs::msg::PointCloud2();
        cloud.header.frame_id = "odom";
        cloud.header.stamp = ros_node_->now();

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(confirmed_obstacles_->size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto& obs : *confirmed_obstacles_) {
            *iter_x = static_cast<float>(obs.first);
            *iter_y = static_cast<float>(obs.second);
            *iter_z = 0.0f;
            ++iter_x; ++iter_y; ++iter_z;
        }
        cloud.header.stamp = ros_node_->now();
        point_cloud_pub_->publish(cloud);
    }

public:
    IsPlanning(const std::string& name, const BT::NodeConfig& config, 
               rclcpp::Node::SharedPtr ros_node, 
               std::shared_ptr<std::vector<std::pair<double, double>>> shared_memory)
        : BT::ConditionNode(name, config), ros_node_(ros_node), confirmed_obstacles_(shared_memory) 
    {
        point_cloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_obstacles", 10);

        state_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/bt/state_transition", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("risk_value"),
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }

    BT::NodeStatus tick() override {
        double risk = 0.0;
        ros_node_->get_parameter("tau_1", tau_1_);
        if (!getInput("risk_value", risk)) {
            return BT::NodeStatus::FAILURE;
        }

        // planning threshold check
        if (risk >= tau_1_) {
            double x = 0.0, y = 0.0;
            if (getInput("target_x", x) && getInput("target_y", y)) {
                
                // Yellow text for medium risk
                std::cout << "\033[1;33m[BT] PLANNING THRESHOLD MET! Risk: " << risk 
                          << ". Updating Map (Smooth Avoidance)...\033[0m\n";
                
                // to point cloud
                
                msg.data = "PLANNING";
                state_pub_->publish(msg);

                confirmed_obstacles_->push_back({x, y});
                std::cout << confirmed_obstacles_->size();
                publishSemanticMap();
            }
            return BT::NodeStatus::SUCCESS;
        }

        // Risk is lower than tau_1. Move to Monitor branch.
        return BT::NodeStatus::FAILURE;
    }
};

class IsMonitor : public BT::ConditionNode {
private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    std_msgs::msg::String msg;

public:
    IsMonitor(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node)
        : BT::ConditionNode(name, config), ros_node_(ros_node) 
    {
        state_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/bt/state_transition", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("risk_value")
        };
    }

    BT::NodeStatus tick() override {
        double risk = 0.0;
        
        if (!getInput("risk_value", risk)) {
            return BT::NodeStatus::FAILURE;
        }

        if (risk > 0.0) {
            std::cout << "\033[1;32m[BT] Monitor Mode. Risk: " << risk 
                      << " is too low to act. Monitoring.\033[0m\n";

            msg.data = "MONITORING";
            state_pub_->publish(msg);          
        }
        return BT::NodeStatus::SUCCESS;
    }
};

class EmergencyStop : public BT::SyncActionNode {
private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr brake_client_;

public:
    EmergencyStop(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
        : BT::SyncActionNode(name, config), ros_node_(ros_node) 
    {
        brake_client_ = ros_node_->create_client<std_srvs::srv::Empty>("/move_stop");
    }

    static BT::PortsList providedPorts() {
        return {}; // No blackboard ports required
    }

    BT::NodeStatus tick() override {
        std::cout << "\033[1;31m[BT] Executing Emergency Stop!\033[0m\n";
        
        // Call the stopping service
        if (brake_client_->wait_for_service(std::chrono::milliseconds(50))) {
            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            brake_client_->async_send_request(req);
        } else {
            RCLCPP_WARN(ros_node_->get_logger(), "/stop service not available!");
        }

        // Return SUCCESS instantly so the Sequence can proceed to ReactToObstacle
        return BT::NodeStatus::SUCCESS;
    }
};

// Follow Trajectory (Mock Async Action)
class FollowTrajectory : public BT::StatefulActionNode {
private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_pose_client_;
    
    // Dictionary of waypoints
    // std::map<std::string, std::pair<double, double>> waypoints_ {
    //     {"Waypoint_Alpha", {-5.0, 13.0}},
    //     {"Waypoint_Beta", {-3.0, 20.0}}
    // };
    std::map<std::string, std::pair<double, double>> waypoints_ {
        {"Waypoint_Alpha", {5.2, 6.5}},
        {"Waypoint_Beta", {5.0, 16.0}}
    };
    // std::map<std::string, std::pair<double, double>> waypoints_ {
    //     {"Waypoint_Alpha", {21.7, 26.4}},
    //     {"Waypoint_Beta", {0.5, 11.0}}
    // };
    // std::map<std::string, std::pair<double, double>> waypoints_ {
    //     {"Waypoint_Alpha", {0.7,13.0}},
    //     {"Waypoint_Beta", {13.0, 16.0}}
    // };
    // std::map<std::string, std::pair<double, double>> waypoints_ {
    //     {"Waypoint_Alpha", {-8.0, 15.0}},
    //     {"Waypoint_Beta", {-7.0, -2.0}}
    // };

    bool goal_reached_ = false;
    bool goal_responded_ = false;
    bool goal_accepted_ = false;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

public:
    FollowTrajectory(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
        : BT::StatefulActionNode(name, config), ros_node_(ros_node) 
    {
        nav_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ros_node_, "/navigate_to_pose");
        status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/bt/mission_status", 10); 
    }    
    
    static BT::PortsList providedPorts() { 
        return { BT::InputPort<std::string>("goal_input") }; 
    }

    BT::NodeStatus onStart() override {
        std::string goal_str;
        if (!getInput("goal_input", goal_str)) {
            RCLCPP_WARN(ros_node_->get_logger(), "Missing 'goal_input' port!");
            return BT::NodeStatus::FAILURE;
        }

        auto it = waypoints_.find(goal_str);
        if (it == waypoints_.end()) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Waypoint '%s' not found in dictionary!", goal_str.c_str());
            return BT::NodeStatus::FAILURE;
        }
        
        double target_x = it->second.first;
        double target_y = it->second.second;

        if (!nav_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Nav2 Action Server is offline! Cannot drive.");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map"; 
        goal_msg.pose.header.stamp = ros_node_->now();
        goal_msg.pose.pose.position.x = target_x;
        goal_msg.pose.pose.position.y = target_y;
        goal_msg.pose.pose.position.z = 0.0;
        
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
                this->goal_responded_ = true;
                if (!goal_handle) {
                    std::cout << "\033[1;31m[BT] Nav2 Rejected the Goal!\033[0m\n";
                    this->goal_accepted_ = false;
                } else {
                    std::cout << "\033[1;32m[BT] Goal successfully passed to Nav2\033[0m\n";
                    this->goal_accepted_ = true;
                    this->goal_handle_ = goal_handle; 
                }
            };

        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
                (void)result;
                this->goal_reached_ = true; 
            };

        std::cout << "\033[1;36m[BT] Sending Nav2 Goal: " << goal_str 
                  << " (X: " << target_x << ", Y: " << target_y << ")...\033[0m\n";
        
        nav_pose_client_->async_send_goal(goal_msg, send_goal_options);
        
        goal_responded_ = false;
        goal_accepted_ = false;
        goal_reached_ = false; 
        
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override {
        if (!goal_responded_) return BT::NodeStatus::RUNNING; 
        if (goal_responded_ && !goal_accepted_) return BT::NodeStatus::FAILURE; 

        if (goal_reached_) {
            auto status = goal_handle_->get_status();
            if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                std::cout << "\033[1;32m[BT] Waypoint Reached Successfully!\033[0m\n";
                config().blackboard->set("waypoint_reached", true);
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "\033[1;31m[BT] Nav2 Failed to reach the goal.\033[0m\n";
                std_msgs::msg::String msg;
                msg.data = "FAILED";
                status_pub_->publish(msg);
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING; 
    }

    void onHalted() override {
        // cleaning up Nav2.
        std::cout << "\033[1;33m[BT] FollowTrajectory Halted! Canceling Nav2 Goal...\033[0m\n";
        
        if (nav_pose_client_ && goal_handle_) {
            nav_pose_client_->async_cancel_goal(goal_handle_);
        }
        
        goal_handle_.reset();
    }
};

// React to Obstacle (different than in classic)
class ReactToObstacle : public BT::StatefulActionNode {
private:
    // joint sub and pub to react according to the zone
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    //LiDAR
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Time scan_start_time_;
    rclcpp::Time movement_start_time_;
    rclcpp::Time last_scan_time_;

    // relocation of obstacle possible

    std::shared_ptr<std::vector<std::pair<double, double>>> confirmed_obstacles_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double original_target_yaw_ = 0.0; // original relative yaw for math
    double refined_distance_ = 0.0;    // distance measured by LiDAR

    double current_mast_p_ = 0.0;    // Joint 1
    double current_yaw_ = 0.0;       // Joint 2 (mast_02_joint) YAW
    double current_tilt_ = 0.0;      // Joint 3 (mast_cameras_joint)   PITCH

    double expected_distance_ = 0.0;
    double target_yaw_absolute_ = 0.0;
    double target_tilt_absolute_ = 0.0;
    
    bool is_looking_down_ = false;
    bool waiting_for_scan_ = false; 
    bool lidar_hit_confirmed_ = false; // if the lidar hit the rock
    const std::string topic_name = "/mast_joint_trajectory_controller/joint_trajectory"; // topic for traj control to send the desired joint angle

    //republish to point cloud

    void publishSemanticMap() {
        auto cloud = sensor_msgs::msg::PointCloud2();
        cloud.header.frame_id = "odom";
        cloud.header.stamp = ros_node_->now();

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(confirmed_obstacles_->size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto& obs : *confirmed_obstacles_) {
            *iter_x = static_cast<float>(obs.first);
            *iter_y = static_cast<float>(obs.second);
            *iter_z = 0.0f;
            ++iter_x; ++iter_y; ++iter_z;
        }
        cloud.header.stamp = ros_node_->now();
        point_cloud_pub_->publish(cloud);
    }

public:
    ReactToObstacle(const std::string& name, const BT::NodeConfig& config, 
                    rclcpp::Node::SharedPtr ros_node,
                    std::shared_ptr<std::vector<std::pair<double, double>>> shared_memory) 
            : BT::StatefulActionNode(name, config), ros_node_(ros_node), confirmed_obstacles_(shared_memory)
    {
        joint_pub_ = ros_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic_name, 10);
        point_cloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_obstacles", 10);
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        joint_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (msg->name[i] == "mast_p_joint") this->current_mast_p_ = msg->position[i];
                    else if (msg->name[i] == "mast_02_joint") this->current_yaw_ = msg->position[i];       
                    else if (msg->name[i] == "mast_cameras_joint") this->current_tilt_ = msg->position[i];      
                }
            });

        //Subscribe to the 1D LIDAR topic
        scan_sub_ = ros_node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                
                if (this->waiting_for_scan_ && !this->lidar_hit_confirmed_) {
                    int center_idx = msg->ranges.size() / 2;
                    int search_window = 20; 
                    int start_idx = std::max(0, center_idx - search_window);
                    int end_idx = std::min((int)msg->ranges.size(), center_idx + search_window);

                    double best_match_diff = 0.4; // Strict 0.4m tolerance
                    bool found_better_match = false;
                    double best_range = 0.0;

                    for (int i = start_idx; i < end_idx; i++) {
                        float range = msg->ranges[i];
                        
                        if (!std::isinf(range) && !std::isnan(range) && range > 0.1) {
                            // Find how close this LiDAR ray is to the Camera's distance
                            double diff = std::abs(range - this->expected_distance_);
                            
                            // Keep the ray that most perfectly matches the camera's estimate
                            if (diff < best_match_diff) {
                                best_match_diff = diff;
                                best_range = range;
                                found_better_match = true;
                            }
                        }
                    }

                    // If we found a valid ray in the window, confirm the hit
                    if (found_better_match) {
                        this->lidar_hit_confirmed_ = true;
                        this->refined_distance_ = best_range;
                        this->last_scan_time_ = msg->header.stamp; 
                    }
                }
            });
    }

    static BT::PortsList providedPorts() {
        return{
            //blackboard input ports
            BT::InputPort<double>("target_tilt"),
            BT::InputPort<double>("target_yaw"),
            BT::InputPort<double>("target_distance")
        };
    }

    BT::NodeStatus onStart() override 
    {
        //check if the variable of Blackboard are assigned correctly
        double target_tilt = 0.0;
        if (!getInput("target_tilt", target_tilt) || 
            !getInput("target_yaw", original_target_yaw_) || 
            !getInput("target_distance", expected_distance_)) {
            RCLCPP_WARN(ros_node_->get_logger(),"Missing BT variables!");
            return BT::NodeStatus::FAILURE;
        }
        expected_distance_ = std::max(0.1, expected_distance_);
        // needed to be included for the mast tilt. If very close the tilt correction will be bigger is further it will be small
        double lidar_height_offset = 0.325;
        double parallax_correction = std::atan(lidar_height_offset / expected_distance_);
        //final target pitch (tilt) and yaw for he action of active perception checking
        target_tilt_absolute_ = current_tilt_+ target_tilt + parallax_correction;
        target_yaw_absolute_ = current_yaw_ + original_target_yaw_;

        std::cout << "\033[1;35m[BT] -> FALLBACK: Panning to " << target_yaw_absolute_ 
                  << " rads, and Tilting to " << target_tilt_absolute_ << " ...\033[0m\n";
        publishMastCommand(target_yaw_absolute_,target_tilt_absolute_);
        
        is_looking_down_ = true;
        waiting_for_scan_ = false;
        lidar_hit_confirmed_ = false; // Reset the laser flag
        movement_start_time_ = ros_node_->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override 
    {
        // yaw and pitch reached with a tolerence
        bool target_reached = (std::abs(current_tilt_ - target_tilt_absolute_) < 0.05 &&
                               std::abs(current_yaw_ - target_yaw_absolute_) < 0.05);
        bool movement_timeout = (ros_node_->now() - movement_start_time_).seconds() > 4.0;

        if (is_looking_down_ && !waiting_for_scan_ && (target_reached || movement_timeout)) {
            
            if (movement_timeout) {
                std::cout << "\033[1;33m[BT] -> FALLBACK: Mast movement timeout! Scanning anyway...\033[0m\n";
            } else {
                std::cout << "\033[1;36m[BT] -> FALLBACK: Target reached. Waiting for LiDAR strike...\033[0m\n";
            }
            
            waiting_for_scan_ = true; 
            scan_start_time_ = ros_node_->now(); 
            return BT::NodeStatus::RUNNING;
        }

        // LIDAR scan the rock, if waiting_for_scan true then the system waits for the lidar to scan (by the time the yaw and pitch will be reached)
        if (waiting_for_scan_) {
            if (lidar_hit_confirmed_) {
                std::cout << "\033[1;35m[BT] -> FALLBACK: LiDAR Strike! Relocating rock on map...\033[0m\n";
                
                // ACTIVE PERCEPTION RELOCATION
                // Because the mast is currently pointing directly at the rock, 
                // the rock lies straight ahead on the camera_link's X-axis!
                geometry_msgs::msg::PointStamped point_sensor;
                point_sensor.header.frame_id = "camera_link"; 
                point_sensor.header.stamp = last_scan_time_; // Get the rotated mast TF
                point_sensor.point.x = refined_distance_;    // Straight ahead
                point_sensor.point.y = 0.0;                  // Perfectly centered
                point_sensor.point.z = 0.0;

                try {
                    // TF2 automatically accounts for the mast's current yaw and tilt!
                    auto point_odom = tf_buffer_->transform(point_sensor, "odom", tf2::durationFromSec(0.1));
                    
                        confirmed_obstacles_->push_back({point_odom.point.x, point_odom.point.y});
                        // publish map with highly accurate coordinate
                        publishSemanticMap();

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN(ros_node_->get_logger(), "TF2 Error during relocation: %s", ex.what());
                }

                publishMastCommand(0.0, 0.0); 
                waiting_for_scan_ = false;
                is_looking_down_ = false;
            } 
            else if ((ros_node_->now() - scan_start_time_).seconds() > 3.0) { 
                std::cout << "\033[1;33m[BT] -> FALLBACK: LiDAR missed the target (Timeout). Resetting...\033[0m\n";
                publishMastCommand(0.0, 0.0);
                // deleting the false rock
                waiting_for_scan_ = false;
                is_looking_down_ = false;
            }
            return BT::NodeStatus::RUNNING;
        }
        
        if (!is_looking_down_ && !waiting_for_scan_ && 
            std::abs(current_tilt_) < 0.03 && 
            std::abs(current_yaw_) < 0.03) {
            
            std::cout << "\033[1;32m[BT] -> FALLBACK: Mast reset. Resuming mission.\033[0m\n";
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        publishMastCommand(0.0, 0.0); 
    }
    
    void publishMastCommand(double yaw_angle, double tilt_angle) {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {"mast_p_joint", "mast_02_joint", "mast_cameras_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        point.positions = {current_mast_p_, yaw_angle, tilt_angle}; 
        point.time_from_start.sec = 1; 
        traj_msg.points.push_back(point);
        joint_pub_->publish(traj_msg); 
    }
};

// Wait For Next Task
class WaitForNextTask : public BT::SyncActionNode {
public:
    WaitForNextTask(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "[BT] Task complete. Waiting for next instruction..." << std::endl;
        std::cout << "------------------------------------------------" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};


int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("bt_autonomy_node");

    //sim time from gazebo as a real time of simulation
    ros_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    ros_node->declare_parameter<double>("tau_1", 0.3);
    ros_node->declare_parameter<double>("tau_2", 0.75);
    ros_node->declare_parameter<double>("var_track", 0.02);
    ros_node->declare_parameter<double>("d_safe", 1.5);
    ros_node->declare_parameter<double>("t_react", 0.5);

    BT::BehaviorTreeFactory factory;

    auto shared_memory = std::make_shared<std::vector<std::pair<double, double>>>();

    //register custom nodes
    // nodes with shared memory blackboard!!
    factory.registerBuilder<RiskAssessment>("RiskAssessment", 
        [ros_node, shared_memory](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<RiskAssessment>(name, config, ros_node, shared_memory);
        });

    factory.registerBuilder<IsDanger>("IsDanger", 
        [ros_node, shared_memory](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsDanger>(name, config, ros_node, shared_memory);
        });

    factory.registerBuilder<IsPlanning>("IsPlanning", 
        [ros_node, shared_memory](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsPlanning>(name, config, ros_node, shared_memory);
        });

    factory.registerBuilder<ReactToObstacle>("ReactToObstacle", 
        [ros_node, shared_memory](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ReactToObstacle>(name, config, ros_node, shared_memory);
        });


    // Nodes requiring only ROS
    factory.registerBuilder<EmergencyStop>("EmergencyStop", 
        [ros_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<EmergencyStop>(name, config, ros_node);
        });

    factory.registerBuilder<FollowTrajectory>("FollowTrajectory", 
        [ros_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<FollowTrajectory>(name, config, ros_node);
        });

    factory.registerBuilder<CheckMissionObjective>("CheckMissionObjective", 
        [ros_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<CheckMissionObjective>(name, config, ros_node);
        });    
    factory.registerBuilder<IsMonitor>("IsMonitor", 
        [ros_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsMonitor>(name, config, ros_node);
        });    

    // Standard BT Nodes
    factory.registerNodeType<ExecuteTask>("ExecuteTask");
    factory.registerNodeType<WaitForNextTask>("WaitForNextTask");

    //timer for tick measurement
    auto tick_time_pub = ros_node->create_publisher<std_msgs::msg::Float64>("/bt/tick_time_ms", 10);

    // load xml
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_autonomy_pkg");
    
    // path
    std::string xml_path = package_share_directory + "/behavior_trees/unc_ext_bt.xml"; 

    auto tree = factory.createTreeFromFile(xml_path);
    std::cout << "\033[1;32m--- Risk-Based Behavior Tree Started ---\033[0m" << std::endl;

    //execution loop
    std_msgs::msg::Float64 tick_msg;
    // rclcpp::Rate instead of manual sleep_for to ensure a steady 100ms tick rate, dynamically adjusting for compute time.
    rclcpp::Rate rate(10.0); 
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        auto start_time = std::chrono::steady_clock::now();
        tree.tickExactlyOnce();      
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> tick_duration = end_time - start_time;
        tick_msg.data = tick_duration.count();
        tick_time_pub->publish(tick_msg);
        rate.sleep(); 
    }

    rclcpp::shutdown();
    return 0;
}
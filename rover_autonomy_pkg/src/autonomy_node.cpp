#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
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

// Check Mission Objective

class CheckMissionObjective : public BT::SyncActionNode {
public:
    CheckMissionObjective(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::OutputPort<std::string>("goal_output") }; }
    BT::NodeStatus tick() override {
        //std::cout << "[BT] Checking Mission... Assigning Waypoint: Alpha" << std::endl;
        setOutput("goal_output", "Waypoint_Alpha");
        return BT::NodeStatus::SUCCESS;
    }
};

// Execute Task
class ExecuteTask : public BT::SyncActionNode {
public:
    explicit ExecuteTask(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "[BT] Task execution..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }


}; 

// Obstacle detection (The Perception Condition)
class CheckObstacle : public BT::ConditionNode {
private:
    rclcpp::Node::SharedPtr perception_check_node_;
    rclcpp::Subscription<rover_interfaces::msg::PerceptionResultArray>::SharedPtr sub_;
    rover_interfaces::msg::PerceptionResultArray::SharedPtr last_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    //neccessary tf2 objects for odometry
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Spatial Memory (A list of X,Y coordinates we have already inspected)
    std::vector<std::pair<double, double>> confirmed_obstacles_;
    const double MAST_HEIGHT = 1.236;
    bool is_path_blocked_ = false;

public:
    CheckObstacle(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
        : BT::ConditionNode(name, config), perception_check_node_(ros_node) 
    {
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(perception_check_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        //init point_c sub
        point_cloud_pub_ = perception_check_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_obstacles",10);

        sub_ = perception_check_node_->create_subscription<rover_interfaces::msg::PerceptionResultArray>(
            "/perception/obstacle_info", 10,
            [this](const rover_interfaces::msg::PerceptionResultArray::SharedPtr msg) {
                this->last_msg_ = msg;

                //this->processBackgroundScan(msg);
            });
    }

    static BT::PortsList providedPorts() 
    {
        return 
        {
            BT::OutputPort<double>("target_x"),
            BT::OutputPort<double>("target_y"),
            BT::OutputPort<double>("target_tilt"),
            BT::OutputPort<double>("target_yaw"),
            BT::OutputPort<double>("target_distance"),
        };
    }

    // --- BEHAVIOR TREE LOGIC ---
    BT::NodeStatus tick() override {
        if (!last_msg_ || !last_msg_->obstacle_detected) {
            return BT::NodeStatus::SUCCESS;
        }

        bool trigger_inspection = false;

        for (const auto& rock : last_msg_->detections) {
            
            // 1. Calculate the Map (X,Y) coordinate using the perfect RGBD distance!
            double rel_x = rock.distance_meters * std::cos(rock.theta_angle);
            double rel_y = rock.distance_meters * std::sin(rock.theta_angle);

            geometry_msgs::msg::PointStamped point_sensor;
            point_sensor.header.frame_id = "camera_link"; 
            point_sensor.header.stamp = rclcpp::Time(0);
            point_sensor.point.x = rel_x;
            point_sensor.point.y = rel_y;
            point_sensor.point.z = 0.0;

            try {
                auto point_odom = tf_buffer_->transform(point_sensor, "odom", tf2::durationFromSec(0.0));
                double global_x = point_odom.point.x;
                double global_y = point_odom.point.y;

                // 2. Check Spatial Memory (Have we scanned this area yet?)
                bool already_known = false;
                for (const auto& known : confirmed_obstacles_) {
                    if (std::hypot(global_x - known.first, global_y - known.second) < 0.75) { 
                        already_known = true; break; 
                    }
                }

                // 3. ZONE CLASSIFICATION LOGIC
                if (!already_known) {                 
                    
                    if (rock.distance_meters > 2.0 &&  rock.distance_meters <= 10.0) {
                        // ZONE B: PLANNING ZONE (> 2.0m)
                        // Add to map quietly, do not stop the rover.
                        std::cout << "\033[1;36m[BT] Planning Zone Rock mapped at (X: " << global_x << ", Y: " << global_y << ")\033[0m\n";
                        confirmed_obstacles_.push_back({global_x, global_y});
                        publishSemanticMap();
                    } 
                    else if (rock.distance_meters > 10.0)
                    {
                        // ZONE C: MONITORING ZONE
                        // obstacles too far to include on the map
                        std::cout << "\033[1;31m[BT] Monitor Zone Rock detected at (X: " << global_x << ", Y: " << global_y << "\033[0m\n";
                    }
                    
                    else {
                        // ZONE A: DANGER / SCIENCE ZONE (< 2.0m)
                        // It is too close! We must stop and use the LiDAR!
                        std::cout << "\033[1;31m[BT] Danger Zone Rock at (X: " << global_x << ", Y: " << global_y << ")! Halting Rover for LiDAR Scan!\033[0m\n";
                        
                        // We set the target angle to exactly center the rock for the LiDAR
                        setOutput("target_tilt", rock.phi_angle); 
                        setOutput("target_yaw", rock.theta_angle);
                        setOutput("target_distance", rock.distance_meters);
                        // We map it so we don't scan it twice
                        confirmed_obstacles_.push_back({global_x, global_y});
                        publishSemanticMap();
                        
                        trigger_inspection = true;

                        break;
                    }
                }
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(perception_check_node_->get_logger(), "TF2 Error: %s", ex.what());
            }
        }
        
        // 4. Trigger the physical behavior
        if (trigger_inspection) { 
            return BT::NodeStatus::FAILURE; // This HALTS the wheels and tilts the mast!
        }
        
        return BT::NodeStatus::SUCCESS; // Keep driving!
    }

    void publishSemanticMap() {
        auto cloud = sensor_msgs::msg::PointCloud2();
        cloud.header.frame_id = "odom";
        cloud.header.stamp = perception_check_node_->now();

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(confirmed_obstacles_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto& obs : confirmed_obstacles_) {
            *iter_x = static_cast<float>(obs.first);
            *iter_y = static_cast<float>(obs.second);
            *iter_z = 0.0f;
            ++iter_x; ++iter_y; ++iter_z;
        }

        point_cloud_pub_->publish(cloud);
    }


};

// Follow Trajectory (Mock Async Action)
class FollowTrajectory : public BT::StatefulActionNode {
public:
    FollowTrajectory(const std::string& name, const BT::NodeConfig& config) 
        : BT::StatefulActionNode(name, config) {}
        
    static BT::PortsList providedPorts() { 
        return { BT::InputPort<std::string>("goal_input") }; 
    }

    BT::NodeStatus onStart() override {
        std::string goal;
        getInput("goal_input", goal);
        // This runs once when the rover starts driving
        //std::cout << "[BT] Following the trajectory to desired waypoint " << goal << "..." << std::endl;
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override {
        // This runs every tick while the rover is driving
        // We return RUNNING forever right now so we can test the obstacle scanning!
        return BT::NodeStatus::RUNNING; 
    }

    void onHalted() override {
        // This runs if the ReactiveSequence aborts the drive (e.g., rock in Danger Zone)
        std::cout << "\033[1;33m[BT] Driving halted by Obstacle Gatekeeper!\033[0m\n";
    }
};

// React to Obstacle
class ReactToObstacle : public BT::StatefulActionNode {
private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    //LiDAR
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Time scan_start_time_;

    double current_mast_p_ = 0.0;    // Joint 1
    double current_yaw_ = 0.0;       // Joint 2 (mast_02_joint) YAW
    double current_tilt_ = 0.0;      // Joint 3 (mast_cameras_joint)   PITCH

    double expected_distance_ = 0.0;
    double target_yaw_absolute_ = 0.0;
    double target_tilt_absolute_ = 0.0;
    
    bool is_looking_down_ = false;
    bool waiting_for_scan_ = false; 
    bool lidar_hit_confirmed_ = false; //Tracks when the laser strikes the rock

    //const std::string joint_name = "mast_cameras_joint";
    const std::string topic_name = "/mast_joint_trajectory_controller/joint_trajectory";

public:
    ReactToObstacle(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
            : BT::StatefulActionNode(name, config), ros_node_(ros_node)
    {
        joint_pub_ = ros_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(topic_name, 10);

        joint_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) 
            {
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (msg->name[i] == "mast_p_joint") {
                        this->current_mast_p_ = msg->position[i];
                    } else if (msg->name[i] == "mast_02_joint") {
                        this->current_yaw_ = msg->position[i];       // YAW
                    } else if (msg->name[i] == "mast_cameras_joint") {
                        this->current_tilt_ = msg->position[i];      // PITCH
                    }
                }
            });

        //Subscribe directly to the 1D Laser
        scan_sub_ = ros_node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                
                // Only process if the mast has reached its target tilt
                if (this->waiting_for_scan_ && !this->lidar_hit_confirmed_) {
                    
                    // 1. Find the center of the laser beam
                    int center_idx = msg->ranges.size() / 2;
                    
                    // 2. Create a narrow window (e.g., 20 rays left and right of center)
                    int search_window = 20; 
                    int start_idx = std::max(0, center_idx - search_window);
                    int end_idx = std::min((int)msg->ranges.size(), center_idx + search_window);

                    // 3. Check only the center rays
                    for (int i = start_idx; i < end_idx; i++) {
                        float range = msg->ranges[i];
                        
                        if (!std::isinf(range) && !std::isnan(range) && range > 0.1) {
                            // 4. THE MAGIC: Does the laser distance match the RGBD distance?
                            // We use a 0.4 meter tolerance for uneven rock surfaces
                            if (std::abs(range - this->expected_distance_) < 0.4) {
                                this->lidar_hit_confirmed_ = true;
                                break;
                            }
                        }
                    }
                }
            });
    }

    static BT::PortsList providedPorts() {
        return{
            BT::InputPort<double>("target_tilt"),
            BT::InputPort<double>("target_yaw"),
            BT::InputPort<double>("target_distance")
        };
    }

    BT::NodeStatus onStart() override 
    {
        double target_tilt = 0.0;
        double target_yaw = 0.0;
        if (!getInput("target_tilt", target_tilt) || 
            !getInput("target_yaw", target_yaw) || 
            !getInput("target_distance", expected_distance_)) {
            RCLCPP_WARN(ros_node_->get_logger(),"Missing BT variables!");
            return BT::NodeStatus::FAILURE;
        }
        double lidar_height_offset = 0.325;

        double parallax_correction = std::atan(lidar_height_offset / expected_distance_);

        target_tilt_absolute_ = current_tilt_+ target_tilt + parallax_correction;
        target_yaw_absolute_ = current_yaw_ + target_yaw;

        std::cout << "\033[1;35m[BT] -> FALLBACK: Panning to " << target_yaw_absolute_ 
                  << " rads, and Tilting to " << target_tilt_absolute_ << " ...\033[0m\n";
        publishMastCommand(target_yaw_absolute_,target_tilt_absolute_);
        
        is_looking_down_ = true;
        waiting_for_scan_ = false;
        lidar_hit_confirmed_ = false; // Reset the laser flag

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override 
    {
        // Phase 1: Have BOTH Pan and Tilt reached their targets?
        // Note: Tolerance is 0.02 rads (~1 degree) so the BT doesn't get stuck waiting for mathematical perfection
        if (is_looking_down_ && !waiting_for_scan_ && 
            std::abs(current_tilt_ - target_tilt_absolute_) < 0.05 &&
            std::abs(current_yaw_ - target_yaw_absolute_) < 0.05) {
            
            std::cout << "\033[1;36m[BT] -> FALLBACK: Target reached. Waiting for LiDAR strike...\033[0m\n";
            waiting_for_scan_ = true; // Turn on the laser listener!
            scan_start_time_ = ros_node_->now(); //measure time from gazebo
            return BT::NodeStatus::RUNNING;
        }

        // Phase 2: Did the LiDAR strike the rock? Look back to 0,0!
        if (waiting_for_scan_) {

            if (lidar_hit_confirmed_) {
                std::cout << "\033[1;35m[BT] -> FALLBACK: LiDAR Strike Confirmed! Resetting Mast...\033[0m\n";
                publishMastCommand(0.0, 0.0); 
                waiting_for_scan_ = false;
                is_looking_down_ = false;
            } 
            // timeout
            else if ((ros_node_->now() - scan_start_time_).seconds() > 3.0) { 
                std::cout << "\033[1;33m[BT] -> FALLBACK: LiDAR missed the target (Timeout). Resetting...\033[0m\n";
                publishMastCommand(0.0, 0.0); 
                waiting_for_scan_ = false;
                is_looking_down_ = false;
            }
            return BT::NodeStatus::RUNNING;
        }
        
        // Phase 3: Have BOTH Pan and Tilt reached 0.0 (straight ahead)?
        if (!is_looking_down_ && !waiting_for_scan_ && 
            std::abs(current_tilt_) < 0.03 && 
            std::abs(current_yaw_) < 0.03) {
            
            std::cout << "\033[1;32m[BT] -> FALLBACK: Mast reset. Resuming mission.\033[0m\n";
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        publishMastCommand(0.0, 0.0); // Safety reset both joints
    }

    void publishMastCommand(double yaw_angle, double tilt_angle) {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {"mast_p_joint", "mast_02_joint", "mast_cameras_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Ensure the order matches the names exactly!
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

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<CheckMissionObjective>("CheckMissionObjective");
    factory.registerNodeType<ExecuteTask>("ExecuteTask");
    factory.registerNodeType<FollowTrajectory>("FollowTrajectory");
    factory.registerNodeType<WaitForNextTask>("WaitForNextTask");



    // Register node

    BT::NodeBuilder builder_check_obstacle = [ros_node](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<CheckObstacle>(name, config, ros_node);
    };
    factory.registerBuilder<CheckObstacle>("CheckObstacle", builder_check_obstacle);

    BT::NodeBuilder builder_react_to_obstacle = [ros_node](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<ReactToObstacle>(name, config, ros_node);
    };
    factory.registerBuilder<ReactToObstacle>("ReactToObstacle", builder_react_to_obstacle);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_autonomy_pkg");
    std::string xml_path = package_share_directory + "/behavior_trees/classic_bt.xml";

    auto tree = factory.createTreeFromFile(xml_path);
    std::cout << "--- Behavior Tree Started ---" << std::endl;

    while (rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        tree.tickExactlyOnce();      
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    rclcpp::shutdown();
    return 0;
}
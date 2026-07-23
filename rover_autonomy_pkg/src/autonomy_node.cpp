#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>

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
#include "action_msgs/msg/goal_status.hpp"

#include <std_srvs/srv/empty.hpp>

// Check Mission Objective (first action node to set the goal)

class CheckMissionObjective : public BT::SyncActionNode {
private:
    bool goal_set_ = false; //flag
public:
    CheckMissionObjective(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::OutputPort<std::string>("goal_output") }; }
    
    BT::NodeStatus tick() override {
        // Only write to the port ONCE 
        if (!goal_set_) {
            setOutput("goal_output", "Waypoint_Alpha");
            goal_set_ = true;
        }
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

// Check obstacle, responisble for detection handle when the data is captured from ROS2 node
class CheckObstacle : public BT::ConditionNode {
private:
    // classes for ros2 rocks topic from preception habdle and to save in point cloud confirmed rock
    rclcpp::Node::SharedPtr perception_check_node_;
    rclcpp::Subscription<rover_interfaces::msg::PerceptionResultArray>::SharedPtr sub_;
    rover_interfaces::msg::PerceptionResultArray::SharedPtr last_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    // neccessary tf2 objects for odometry
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Memory for coordinates of rocks
    std::vector<std::pair<double, double>> confirmed_obstacles_;
    const double MAST_HEIGHT = 1.236;
    bool is_path_blocked_ = false;

public:
    CheckObstacle(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
        : BT::ConditionNode(name, config), perception_check_node_(ros_node) 
    {
        // init TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(perception_check_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        //init point cloud sub
        point_cloud_pub_ = perception_check_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/semantic_obstacles",10);

        sub_ = perception_check_node_->create_subscription<rover_interfaces::msg::PerceptionResultArray>(
            "/perception/obstacle_info", 10,
            [this](const rover_interfaces::msg::PerceptionResultArray::SharedPtr msg) {
                this->last_msg_ = msg;
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

    // Computation of this Obstacle check node
    BT::NodeStatus tick() override {
        if (!last_msg_ || !last_msg_->obstacle_detected) {
            return BT::NodeStatus::SUCCESS;
        }

        bool trigger_inspection = false;

        for (const auto& rock : last_msg_->detections) {
            
            // Calculate the Map (X,Y) coordinate based on angles (yaw only no Z cord calculation) and distance
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
                //global camera position
                double global_x = point_odom.point.x;
                double global_y = point_odom.point.y;

                // ========================================================
                // PLATT SCALING (CALIBRATED PROBABILITY) (not utilised in classic implementation) (Every obstacle is treated as the truth)
                // ========================================================
                double raw_confidence = rock.confidence; 
                
                double eps = 1e-7;
                double conf_clipped = std::max(eps, std::min(1.0 - eps, raw_confidence));
                double logit = std::log(conf_clipped / (1.0 - conf_clipped));

                // Logistic Regression Sigmoid Parameters
                double A = 1.68825; 
                double B = 0.02879;

                // Calibrated Probability (P_obstacle)
                double p_obstacle = 1.0 / (1.0 + std::exp(-(A * logit + B))); 
                // log to compare the confidence score to P_obstacle
                std::cout << "\033[1;34m[Math] YOLO: " << raw_confidence 
                          << " -> P_obs: " << p_obstacle << "\033[0m\n";
                // ========================================================

                // Check if the rock found was already processed
                bool already_known = false;
                for (const auto& known : confirmed_obstacles_) {
                    if (std::hypot(global_x - known.first, global_y - known.second) < 0.75) { 
                        already_known = true; break; 
                    }
                }
                //===============================================
                // ZONE CLASSIFICATION LOGIC
                //===============================================
                if (!already_known) {                 
                    
                    if (rock.distance_meters > 3.0 &&  rock.distance_meters <= 10.0) {
                        // PLANNING ZONE (> 3.0m)
                        // Adds the rock to the point cloud for the navigation to include it in path planning and actuation
                        std::cout << "\033[1;36m[BT] Planning Zone Rock mapped at (X: " << global_x << ", Y: " << global_y << ")\033[0m\n";
                        confirmed_obstacles_.push_back({global_x, global_y});
                        publishSemanticMap();
                    } 
                    else if (rock.distance_meters > 10.0)
                    {
                        // MONITORING ZONE
                        // obstacles too far to take action, but the rock will be still mointored becasue it is not added to the map
                        RCLCPP_INFO_THROTTLE(perception_check_node_->get_logger(), 
                                             *perception_check_node_->get_clock(), 
                                             2000, // 2000 milliseconds
                                             "[BT] Monitor Zone Rock detected at (X: %f, Y: %f)", global_x, global_y);
                    }
                    
                    else if (rock.distance_meters >1.3 && rock.distance_meters <=3.0)
                    {
                        // DANGER ZONE (<= 3.0m)
                        // Close to dangerous action, robot needs to take a critical action to avoid the rock
                        std::cout << "\033[1;31m[BT] Danger Zone Rock at (X: " << global_x << ", Y: " << global_y << ")! Halting Rover for LiDAR Scan!\033[0m\n";
                        
                        // Set the target angle of the mast to the center from the start
                        setOutput("target_tilt", rock.phi_angle); 
                        setOutput("target_yaw", rock.theta_angle);
                        setOutput("target_distance", rock.distance_meters);
                        // push to the map the obstacle
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
        
        // Trigger of the behavior - only for the danger zone
        if (trigger_inspection) { 
            return BT::NodeStatus::FAILURE; // WHEELS STOP, inspection needed
        }
        
        return BT::NodeStatus::SUCCESS; // Means that we can drive
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
private:
// needed classes for topic handle and navigation/control 
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_pose_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr brake_client_;
// dictionary of the waypoints (for now)
    std::map<std::string,std::pair<double,double>> waypoints_ {
        {"Waypoint_Alpha", {5.0, 0.0}},
        {"Waypoint_Beta", {10.0, -2.5}}
    };

    bool goal_reached_ = false;
    bool goal_responded_ = false;
    bool goal_accepted_ = false;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle_;

public:
    FollowTrajectory(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
        : BT::StatefulActionNode(name, config), ros_node_(ros_node) 
        {
            //creating clients for services of navigation and emergency braking
            nav_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ros_node_, "/navigate_to_pose");
            brake_client_ = ros_node_->create_client<std_srvs::srv::Empty>("/stop");
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

        // Take the goal from dictionary (for now Alpha only is the target waypoint)
        auto it = waypoints_.find(goal_str);
        if (it == waypoints_.end()) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Waypoint '%s' not found in dictionary!", goal_str.c_str());
            return BT::NodeStatus::FAILURE;
        }
        
        double target_x = it->second.first;
        double target_y = it->second.second;

        // Checking for NAV2 Server, if it was correclty init
        if (!nav_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Nav2 Action Server is offline! Cannot drive.");
            return BT::NodeStatus::FAILURE;
        }

        // Goal message
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        
        // goal parameters in map frame
        goal_msg.pose.header.frame_id = "map"; 
        goal_msg.pose.header.stamp = ros_node_->now();
        goal_msg.pose.pose.position.x = target_x;
        goal_msg.pose.pose.position.y = target_y;
        goal_msg.pose.pose.position.z = 0.0;
        
        // No orientation desired
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        // Callback for goal handle
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
                this->goal_responded_ = true;
                if (!goal_handle) {
                    std::cout << "\033[1;31m[BT] Nav2 Rejected the Goal!\033[0m\n";
                    this->goal_accepted_ = false;
                } else {
                    std::cout << "\033[1;32m[BT] Goal successfully passed\033[0m\n";
                    this->goal_accepted_ = true;
                    this->goal_handle_ = goal_handle; 
                }
            };

        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
                // finish goal reached
                (void)result;
                this->goal_reached_ = true; 
            };

        std::cout << "\033[1;36m[BT] Sending Nav2 Goal: " << goal_str 
                  << " (X: " << target_x << ", Y: " << target_y << ")...\033[0m\n";
        
        // asynchronous send of the goal
        nav_pose_client_->async_send_goal(goal_msg, send_goal_options);
        
        goal_responded_ = false;
        goal_accepted_ = false;
        goal_reached_ = false; // Reset the flag
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override {
        if (!goal_responded_) {
            return BT::NodeStatus::RUNNING; 
        }

        // sth went wrong, goal not accepted, so the server didnt work
        if (goal_responded_ && !goal_accepted_) {
            return BT::NodeStatus::FAILURE; 
        }

        // result callback check
        if (goal_reached_) {
            auto status = goal_handle_->get_status();
            if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                std::cout << "\033[1;32m[BT] Waypoint Reached Successfully!\033[0m\n";
                return BT::NodeStatus::SUCCESS;
            } else {
                std::cout << "\033[1;31m[BT] Nav2 Failed to reach the goal.\033[0m\n";
                return BT::NodeStatus::FAILURE;
            }
        }

        // if nothing wrong happened or goal not reached, the node is running still
        return BT::NodeStatus::RUNNING; 
    }

    void onHalted() override {
        // This runs if the ReactiveSequence aborts the drive (e.g., rock in Danger Zone)
        std::cout << "\033[1;33m[BT] Rover stopped, danger detected!\033[0m\n";
        
        if (brake_client_->wait_for_service(std::chrono::milliseconds(50))) {
            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            brake_client_->async_send_request(req);
        } else {
            RCLCPP_WARN(ros_node_->get_logger(), "/stop service not available!");
        }

        //Cancel the Nav2 Goal
        if (nav_pose_client_ && goal_handle_) {
            nav_pose_client_->async_cancel_goal(goal_handle_);
        }
        
        goal_handle_.reset();
    }
};

// React to Obstacle
class ReactToObstacle : public BT::StatefulActionNode {
private:
// joint sub and pub to react according to the zone
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
    bool lidar_hit_confirmed_ = false; // if the lidar hit the rock
    const std::string topic_name = "/mast_joint_trajectory_controller/joint_trajectory"; // topic for traj control to send the desired joint angle

public:
    ReactToObstacle(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr ros_node) 
            : BT::StatefulActionNode(name, config), ros_node_(ros_node)
    {
        // create pub and sub
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

        //Subscribe to the 1D LIDAR topic
        scan_sub_ = ros_node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                
                // to check if the target tilt was reached
                if (this->waiting_for_scan_ && !this->lidar_hit_confirmed_) {
                    
                    // laser scan of the rock only with the center laser -> so the lidar centered to the rock
                    int center_idx = msg->ranges.size() / 2;
                    
                    // narrow window of rays
                    int search_window = 20; 
                    int start_idx = std::max(0, center_idx - search_window);
                    int end_idx = std::min((int)msg->ranges.size(), center_idx + search_window);

                    // check the center ray
                    for (int i = start_idx; i < end_idx; i++) {
                        float range = msg->ranges[i];
                        
                        if (!std::isinf(range) && !std::isnan(range) && range > 0.1) {
                            // if the distance of LIDAR scan does not match the distance of RGBD (with tolerence 0.4 m) then the actuation is still to be goin for the tilt
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
        double target_yaw = 0.0;
        if (!getInput("target_tilt", target_tilt) || 
            !getInput("target_yaw", target_yaw) || 
            !getInput("target_distance", expected_distance_)) {
            RCLCPP_WARN(ros_node_->get_logger(),"Missing BT variables!");
            return BT::NodeStatus::FAILURE;
        }
        // needed to be included for the mast tilt. If very close the tilt correction will be bigger is further it will be small
        double lidar_height_offset = 0.325;
        double parallax_correction = std::atan(lidar_height_offset / expected_distance_);
        //final target pitch (tilt) and yaw for he action of active perception checking
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
        // yaw and pitch reached with a tolerence
        if (is_looking_down_ && !waiting_for_scan_ && 
            std::abs(current_tilt_ - target_tilt_absolute_) < 0.05 &&
            std::abs(current_yaw_ - target_yaw_absolute_) < 0.05) {
            
            std::cout << "\033[1;36m[BT] -> FALLBACK: Target reached. Waiting for LiDAR strike...\033[0m\n";
            waiting_for_scan_ = true; // Waiting for scan
            scan_start_time_ = ros_node_->now(); //measure time from gazebo (to handle the time, for critical action needed)
            return BT::NodeStatus::RUNNING;
        }

        // LIDAR scan the rock, if waiting_for_scan true then the system waits for the lidar to scan (by the time the yaw and pitch will be reached)
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
                publishMastCommand(0.0, 0.0); // After the correct obstacle scan, command to reset the tilt
                waiting_for_scan_ = false;
                is_looking_down_ = false;
            }
            return BT::NodeStatus::RUNNING;
        }
        
        // waiting for mast to reset its angles
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
    // function to publish MAST joint positions to the controller
    void publishMastCommand(double yaw_angle, double tilt_angle) {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {"mast_p_joint", "mast_02_joint", "mast_cameras_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // joints of the camera second and third are pitch, yaw
        point.positions = {current_mast_p_, yaw_angle, tilt_angle}; 
        
        point.time_from_start.sec = 1; 
        traj_msg.points.push_back(point);
        joint_pub_->publish(traj_msg); // publish the joint angles
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
    // defining the nodes
    factory.registerNodeType<CheckMissionObjective>("CheckMissionObjective");
    factory.registerNodeType<ExecuteTask>("ExecuteTask");
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

    BT::NodeBuilder builder_follow_traj = [ros_node](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<FollowTrajectory>(name, config, ros_node);
    };
    factory.registerBuilder<FollowTrajectory>("FollowTrajectory", builder_follow_traj);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_autonomy_pkg");
    std::string xml_path = package_share_directory + "/behavior_trees/classic_bt.xml";

    auto tree = factory.createTreeFromFile(xml_path);
    std::cout << "--- Behavior Tree Started ---" << std::endl;
    // IMPORTANT TO USE THE SIMULATION TIME AS REFERENCE FOR ACTIONS NOT THE COMPUTER TIME!!!!
    ros_node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    auto sleep_time = rclcpp::Duration::from_seconds(0.1);
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        tree.tickExactlyOnce();      
        ros_node->get_clock()->sleep_for(sleep_time);
    }

    rclcpp::shutdown();
    return 0;
}
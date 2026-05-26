#include <iostream>
#include <chrono>
#include <thread>
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_interfaces/msg/perception_result_array.hpp"

// Check Mission Objective

class CheckMissionObjective : public BT::SyncActionNode {
public:
    CheckMissionObjective(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::OutputPort<std::string>("goal_output") }; }
    BT::NodeStatus tick() override {
        std::cout << "[BT] Checking Mission... Assigning Waypoint: Alpha" << std::endl;
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

// Is Path Clear (The Perception Condition)
class IsPathClear : public BT::ConditionNode {
private:
    rclcpp::Node::SharedPtr perception_check_node_;
    rclcpp::Subscription<rover_interfaces::msg::PerceptionResultArray>::SharedPtr sub_;
    bool is_path_blocked_ = false;

public:
    // We pass the ROS 2 Node into the constructor so we can create a subscriber
    IsPathClear(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr perception_check_node_) 
        : BT::ConditionNode(name, config), perception_check_node_(perception_check_node_) 
    {
        sub_ = perception_check_node_->create_subscription<rover_interfaces::msg::PerceptionResultArray>(
            "/perception/obstacle_info", 10,
            [this](const rover_interfaces::msg::PerceptionResultArray::SharedPtr msg) {
                // Instantly update our boolean whenever Python sends a message
                this->is_path_blocked_ = msg->obstacle_detected;
            });
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        if (is_path_blocked_) {
            std::cout << "\033[1;31m[BT] PERCEPTION: Path is BLOCKED! Returning FAILURE.\033[0m" << std::endl;
            return BT::NodeStatus::FAILURE; // This will trigger the Fallback!
        }
        
        std::cout << "\033[1;32m[BT] Perception: Path is CLEAR.\033[0m" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// Follow Trajectory
class FollowTrajectory : public BT::SyncActionNode {
public:
    FollowTrajectory(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("goal_input") }; }
    BT::NodeStatus tick() override {
        std::string goal;
        getInput("goal_input", goal);
        std::cout << "[BT] Following the trajectory to desired waypoint" << goal << "..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return BT::NodeStatus::SUCCESS;
    }
};

// React to Obstacle
class ReactToObstacle : public BT::SyncActionNode {
public:
    explicit ReactToObstacle(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "[BT] OBSTACLE DETECTED! Reacting to found obstacle..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }


};

// Wait For Next Task
class WaitForNextTask : public BT::SyncActionNode {
public:
    WaitForNextTask(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "[BT] Task complete. Waiting for next instruction..." << std::endl;
        std::cout << "------------------------------------------------" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return BT::NodeStatus::SUCCESS;
    }
};


int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("bt_autonomy_node");

    BT::BehaviorTreeFactory factory;

    // Register all our custom nodes with the factory
    factory.registerNodeType<CheckMissionObjective>("CheckMissionObjective");
    factory.registerNodeType<ExecuteTask>("ExecuteTask");
    factory.registerNodeType<FollowTrajectory>("FollowTrajectory");
    factory.registerNodeType<ReactToObstacle>("ReactToObstacle");
    factory.registerNodeType<WaitForNextTask>("WaitForNextTask");


    BT::NodeBuilder builder_is_path_clear = [ros_node](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<IsPathClear>(name, config, ros_node);
    };
    factory.registerBuilder<IsPathClear>("IsPathClear", builder_is_path_clear);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rover_autonomy_pkg");
    
    // 2. Combine it with your subfolder and XML file name
    std::string xml_path = package_share_directory + "/behavior_trees/classic_bt.xml";

    std::cout << "[BT] Loading XML from: " << xml_path << std::endl;

    // 3. Pass the absolute path to the factory
    auto tree = factory.createTreeFromFile(xml_path);

    std::cout << "--- Behavior Tree Started ---" << std::endl;

    while (rclcpp::ok()) {
        rclcpp::spin_some(ros_node); // Let ROS 2 update the subscribers
        tree.tickExactlyOnce();      // Let the Behavior Tree execute one logic step
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    rclcpp::shutdown();

    return 0;
}
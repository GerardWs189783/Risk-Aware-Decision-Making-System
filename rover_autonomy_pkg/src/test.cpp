#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

// 1. Create a Custom Behavior Tree Node
class CheckSystems : public BT::SyncActionNode
{
public:
    CheckSystems(const std::string& name) : BT::SyncActionNode(name, {}) {}

    // You must override the tick() function
    BT::NodeStatus tick() override
    {
        std::cout << "[BT NODE] -> Systems Nominal! Rover Autonomy Package is ALIVE." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// 2. The Main ROS 2 Execution
int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("autonomy_test_node");
    
    RCLCPP_INFO(node->get_logger(), "Starting ROS 2 Node. Booting Behavior Tree...");

    // Initialize the Behavior Tree Factory
    BT::BehaviorTreeFactory factory;

    // Register our custom C++ node into the factory
    factory.registerNodeType<CheckSystems>("CheckSystems");

    // Create a tiny XML tree directly in the code for testing
    std::string xml_text = R"(
     <root BTCPP_format="4">
         <BehaviorTree ID="MainTree">
            <Sequence>
                <CheckSystems name="SystemCheck_1"/>
            </Sequence>
         </BehaviorTree>
     </root>
    )";

    // Build the tree and tick it
    auto tree = factory.createTreeFromText(xml_text);
    tree.tickWhileRunning();

    RCLCPP_INFO(node->get_logger(), "Behavior Tree execution complete. Shutting down.");
    
    // Cleanup ROS 2
    rclcpp::shutdown();
    return 0;
}
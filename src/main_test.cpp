/*
main_test.cpp
Entry point for UNDEAD_TEST mission mode.
*/

#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/node_with_mode.hpp>

#include <iostream>
#include <string>

#include <undead_test.hpp>

using TestModeExecutorNode = px4_ros2::NodeWithModeExecutor<executeUndeadTestMode, UndeadTestMode>;

static const std::string kNodeName = "mission_test";

static bool hasParamsFileArg(int argc, char * argv[])
{
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--params-file") {
            return true;
        }
    }
    return false;
}

int main(int argc, char * argv[])
{
    if (!hasParamsFileArg(argc, argv)) {
        std::cerr << "Missing required config file. Start with --ros-args --params-file <config.yaml>\n";
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestModeExecutorNode>(kNodeName, true));
    rclcpp::shutdown();
    return 0;
}

/*
mainUndead.cpp
Written by Bijan Jourabchi
University of Colorado Boulder
Undead Reckoning
*/

#include "rclcpp/rclcpp.hpp"
#include <undead.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <iostream>
#include <string>

using DrawModeExecutorNode = px4_ros2::NodeWithModeExecutor<executeMOFlightMode, MOFlightMode>;

static const std::string kNodeName = "mission";

static bool hasParamsFileArg(int argc, char * argv[])
{
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--params-file") {
            return true;
        }
    }
    return false;
}

int main(int argc, char * argv[]) {
    if (!hasParamsFileArg(argc, argv)) {
        std::cerr << "Missing required config file. Start fly_MO with --ros-args --params-file <config.yaml>\n";
        return 1;
    }

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DrawModeExecutorNode>(kNodeName, true));
    rclcpp::shutdown();
    return 0;
}

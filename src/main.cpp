/*
main.cpp
Written by Bijan Jourabchi
University of Colorado Boulder
Undead Reckoning
*/

#include "rclcpp/rclcpp.hpp"
#include <mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using DrawModeExecutorNode = px4_ros2::NodeWithModeExecutor<executeUAS, UASFlightMode>;

static const std::string kNodeName = "NAME";

int main(int argc, char * argv[]) {
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DrawModeExecutorNode>(kNodeName, true));
    rclcpp::shutdown();
    return 0;
}
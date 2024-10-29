#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/device_management.h>
#include <rv2_interfaces/qos.h>


namespace rv2_interfaces
{

class VehicleServiceNode : public DevManageNode, public QoSNode// , public SafetyNode, public TimeSyncNode
{
public:
    VehicleServiceNode(const std::string& nodeName, const rclcpp::NodeOptions & options) : 
        DevManageNode(nodeName, options), 
        QoSNode(nodeName, options), 
        rclcpp::Node(nodeName, options) {}
};

}

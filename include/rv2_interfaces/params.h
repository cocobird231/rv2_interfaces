// #ver=0.1.1
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/utils.h>

namespace rv2_interfaces
{

/* The GenericParams class defines the generic settings for ros2 node and extended services, e.g., 
 * time sync service, safety service and qos service, etc..
 * The configure file can be stored under launch directory in each package.
 */
class GenericParams : public rclcpp::Node
{
public:
    bool useSimTime = false;
    std::string nodeName = "node";
    std::string id = "0";
    std::string devManageService = "";// devmanage_0
    std::string devInterface = "";
    std::string qosService = "";// qos_0
    std::string qosDirPath = "";
    std::string safetyService = "";// safety_0
    std::string timesyncService = "";// timesync_0
    double timesyncPeriod_ms = 10000.0;
    double timesyncAccuracy_ms = 2.0;
    bool timesyncWaitService = false;

public:
    GenericParams(std::string nodeName) : Node(nodeName)
    {
        this->getParam("nodeName", this->nodeName, this->nodeName, "nodeName: ", false);
        this->getParam("id", id, id, "Node ID: ", false);
        this->getParam("use_sim_time", useSimTime, useSimTime, "Use sim time: ", false, true);
        this->getParam("devManageService", devManageService, devManageService, "Device manage service: ", false);
        this->getParam("devInterface", devInterface, devInterface, "Device interface: ", false);
        this->getParam("qosService", qosService, qosService, "QoS service: ", false);
        this->getParam("qosDirPath", qosDirPath, qosDirPath, "QoS directory path: ", false);
        this->getParam("safetyService", safetyService, safetyService, "Safety service: ", false);
        this->getParam("timesyncService", timesyncService, timesyncService, "Time sync service: ", false);
        this->getParam("timesyncPeriod_ms", timesyncPeriod_ms, timesyncPeriod_ms, "Time sync period (ms): ", false);
        this->getParam("timesyncAccuracy_ms", timesyncAccuracy_ms, timesyncAccuracy_ms, "Time sync accuracy (ms): ", false);
        this->getParam("timesyncWaitService", timesyncWaitService, timesyncWaitService, "Time sync wait service: ", false);
    }

    template<typename T>
    void getParam(std::string paramName, T defValue, T & outVal, std::string log_info, bool dynamic, bool noDeclare = false)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = !dynamic;

        if (!noDeclare)
            declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

        if (!get_parameter(paramName, outVal))
        {
            RCLCPP_WARN_STREAM(get_logger(),
                "The parameter '"
                << paramName
                << "' is not available or is not valid, using the default value: "
                << rv2_interfaces::CvtSString(defValue).str());
        }

        if (!log_info.empty())
        {
            RCLCPP_INFO_STREAM(get_logger(), log_info << rv2_interfaces::CvtSString(outVal).str());
        }
    }
};

}// namespace rv2_interfaces

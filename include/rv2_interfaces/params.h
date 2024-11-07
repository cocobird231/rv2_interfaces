// #ver=0.2.1
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/utils.h>

namespace rv2_interfaces
{

struct GenericParams
{
    std::string devManageService = "";
    std::string devInterface = "";
    double procStatusRegPeriod_ms = -1.0;
    std::string qosService = "";
    std::string qosDirPath = "";
    std::string safetyService = "";
    std::string timesyncService = "";
    double timesyncPeriod_ms = -1.0;
    double timesyncAccuracy_ms = -1.0;
    bool timesyncWaitService = false;
};

GenericParams GetGenericParams(rclcpp::Node::SharedPtr node)
{
    GenericParams ret;

    GetParam(node, "devManageService", ret.devManageService, ret.devManageService, "devManageService: ", false);
    GetParam(node, "devInterface", ret.devInterface, ret.devInterface, "devInterface: ", false);
    GetParam(node, "procStatusRegPeriod_ms", ret.procStatusRegPeriod_ms, ret.procStatusRegPeriod_ms, "procStatusRegPeriod_ms: ", false);
    GetParam(node, "qosService", ret.qosService, ret.qosService, "qosService: ", false);
    GetParam(node, "qosDirPath", ret.qosDirPath, ret.qosDirPath, "qosDirPath: ", false);
    GetParam(node, "safetyService", ret.safetyService, ret.safetyService, "safetyService: ", false);
    GetParam(node, "timesyncService", ret.timesyncService, ret.timesyncService, "timesyncService: ", false);
    GetParam(node, "timesyncPeriod_ms", ret.timesyncPeriod_ms, ret.timesyncPeriod_ms, "timesyncPeriod_ms: ", false);
    GetParam(node, "timesyncAccuracy_ms", ret.timesyncAccuracy_ms, ret.timesyncAccuracy_ms, "timesyncAccuracy_ms: ", false);
    GetParam(node, "timesyncWaitService", ret.timesyncWaitService, ret.timesyncWaitService, "timesyncWaitService: ", false);
    return ret;
}

void SetGenericParams(rclcpp::Node::SharedPtr node, GenericParams params)
{
    SetParam(node, "devManageService", params.devManageService, false);
    SetParam(node, "devInterface", params.devInterface, false);
    SetParam(node, "procStatusRegPeriod_ms", params.procStatusRegPeriod_ms, false);
    SetParam(node, "qosService", params.qosService, false);
    SetParam(node, "qosDirPath", params.qosDirPath, false);
    SetParam(node, "safetyService", params.safetyService, false);
    SetParam(node, "timesyncService", params.timesyncService, false);
    SetParam(node, "timesyncPeriod_ms", params.timesyncPeriod_ms, false);
    SetParam(node, "timesyncAccuracy_ms", params.timesyncAccuracy_ms, false);
    SetParam(node, "timesyncWaitService", params.timesyncWaitService, false);
}

}// namespace rv2_interfaces

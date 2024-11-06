#pragma once
#include <iostream>
#include <iomanip>
#include <sstream>

#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <vector>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/interactive_service.h>
#include <rv2_interfaces/service.h>
#include <rv2_interfaces/timer.h>
#include <rv2_interfaces/utils.h>

#include <rv2_interfaces/srv/dev_info_req.hpp>
#include <rv2_interfaces/srv/dev_manage_server.hpp>
#include <rv2_interfaces/srv/node_addr_reg.hpp>
#include <rv2_interfaces/srv/node_addr_req.hpp>
#include <rv2_interfaces/srv/interactive_service_reg.hpp>
#include <rv2_interfaces/srv/proc_status_reg.hpp>
#include <rv2_interfaces/srv/proc_status_req.hpp>

#include <fstream>
#include <nlohmann/json.hpp>

#define DevManageNode_ISrvName(serviceName) (std::string)serviceName + "_devmanage_isrv"
#define DevManageServer_ProcStatusRegSrvName(serviceName) (std::string)serviceName + "_ps_Reg"
#define DevManageServer_NodeAddrRegSrvName(serviceName) (std::string)serviceName + "_nodeaddr_Reg"
#define DevManageServer_NodeAddrReqSrvName(serviceName) (std::string)serviceName + "_nodeaddr_Req"
#define DevManageServer_DevInfoReqSrvName(serviceName) (std::string)serviceName + "_devinfo_Req"
#define DevManageServer_DevManageServerSrvName(serviceName) (std::string)serviceName

// #define RV2_USE_IPV6

using namespace std::chrono_literals;

/**
 * ================================================================================================
 *                                  Rules for the Device Management Service
 * ================================================================================================
 * 
 * ------------------------------------------------------------------------------------------------
 * DevManageNode:
 *      The class is used to register the device information to the DevManageServer, 
 *      and provide the procedure monitoring service.
 *      For nodes that need to join the DevManageServer, they should inherit the DevManageNode class.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <nodeName>_devmanage_isrv        : The InteractiveService service for the DevManageNode.
 *          <nodeName>_devmanage_isrv_Req    : The InteractiveService status request service for the DevManageNode.
 * ------------------------------------------------------------------------------------------------
 * DevManageServer:
 *      The Device Management Service is used to manage the device information and the procedure status, 
 *      and provide the node exit command to the DevManageNode.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <service_name>              : The DevManageServer control service.
 *          <service_name>_ps_Reg       : The procedure status registration service.
 *          <service_name>_nodeaddr_Reg : The node address registration service.
 *          <service_name>_nodeaddr_Req : The node address request service.
 *          <service_name>_devinfo_Req  : The device information request service.
 * ------------------------------------------------------------------------------------------------
 * InteractiveService Command:
 *      - "node_exit" : Exit the DevManageNode.
 *          Arguments:
 *                      "exit_code:<int status>"
 *                      "delay_ms:<int ms>"
 * ------------------------------------------------------------------------------------------------
 */



namespace rv2_interfaces
{

/**
 * ================================================================
 * ProcedureStatus, ProcedureMonitorProp and Functions
 * ================================================================
 */



/**
 * ProcedureStatus is used to describe the status of a procedure.
 * PROCEDURE_STATUS_OK: The procedure is OK. Set by the procedure itself.
 * PROCEDURE_STATUS_WARN: The procedure has some warnings but still response. Set by the procedure itself.
 * PROCEDURE_STATUS_ERROR: The procedure has some errors but still response. Set by the procedure itself.
 * PROCEDURE_STATUS_NO_RESPONSE: The procedure has no response. Set by the procedure status service.
 */
enum ProcedureStatus { PROCEDURE_STATUS_OK, PROCEDURE_STATUS_WARN, PROCEDURE_STATUS_ERROR, PROCEDURE_STATUS_NO_RESPONSE };

inline std::string GetProcedureStatusStr(ProcedureStatus status)
{
    switch (status)
    {
    case PROCEDURE_STATUS_OK:
        return "OK";
    case PROCEDURE_STATUS_WARN:
        return "WARN";
    case PROCEDURE_STATUS_ERROR:
        return "ERROR";
    case PROCEDURE_STATUS_NO_RESPONSE:
        return "NO_RESPONSE";
    default:
        return "UNKNOWN";
    }
}



struct ProcedureMonitorProp
{
    // Properties
    std::string procedureName;
    std::chrono::nanoseconds timeout_ns;

    // Status
    ProcedureStatus status;
    std::string statusMsg;
    std::chrono::system_clock::time_point lastResponseTime;
};



inline ProcedureMonitorProp CvtProcedureStatusMsg(const msg::ProcStatus& msg)
{
    ProcedureMonitorProp ret;
    ret.procedureName = msg.proc_name;
    ret.timeout_ns = std::chrono::milliseconds(msg.timeout_ns);
    ret.status = (ProcedureStatus)msg.status;
    ret.statusMsg = msg.message;
    ret.lastResponseTime = std::chrono::system_clock::time_point(std::chrono::nanoseconds(rclcpp::Time(msg.timestamp).nanoseconds()));
}



inline msg::ProcStatus CvtProcedureStatusMsg(const ProcedureMonitorProp& prop)
{
    msg::ProcStatus ret;
    ret.proc_name = prop.procedureName;
    ret.timeout_ns = prop.timeout_ns.count();
    ret.status = prop.status;
    ret.message = prop.statusMsg;
    ret.timestamp = rclcpp::Time(prop.lastResponseTime.time_since_epoch().count());
    return ret;
}



inline void AddHierarchicalPrint(HierarchicalPrint& hprint, int hierarchy, const msg::ProcStatus& msg)
{
    hprint.push(hierarchy, "[%s]", msg.proc_name.c_str());
    hprint.push(hierarchy + 1, "[timeout] %ld ns", msg.timeout_ns);
    hprint.push(hierarchy + 1, "[status] %s", GetProcedureStatusStr((ProcedureStatus)msg.status).c_str());
    hprint.push(hierarchy + 1, "[message] %s", msg.message.c_str());
}



inline void AddHierarchicalPrint(HierarchicalPrint& hprint, int hierarchy, const msg::NodeAddr& msg)
{
    hprint.push(hierarchy, "[%s]", msg.node_name.c_str());
    hprint.push(hierarchy + 1, "[hostname] %s", msg.hostname.c_str());
    hprint.push(hierarchy + 1, "[mac_addr] %s", msg.mac_addr.c_str());
    hprint.push(hierarchy + 1, "[ipv4_addr] %s", msg.ipv4_addr.c_str());
    hprint.push(hierarchy + 1, "[ipv6_addr] %s", msg.ipv6_addr.c_str());
}



inline void AddHierarchicalPrint(HierarchicalPrint& hprint, int hierarchy, const msg::DevInfo& msg)
{
    hprint.push(hierarchy, "[%s%s]", msg.node_addr.node_name.c_str(), msg.is_proc_status_expired ? " (Expired)" : "");
    hprint.push(hierarchy + 1, "[Node Address]");
    hprint.push(hierarchy + 2, "[hostname] %s", msg.node_addr.hostname.c_str());
    hprint.push(hierarchy + 2, "[mac_addr] %s", msg.node_addr.mac_addr.c_str());
    hprint.push(hierarchy + 2, "[ipv4_addr] %s", msg.node_addr.ipv4_addr.c_str());
    hprint.push(hierarchy + 2, "[ipv6_addr] %s", msg.node_addr.ipv6_addr.c_str());
    hprint.push(hierarchy + 1, "[Procedures]");
    for (const auto& p : msg.proc_status_vec)
    {
        AddHierarchicalPrint(hprint, hierarchy + 2, p);
    }
}



/**
 * ================================================================
 * NodeAddrIndicators, NodeAddrCompFunc and Functions
 * ================================================================
 */



struct NodeAddrIndicators
{
    bool node_name;
    bool hostname;
    bool ipv4_addr;
    bool ipv6_addr;
    bool mac_addr;

    NodeAddrIndicators() : node_name(false), hostname(false), ipv4_addr(false), ipv6_addr(false), mac_addr(false) {}
    NodeAddrIndicators(bool initFlag) : node_name(initFlag), hostname(initFlag), ipv4_addr(initFlag), ipv6_addr(initFlag), mac_addr(initFlag) {}
};



struct NodeAddrCompFunc
{
    std::function<bool(const std::string&, const std::string&)> node_name_func;
    std::function<bool(const std::string&, const std::string&)> hostname_func;
    std::function<bool(const std::string&, const std::string&)> ipv4_addr_func;
    std::function<bool(const std::string&, const std::string&)> ipv6_addr_func;
    std::function<bool(const std::string&, const std::string&)> mac_addr_func;
};



inline NodeAddrCompFunc make_node_addr_comp_func_same_profile()
{
    NodeAddrCompFunc ret;
    ret.node_name_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    ret.hostname_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    ret.ipv4_addr_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    ret.ipv6_addr_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    ret.mac_addr_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    return ret;
}



inline NodeAddrCompFunc make_node_addr_comp_func_same_device()
{
    NodeAddrCompFunc ret;
    ret.node_name_func = [](const std::string& s1, const std::string& s2) { return s1 != s2; };
    ret.hostname_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    ret.ipv4_addr_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    ret.ipv6_addr_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    ret.mac_addr_func = [](const std::string& s1, const std::string& s2) { return s1 == s2; };
    return ret;
}



inline NodeAddrCompFunc make_node_addr_comp_func_diff_device()
{
    NodeAddrCompFunc ret;
    ret.node_name_func = [](const std::string& s1, const std::string& s2) { return s1 != s2; };
    ret.hostname_func = [](const std::string& s1, const std::string& s2) { return s1 != s2; };
    ret.ipv4_addr_func = [](const std::string& s1, const std::string& s2) { return s1 != s2; };
    ret.ipv6_addr_func = [](const std::string& s1, const std::string& s2) { return s1 != s2; };
    ret.mac_addr_func = [](const std::string& s1, const std::string& s2) { return s1 != s2; };
    return ret;
}



inline NodeAddrIndicators CompNodeAddr(const msg::NodeAddr& d1, const msg::NodeAddr& d2, NodeAddrCompFunc compFunc)
{
    NodeAddrIndicators ret;
    ret.node_name = compFunc.node_name_func(d1.node_name, d2.node_name);
    ret.hostname = compFunc.hostname_func(d1.hostname, d2.hostname);
    ret.ipv4_addr = compFunc.ipv4_addr_func(d1.ipv4_addr, d2.ipv4_addr);
    ret.ipv6_addr = compFunc.ipv6_addr_func(d1.ipv6_addr, d2.ipv6_addr);
    ret.mac_addr = compFunc.mac_addr_func(d1.mac_addr, d2.mac_addr);
    return ret;
}



inline bool IsNodeAddrIntersect(const msg::NodeAddr& d1, const msg::NodeAddr& d2, NodeAddrIndicators& indicators)
{
    static NodeAddrCompFunc compFunc = make_node_addr_comp_func_same_profile();
    indicators = CompNodeAddr(d1, d2, compFunc);
    return indicators.node_name || 
            indicators.hostname || 
            indicators.ipv4_addr || 
#ifdef RV2_USE_IPV6
            indicators.ipv6_addr || 
#endif
            indicators.mac_addr;
}



inline bool IsNodeAddrSame(const msg::NodeAddr& d1, const msg::NodeAddr& d2, NodeAddrIndicators& indicators)
{
    static NodeAddrCompFunc compFunc = make_node_addr_comp_func_same_profile();
    indicators = CompNodeAddr(d1, d2, compFunc);
    return indicators.node_name && 
            indicators.hostname && 
            indicators.ipv4_addr && 
#ifdef RV2_USE_IPV6
            indicators.ipv6_addr && 
#endif
            indicators.mac_addr;
}



enum NodeAddrCompResult { DEVINFO_CONFLICT_NODE_NAME = -2, DEVINFO_CONFLICT, DEVINFO_SAME, DEVINFO_DIFFERENT };// -2: Node name conflict, -1: Conflict, 0: Same, 1: Different

inline NodeAddrCompResult CheckNodeAddrConflict(const msg::NodeAddr& d1, const msg::NodeAddr& d2)
{
    NodeAddrIndicators result;
    IsNodeAddrSame(d1, d2, result);

    if (result.node_name)// Node name conflict. Node name should be unique.
        return NodeAddrCompResult::DEVINFO_CONFLICT_NODE_NAME;

    // If same device, the hostname, ipv4, ipv6, and mac address should be same.
    // If different device, the hostname, ipv4, ipv6, and mac address should be different.

#ifdef RV2_USE_IPV6
    if (result.hostname && result.ipv4_addr && result.ipv6_addr && result.mac_addr)// Same device.
        return NodeAddrCompResult::DEVINFO_SAME;
    else if (!result.hostname && !result.ipv4_addr && !result.ipv6_addr && !result.mac_addr)// Different device.
        return NodeAddrCompResult::DEVINFO_DIFFERENT;
#else
    if (result.hostname && result.ipv4_addr && result.mac_addr)// Same device.
        return NodeAddrCompResult::DEVINFO_SAME;
    else if (!result.hostname && !result.ipv4_addr && !result.mac_addr)// Different device.
        return NodeAddrCompResult::DEVINFO_DIFFERENT;
#endif
    return NodeAddrCompResult::DEVINFO_CONFLICT;// Conflict hostnames, ipv4, ipv6, or mac address.
}



inline bool CvtNodeAddrToJSON(const msg::NodeAddr& profile, nlohmann::json& json)
{
    try
    {
        json["node_name"] = profile.node_name;
        json["hostname"] = profile.hostname;
        json["mac_addr"] = profile.mac_addr;
        json["ipv4_addr"] = profile.ipv4_addr;
        json["ipv6_addr"] = profile.ipv6_addr;
        return true;
    }
    catch (const nlohmann::json::exception& e)
    {
        printf("Caught exception: %s\n", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        printf("Caught exception: %s\n", e.what());
        return false;
    }
    catch (...)
    {
        printf("Caught exception: Unknown\n");
        return false;
    }
}



inline bool CvtJSONToNodeAddr(const nlohmann::json& json, msg::NodeAddr& profile)
{
    try
    {
        profile.node_name = json["node_name"];
        profile.hostname = json["hostname"];
        profile.mac_addr = json["mac_addr"];
        profile.ipv4_addr = json["ipv4_addr"];
        profile.ipv6_addr = json["ipv6_addr"];
        return true;
    }
    catch (const nlohmann::json::exception& e)
    {
        printf("Caught exception: %s\n", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        printf("Caught exception: %s\n", e.what());
        return false;
    }
    catch (...)
    {
        printf("Caught exception: Unknown\n");
        return false;
    }
}



inline std::string GetNodeAddrStr(const msg::NodeAddr& profile)
{
    std::stringstream ss;
    ss << "["  << std::left << std::setw(20)<< profile.node_name << "] " 
        << "[" << std::left << std::setw(20) << profile.hostname << " " 
        << std::left << std::setw(15) << profile.ipv4_addr << " " 
        << std::left << std::setw(17) << profile.mac_addr << "]";
    return ss.str();
}



/**
 * ================================================================
 * InteractiveService Command and Functions
 * ================================================================
 */



struct DevManageForceExit
{
    int exit_code;
    int delay_ms;

    DevManageForceExit() : exit_code(1), delay_ms(500) {}
};



inline std::vector<std::string> CvtDevManageForceExitToInteractiveServiceCommandArgs(const DevManageForceExit prof)
{
    std::vector<std::string> ret;
    ret.push_back("exit_code:" + std::to_string(prof.exit_code));
    ret.push_back("delay_ms:" + std::to_string(prof.delay_ms));
    return ret;
}



inline DevManageForceExit CvtInteractiveServiceCommandArgsToDevManageForceExit(const std::vector<std::string>& args)
{
    /**
     * The arguments of the service command arguments should be in the following format:
     * "exit_code:1"
     * "delay_ms:500"
     */
    DevManageForceExit prof;
    for (const auto& i : args)
    {
        auto arg = rv2_interfaces::split(i, ":");
        if (arg.size() != 2)
            continue;
        try
        {
            if (arg[0] == "exit_code")
                prof.exit_code = stoi(arg[1]);
            else if (arg[0] == "delay_ms")
                prof.delay_ms = stoi(arg[1]);
        }
        catch(...)
        {
            continue;
        }
    }
    return prof;
}



/**
 * ================================================================
 * DevManageNode Class Implementation
 * ================================================================
 */



/**
 *      The class is used to register the device information to the DevManageServer, 
 *      and provide the procedure monitoring service.
 *      For nodes that need to join the DevManageServer, they should inherit the DevManageNode class.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <nodeName>_devmanage_isrv        : The InteractiveService service for the DevManageNode.
 *          <nodeName>_devmanage_isrv_Req    : The InteractiveService status request service for the DevManageNode.
 */
class DevManageNode : virtual public rclcpp::Node
{
private:
    std::string mNodeName_;// Node name.
    std::string mDevManageSrvName_;// Service name.
    std::string mIfName_;// Network interface.
    rv2_interfaces::unique_thread mCollectNodeAddrTh_;// Collect device information thread.

    // Node address
    msg::NodeAddr mNodeAddr_;
    std::mutex mNodeAddrMtx_;

    // Procedure monitor
    std::map<std::string, ProcedureMonitorProp> mPMMap_;// { procedureName, ProcedureMonitorProp }
    std::mutex mPMMapMtx_;

    // InteractiveService
    std::shared_ptr<InteractiveService> mISrv_;// InteractiveService.
    rv2_interfaces::unique_thread mForceExitTh_;// InteractiveService command thread.

    // Procedure status monitor service
    rclcpp::Client<srv::ProcStatusReg>::SharedPtr mPSRegCli_;// Procedure status registration client.
    rclcpp::CallbackGroup::SharedPtr mPSRegCbG_;// Procedure status registration callback group.
    std::shared_ptr<rv2_interfaces::LiteTimer> mPSRegTm_;// Procedure status registration timer.
    std::chrono::nanoseconds mPSRegTmPeriod_ns_;// Procedure status registration timer period.

    // Node enable
    std::atomic<bool> mNodeEnableF_;
    std::atomic<bool> mExitF_;

public:
    DevManageNode(const std::string& nodeName, const rclcpp::NodeOptions & options) : 
        rclcpp::Node(nodeName, options), 
        mNodeEnableF_(false), 
        mExitF_(false)
    {
        // Get parameters.
        double procStatusRegPeriod_ms = -1.0;
        GetParamRawPtr(this, "devManageService", mDevManageSrvName_, mDevManageSrvName_, "devManageService: ", false);
        GetParamRawPtr(this, "devInterface", mIfName_, mIfName_, "devInterface: ", false);
        GetParamRawPtr(this, "procStatusRegPeriod_ms", procStatusRegPeriod_ms, procStatusRegPeriod_ms, "procStatusRegPeriod_ms: ", false);
        mNodeName_ = this->get_name();

        if (mDevManageSrvName_ == "" || mIfName_ == "")
        {
            RCLCPP_WARN(this->get_logger(), "[DevManageNode] Ignored.");
            return;
        }

        if (procStatusRegPeriod_ms > 0)
        {
            mPSRegTmPeriod_ns_ = std::chrono::milliseconds((int)procStatusRegPeriod_ms);
            mPSRegCbG_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            mPSRegCli_ = this->create_client<srv::ProcStatusReg>(DevManageServer_ProcStatusRegSrvName(mDevManageSrvName_), rmw_qos_profile_services_default, mPSRegCbG_);
            mPSRegTm_ = rv2_interfaces::make_unique_timer(procStatusRegPeriod_ms, std::bind(&DevManageNode::_psRegTmCb, this));
            mPSRegTm_->start();
            RCLCPP_INFO(this->get_logger(), "[DevManageNode] Procedure status registration timer started.");
        }
        else
            RCLCPP_WARN(this->get_logger(), "[DevManageNode] Ignore procedure status registration.");

        mNodeAddr_.node_name = mNodeName_;
        mCollectNodeAddrTh_ = rv2_interfaces::make_unique_thread(&DevManageNode::_collectNodeAddr, this);

        RCLCPP_INFO(this->get_logger(), "[DevManageNode] Constructed.");
        mNodeEnableF_.store(true);
    }

    ~DevManageNode()
    {
        mExitF_.store(true);
        mCollectNodeAddrTh_.reset();
    }

private:
    /**
     * This function will be called only once and run under sub-thread while construction time.
     */
    void _collectNodeAddr()
    {
        // Grab device information.
        std::string hostName, ipv4, mac;
        while (!GetHostname(hostName) && !mExitF_.load())
            std::this_thread::sleep_for(1000ms);
        RCLCPP_INFO(this->get_logger(), "[DevManageNode::_collectNodeAddr] Hostname: %s", hostName.c_str());
        while (!GetIPv4Addr(mIfName_, ipv4) && !mExitF_.load())
            std::this_thread::sleep_for(1000ms);
        RCLCPP_INFO(this->get_logger(), "[DevManageNode::_collectNodeAddr] IPv4: %s", ipv4.c_str());
        while (!GetMACAddr(mIfName_, mac) && !mExitF_.load())
            std::this_thread::sleep_for(1000ms);
        RCLCPP_INFO(this->get_logger(), "[DevManageNode::_collectNodeAddr] MAC: %s", mac.c_str());

        {
            std::lock_guard<std::mutex> nodeAddrLock(mNodeAddrMtx_);
            mNodeAddr_.hostname = hostName;
            mNodeAddr_.ipv4_addr = ipv4;
            mNodeAddr_.mac_addr = mac;
        }

        do
        {
            std::this_thread::sleep_for(100ms);
        } while (!mNodeEnableF_.load() && !mExitF_.load());

        // Register device information to the DevManageServer.
        while (this->_regNodeAddr() != ServiceResponseStatus::SRV_RES_SUCCESS && !mExitF_.load())
            std::this_thread::sleep_for(1000ms);

        if (mExitF_.load())
            return;

        try
        {
            // Create InteractiveService.
            InteractiveServiceInitProp iSrvProp;
            iSrvProp.targetAlive = msg::InteractiveService::TARGET_ALIVE_DISABLE;
            iSrvProp.targetActivity = msg::InteractiveService::TARGET_ACTIVITY_DISABLE;
            mISrv_ = std::make_shared<InteractiveService>(this, DevManageNode_ISrvName(mNodeName_), iSrvProp);

            InteractiveServiceMasterPrivilege privi;
            privi.masterID = mDevManageSrvName_;
            privi.targetAlive = InteractiveServiceMasterPrivilegeTargetAlive::TARGET_ALIVE_NONE;
            privi.targetActivity = InteractiveServiceMasterPrivilegeTargetActivity::TARGET_ACTIVITY_NONE;
            privi.serviceCommandSet = { "node_exit" };
            privi.requestInteractiveService = true;
            mISrv_->addMasterPrivilege(privi);
            mISrv_->addServiceCommandEventHandler("node_exit", std::bind(&DevManageNode::_forceExitEventHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[DevManageNode::_collectNodeAddr] Caught unexpected errors.");
        }
    }

    ServiceResponseStatus _regNodeAddr()
    {
        auto req = std::make_shared<srv::NodeAddrReg::Request>();
        {
            std::lock_guard<std::mutex> nodeAddrLock(mNodeAddrMtx_);
            req->node_addr = mNodeAddr_;
        }

        if (req->node_addr.ipv4_addr.length() <= 0 || 
#ifdef RV2_USE_IPV6
            req->node_addr.ipv6_addr.length() <= 0 || 
#endif
            req->node_addr.mac_addr.length() <= 0 || 
            req->node_addr.hostname.length() <= 0)
            return ServiceResponseStatus::SRV_RES_UNKNOWN_ERROR;

        auto res = ClientRequestHelperRawPtr<srv::NodeAddrReg>(this, DevManageServer_NodeAddrRegSrvName(mDevManageSrvName_), req, 500ms);
        if (res)
            return (ServiceResponseStatus)res->response;
        return ServiceResponseStatus::SRV_RES_UNKNOWN_ERROR;
    }

    void _psRegTmCb()
    {
        auto req = std::make_shared<srv::ProcStatusReg::Request>();
        req->node_name = mNodeName_;

        auto tmp = getProcedureStatus();
        for (const auto& [pName, proc] : tmp)
        {
            req->proc_status_vec.push_back(CvtProcedureStatusMsg(proc));
        }
        auto result = mPSRegCli_->async_send_request(req);
        auto fStatus = result.wait_for(mPSRegTmPeriod_ns_ * 0.8);
        if (fStatus != std::future_status::ready || result.get()->response != ServiceResponseStatus::SRV_RES_SUCCESS)
            RCLCPP_WARN(this->get_logger(), "[DevManageNode::_psRegTmCb] Procedure status registration failed.");
    }

    bool _forceExitEventHandler(InteractiveService *iSrv, const std::string deviceID, const std::vector<std::string> args)
    {
        auto prof = CvtInteractiveServiceCommandArgsToDevManageForceExit(args);
        mForceExitTh_ = rv2_interfaces::make_unique_thread(&DevManageNode::_forceExitTh, this, prof);
        return true;
    }

    bool _forceExitTh(DevManageForceExit prof)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10ms));// Wait for _forceExitEventHandler() to return.
        if (prof.delay_ms > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(prof.delay_ms));
        exit(prof.exit_code);
    }

public:
    bool addProcedureMonitor(const std::string& procedureName, std::chrono::nanoseconds timeout_ns)
    {
        if (!mNodeEnableF_.load())
            return false;

        std::lock_guard<std::mutex> pmMapLock(mPMMapMtx_);
        if (mPMMap_.find(procedureName) != mPMMap_.end())// Already exist.
            return false;
        mPMMap_[procedureName] = { procedureName, timeout_ns, PROCEDURE_STATUS_NO_RESPONSE, "INIT", std::chrono::system_clock::now() };
        return true;
    }

    void updateProcedureMonitor(const std::string& procedureName, ProcedureStatus status, std::string msg = "")
    {
        if (!mNodeEnableF_.load())
            return;

        std::lock_guard<std::mutex> pmMapLock(mPMMapMtx_);
        if (mPMMap_.find(procedureName) == mPMMap_.end())// Not exist.
            return;
        mPMMap_[procedureName].status = status;
        mPMMap_[procedureName].statusMsg = msg;
        mPMMap_[procedureName].lastResponseTime = std::chrono::system_clock::now();
    }

    std::map<std::string, ProcedureMonitorProp> getProcedureStatus()
    {
        std::lock_guard<std::mutex> pmMapLock(mPMMapMtx_);
        for (auto& [procedureName, prop] : mPMMap_)
        {
            if (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - prop.lastResponseTime) > prop.timeout_ns)
                prop.status = PROCEDURE_STATUS_NO_RESPONSE;
        }
        return mPMMap_;
    }
};

}

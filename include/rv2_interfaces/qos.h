#pragma once

#include <set>

#include <condition_variable>
#include <stdexcept>
#include <memory>
#include <mutex>


#ifndef FOUND_FOXY
#include <rmw/time.h>
#include <rmw/qos_string_conversions.h>
#endif

#include <rmw/qos_profiles.h>

#include <rv2_interfaces/interactive_publisher.h>
#include <rv2_interfaces/interactive_subscription.h>
#include <rv2_interfaces/service.h>
#include <rv2_interfaces/utils.h>

#include <rv2_interfaces/msg/qos_profile.hpp>
#include <rv2_interfaces/msg/topic_device_info.hpp>
#include <rv2_interfaces/msg/topic_qos_profile.hpp>

#include <rv2_interfaces/srv/qos_server.hpp>
#include <rv2_interfaces/srv/topic_device_info_reg.hpp>
#include <rv2_interfaces/srv/topic_qos_profile_reg.hpp>
#include <rv2_interfaces/srv/topic_qos_profile_req.hpp>

#define QoSNode_ISrvName(serviceName) (std::string)serviceName + "_qos_isrv"
#define QoSServer_TopicDeviceInfoRegSrvName(serviceName) (std::string)serviceName + "_topicdevinfo_Reg"
#define QoSServer_TopicQosProfileRegSrvName(serviceName) (std::string)serviceName + "_Reg"
#define QoSServer_TopicQosProfileReqSrvName(serviceName) (std::string)serviceName + "_Req"
#define QoSServer_QosServerSrvName(serviceName) (std::string)serviceName

using namespace std::chrono_literals;

/**
 * ================================================================================================
 *                                      Rules for the QoS Service
 * ================================================================================================
 * 
 * ------------------------------------------------------------------------------------------------
 * QoSNode:
 *      The class implements the communication to the QoS server. 
 *      For nodes that need to join the QoS management, they should inherit the QoSNode class.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <nodeName>_qos_isrv         : The InteractiveService service for the QoSNode.
 *          <nodeName>_qos_isrv_Req     : The InteractiveService status request service for the QoSNode.
 * ------------------------------------------------------------------------------------------------
 * QoSServer:
 *      The class implements the QoS management service.
 *      The QoSServer manages the Registered QoS profiles and the topic device information, 
 *      and provides the QoS profile update mechanism to the QoSNode.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <service_name>                    : The QoSServer control service.
 *          <service_name>_Reg                : The QoS profile registration service.
 *          <service_name>_Req                : The QoS profile request service.
 *          <service_name>_topicdevinfo_Reg   : The topic device information registration service.
 * ------------------------------------------------------------------------------------------------
 * InteractiveService Command:
 *      - "qos_update" : Update the QoS profile of the registered topic device under the QoSNode.
 *          Arguments:
 *                      "history:<enum>"
 *                      "depth:<int>"
 *                      "reliability:<enum>"
 *                      "durability:<enum>"
 *                      "deadline:<double ms>"
 *                      "lifespan:<double ms>"
 *                      "liveliness:<enum>"
 *                      "liveliness_lease_duration:<double ms>"
 * 
 *      - "qos_rereg" : QoSNode re-register the topic device information to the QoSServer.
 *          Arguments:
 *                      "re_register:<int status>"
 * ------------------------------------------------------------------------------------------------
 */



namespace rv2_interfaces
{

enum RmwQoSIndicatorsInit
{
    RMW_QOS_INDICATORS_INIT_ALL_FALSE = 0, 
    RMW_QOS_INDICATORS_INIT_ALL_TRUE = 1, 
    RMW_QOS_INDICATORS_INIT_REQUIRED_TRUE = 2// Required indicators are history, reliability, durability, and liveliness.
};

struct RmwQoSIndicators
{
    bool history;
    bool depth;
    bool reliability;
    bool durability;
    bool deadline;
    bool lifespan;
    bool liveliness;
    bool liveliness_lease_duration;

    RmwQoSIndicators() : history(false), depth(false), reliability(false), durability(false), deadline(false), lifespan(false), liveliness(false), liveliness_lease_duration(false) {}

    RmwQoSIndicators(const RmwQoSIndicatorsInit init) : history(false), depth(false), reliability(false), durability(false), deadline(false), lifespan(false), liveliness(false), liveliness_lease_duration(false)
    {
        switch (init)
        {
            case RMW_QOS_INDICATORS_INIT_ALL_FALSE:
                break;
            case RMW_QOS_INDICATORS_INIT_ALL_TRUE:
                history = true;
                depth = true;
                reliability = true;
                durability = true;
                deadline = true;
                lifespan = true;
                liveliness = true;
                liveliness_lease_duration = true;
                break;
            case RMW_QOS_INDICATORS_INIT_REQUIRED_TRUE:
                history = true;
                reliability = true;
                durability = true;
                liveliness = true;
                break;
            default:
                break;
        }
    }
};


inline bool CvtJSONToTopicDeviceInfo(const nlohmann::json& json, msg::TopicDeviceInfo& outTopicDeviceInfo)
{
    try
    {
        outTopicDeviceInfo.node_name = json["node_name"];
        outTopicDeviceInfo.topic_name = json["topic_name"];
        outTopicDeviceInfo.qos_type = json["qos_type"];
        return true;
    }
    catch (...)
    {
        return false;
    }
}

inline bool CvtTopicDeviceInfoToJSON(const msg::TopicDeviceInfo& topicDeviceInfo, nlohmann::json& outJSON)
{
    try
    {
        outJSON["node_name"] = topicDeviceInfo.node_name;
        outJSON["topic_name"] = topicDeviceInfo.topic_name;
        outJSON["qos_type"] = topicDeviceInfo.qos_type;
        return true;
    }
    catch (...)
    {
        return false;
    }
}



/**
 * ============================================================================
 * RMW time functions.
 * ============================================================================
 */



#ifdef FOUND_FOXY

// Ref: https://github.com/ros2/rmw/blob/humble/rmw/src/time.c

inline rmw_duration_t
rmw_time_total_nsec(const rmw_time_t time)
{
  static const uint64_t max_sec = INT64_MAX / RCUTILS_S_TO_NS(1);
  if (time.sec > max_sec) {
    // Seconds not representable in nanoseconds
    return INT64_MAX;
  }

  const int64_t sec_as_nsec = RCUTILS_S_TO_NS(time.sec);
  if (time.nsec > (uint64_t)(INT64_MAX - sec_as_nsec)) {
    // overflow
    return INT64_MAX;
  }
  return sec_as_nsec + time.nsec;
}

inline bool
rmw_time_equal(const rmw_time_t left, const rmw_time_t right)
{
  return rmw_time_total_nsec(left) == rmw_time_total_nsec(right);
}

inline rmw_time_t
rmw_time_from_nsec(const rmw_duration_t nanoseconds)
{
  if (nanoseconds < 0) {
    return (rmw_time_t)RMW_DURATION_INFINITE;
  }

  // Avoid typing the 1 billion constant
  rmw_time_t time;
  time.sec = RCUTILS_NS_TO_S(nanoseconds);
  time.nsec = nanoseconds % RCUTILS_S_TO_NS(1);
  return time;
}

inline rmw_time_t
rmw_time_normalize(const rmw_time_t time)
{
  return rmw_time_from_nsec(rmw_time_total_nsec(time));
}

typedef rmw_qos_history_policy_e rmw_qos_history_policy_t;
typedef rmw_qos_reliability_policy_e rmw_qos_reliability_policy_t;
typedef rmw_qos_durability_policy_e rmw_qos_durability_policy_t;
typedef rmw_qos_liveliness_policy_e rmw_qos_liveliness_policy_t;

#endif



/**
 * @brief Convert time double ms to rmw_time_t.
 * @param[in] time_ms The double ms.
 * @return The rmw_time_t.
 */
inline rmw_time_t CvtMsToRmwTime(const double& time_ms)
{
    return rmw_time_from_nsec(RCUTILS_MS_TO_NS(static_cast<int64_t>(time_ms)));
}

/**
 * @brief Convert rmw_time_t to time double ms.
 * @param[in] rmwTime The rmw_time_t.
 * @return The double ms.
 */
inline double CvtRmwTimeToMs(const rmw_time_t& rmwTime)
{
    return static_cast<double>(RCUTILS_NS_TO_MS(rmw_time_total_nsec(rmwTime)));
}



/**
 * ============================================================================
 * RMW QoS functions.
 * ============================================================================
 */

/**
 * @brief Convert the rmw_qos_profile_t to the QosProfile message.
 * @param[in] prof The rmw_qos_profile_t.
 * @return The QosProfile message.
 */
inline msg::QosProfile CvtRmwQoSToMsg(const rmw_qos_profile_t& prof)
{
    msg::QosProfile ret;
    ret.history = prof.history;
    ret.depth = prof.depth;
    ret.reliability = prof.reliability;
    ret.durability = prof.durability;
    ret.deadline_ms = CvtRmwTimeToMs(prof.deadline);
    ret.lifespan_ms = CvtRmwTimeToMs(prof.lifespan);
    ret.liveliness = prof.liveliness;
    ret.liveliness_lease_duration_ms = CvtRmwTimeToMs(prof.liveliness_lease_duration);
    return ret;
}

/**
 * @brief Convert QosProfile message to rmw_qos_profile_t.
 * @param[in] msg The QosProfile message.
 * @return The rmw_qos_profile_t.
 */
inline rmw_qos_profile_t CvtMsgToRmwQoS(const msg::QosProfile& msg)
{
    rmw_qos_profile_t prof = rclcpp::QoS(10).get_rmw_qos_profile();
    prof.history = (rmw_qos_history_policy_e)msg.history;
    prof.depth = msg.depth;
    prof.reliability = (rmw_qos_reliability_policy_e)msg.reliability;
    prof.durability = (rmw_qos_durability_policy_e)msg.durability;
    prof.deadline = CvtMsToRmwTime(msg.deadline_ms);
    prof.lifespan = CvtMsToRmwTime(msg.lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_e)msg.liveliness;
    prof.liveliness_lease_duration = CvtMsToRmwTime(msg.liveliness_lease_duration_ms);
    // printf("history: %d\ndepth: %d\nreliability: %d\ndurability: %d\n \\
    //         deadline:%d,%d\nlifespan: %d,%d\nliveliness: %d\nliveliness_lease_duration: %d,%d\n", 
    //         prof.history, prof.depth, prof.reliability, prof.durability, 
    //         prof.deadline.sec, prof.deadline.nsec, prof.lifespan.sec, prof.lifespan.nsec, 
    //         prof.liveliness, prof.liveliness_lease_duration.sec, prof.liveliness_lease_duration.nsec);
    return prof;
}

/**
 * @brief Compare two rmw_qos_profile_t.
 * @param[in] p1 The first rmw_qos_profile_t.
 * @param[in] p2 The second rmw_qos_profile_t.
 * @return true if the two QoS profiles are the same; false otherwise.
 */
inline bool CompRMWQoS(const rmw_qos_profile_t& p1, const rmw_qos_profile_t& p2)
{
    return p1.history == p2.history && 
            p1.depth == p2.depth && 
            p1.reliability == p2.reliability && 
            p1.durability == p2.durability && 
            rmw_time_equal((rmw_time_t)p1.deadline, (rmw_time_t)p2.deadline) &&
            rmw_time_equal((rmw_time_t)p1.lifespan, (rmw_time_t)p2.lifespan) &&
            p1.liveliness == p2.liveliness && 
            rmw_time_equal((rmw_time_t)p1.liveliness_lease_duration, (rmw_time_t)p2.liveliness_lease_duration);
}

/**
 * @brief Convert the rmw_qos_profile_t to the rclcpp::QoS.
 * @param[in] prof The rmw_qos_profile_t.
 * @return The rclcpp::QoS.
 * @note The function does not support the deadline, lifespan, and liveliness settings.
 */
inline rclcpp::QoS CvtRmwQoSToRclQoS(const rmw_qos_profile_t& prof)
{
    rclcpp::QoS ret(prof.depth);
    ret.history(prof.history);
    ret.reliability(prof.reliability);
    ret.durability(prof.durability);
    // ret.deadline(prof.deadline);
    // ret.lifespan(prof.lifespan);
    // ret.liveliness(prof.liveliness);
    // ret.liveliness_lease_duration(prof.liveliness_lease_duration);
    return ret;
}



/**
 * ============================================================================
 * RMW QoS read and write functions.
 * ============================================================================
 */

/**
 * @brief Convert the QoS profile to the JSON object.
 * @param[in] profile The RMW QoS profile.
 * @param[out] outJSON The output JSON object.
 * @return True if convert successful; false otherwise.
 * @note The function will overwrite the file if it exists.
 */
inline bool CvtRmwQoSToJSON(const rmw_qos_profile_t& profile, nlohmann::json& outJSON)
{
    try
    {
        outJSON["history"] = (int8_t)profile.history;
        outJSON["depth"] = profile.depth;
        outJSON["reliability"] = (int8_t)profile.reliability;
        outJSON["durability"] = (int8_t)profile.durability;
        outJSON["deadline_ms"] = CvtRmwTimeToMs(profile.deadline);
        outJSON["lifespan_ms"] = CvtRmwTimeToMs(profile.lifespan);
        outJSON["liveliness"] = (int8_t)profile.liveliness;
        outJSON["liveliness_lease_duration_ms"] = CvtRmwTimeToMs(profile.liveliness_lease_duration);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

/**
 * @brief Load the QoS profile from the JSON object.
 * @param[in] json The JSON object.
 * @param[out] profile The output QoS profile.
 * @return True if the QoS profile is loaded successfully; false otherwise.
 */
inline bool CvtJSONToRmwQoS(const nlohmann::json& json, rmw_qos_profile_t& outProfile)
{
    try
    {
        outProfile.history = (rmw_qos_history_policy_e)json["history"];
        outProfile.depth = json["depth"];
        outProfile.reliability = (rmw_qos_reliability_policy_e)json["reliability"];
        outProfile.durability = (rmw_qos_durability_policy_e)json["durability"];
        outProfile.deadline = CvtMsToRmwTime(json["deadline_ms"]);
        outProfile.lifespan = CvtMsToRmwTime(json["lifespan_ms"]);
        outProfile.liveliness = (rmw_qos_liveliness_policy_e)json["liveliness"];
        outProfile.liveliness_lease_duration = CvtMsToRmwTime(json["liveliness_lease_duration_ms"]);
    }
    catch (...)
    {
        return false;
    }
    return true;
}



// See QOS_TYPE_XXX in rv2_interfaces/msg/TopicQosProfile.msg
inline std::string GetQosTypeStr(uint8_t type)
{
    switch (type)
    {
        case msg::TopicQosProfile::QOS_TYPE_PUBLISHER:
            return "Publisher";
        case msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION:
            return "Subscription";
        case msg::TopicQosProfile::QOS_TYPE_BOTH:
            return "Both";
        default:
            return "Unknown";
    }
}



inline void AddHierarchicalPrint(HierarchicalPrint& hprint, int hierarchy, const msg::TopicDeviceInfo& msg)
{
    hprint.push(hierarchy, "[%s]", msg.node_name.c_str());
    hprint.push(hierarchy + 1, "[topic_name] %s", msg.topic_name.c_str());
    hprint.push(hierarchy + 1, "[qos_type] %s", GetQosTypeStr(msg.qos_type).c_str());
}



inline void AddHierarchicalPrint(HierarchicalPrint& hprint, int hierarchy, const msg::TopicQosProfile& msg)
{
    hprint.push(hierarchy, "[%s (%s)]", msg.topic_name.c_str(), GetQosTypeStr(msg.qos_type).c_str());
#ifdef FOUND_FOXY
    hprint.push(hierarchy + 1, "[history] %d", msg.qos_profile.history);
    hprint.push(hierarchy + 1, "[depth] %d", msg.qos_profile.depth);
    hprint.push(hierarchy + 1, "[reliability] %d", msg.qos_profile.reliability);
    hprint.push(hierarchy + 1, "[durability] %d", msg.qos_profile.durability);
#else
    hprint.push(hierarchy + 1, "[history] %s", rmw_qos_history_policy_to_str((rmw_qos_history_policy_t)msg.qos_profile.history));
    hprint.push(hierarchy + 1, "[depth] %d", msg.qos_profile.depth);
    hprint.push(hierarchy + 1, "[reliability] %s", rmw_qos_reliability_policy_to_str((rmw_qos_reliability_policy_t)msg.qos_profile.reliability));
    hprint.push(hierarchy + 1, "[durability] %s", rmw_qos_durability_policy_to_str((rmw_qos_durability_policy_t)msg.qos_profile.durability));
#endif
}




/**
 * ============================================================================
 * RMW QoS and InteractiveService command functions.
 * ============================================================================
 */

/**
 * @brief Convert the rmw_qos_profile_t to the InteractivceNode service command arguments.
 * @details The function converts the rmw_qos_profile_t to the arguments of the service command.
 * @param[in] prof The QoS profile.
 * @return The arguments of the service command.
 * @note This argument supports the "qos_update" command.
 */
inline std::vector<std::string> CvtRMWQoSToInteractiveServiceCommandArgs(const rmw_qos_profile_t& prof)
{
    std::vector<std::string> ret;
    ret.push_back("history:" + std::to_string(prof.history));
    ret.push_back("depth:" + std::to_string(prof.depth));
    ret.push_back("reliability:" + std::to_string(prof.reliability));
    ret.push_back("durability:" + std::to_string(prof.durability));
    ret.push_back("deadline:" + std::to_string(CvtRmwTimeToMs(prof.deadline)));
    ret.push_back("lifespan:" + std::to_string(CvtRmwTimeToMs(prof.lifespan)));
    ret.push_back("liveliness:" + std::to_string(prof.liveliness));
    ret.push_back("liveliness_lease_duration:" + std::to_string(CvtRmwTimeToMs(prof.liveliness_lease_duration)));
    return ret;
}

/**
 * @brief Convert the InteractivceNode service command arguments to rclcpp::QoS.
 * @details The function converts the arguments of the service command to rclcpp::QoS.
 * @param[in] args The arguments of the service command funtion.
 * @return The rclcpp::QoS object.
 * @note This argument supports the "qos_update" command.
 */
inline rmw_qos_profile_t CvtInteractiveServiceCommandArgsToRMWQoS(const std::vector<std::string>& args)
{
    /**
     * The arguments of the service command arguments should be in the following format:
     * "history:0"
     * "depth:10"
     * "reliability:0"
     * "durability:0"
     * "deadline:0"
     * "lifespan:0"
     * "liveliness:0"
     * "liveliness_lease_duration:0"
     */
    rmw_qos_profile_t qosProp = rmw_qos_profile_default;
    for (const auto& i : args)
    {
        auto arg = rv2_interfaces::split(i, ":");
        if (arg.size() != 2)
            continue;
        try
        {
            if (arg[0] == "history")
                qosProp.history = rmw_qos_history_policy_e(stoi(arg[1]));
            else if (arg[0] == "depth")
                qosProp.depth = stoi(arg[1]);
            else if (arg[0] == "reliability")
                qosProp.reliability = rmw_qos_reliability_policy_e(stoi(arg[1]));
            else if (arg[0] == "durability")
                qosProp.durability = rmw_qos_durability_policy_e(stoi(arg[1]));
            else if (arg[0] == "deadline")
                qosProp.deadline = CvtMsToRmwTime(stoi(arg[1]));
            else if (arg[0] == "lifespan")
                qosProp.lifespan = CvtMsToRmwTime(stoi(arg[1]));
            else if (arg[0] == "liveliness")
                qosProp.liveliness = rmw_qos_liveliness_policy_e(stoi(arg[1]));
            else if (arg[0] == "liveliness_lease_duration")
                qosProp.liveliness_lease_duration = CvtMsToRmwTime(stoi(arg[1]));
        }
        catch(...)
        {
            continue;
        }
    }
    return qosProp;
}


/**
 * @brief Convert the re-register signal to the InteractivceNode service command arguments.
 * @details The function converts the re-register signal to the arguments of the service command.
 * @param[in] status The re-register status. (preserved for future use)
 * @return The arguments of the service command.
 * @note This argument supports the "qos_rereg" command.
 */
inline std::vector<std::string> CvtReRegisterSignalToInteractiveServiceCommandArgs(int status)
{
    std::vector<std::string> ret;
    ret.push_back("re_register:" + std::to_string(status));
    return ret;
}


/**
 * @brief Convert the InteractivceNode service command arguments to re-register signal.
 * @details The function converts the arguments of the service command to re-register signal.
 * @param[in] args The arguments of the service command funtion.
 * @return The re-register signal.
 * @note This argument supports the "qos_rereg" command.
 */
inline int CvtInteractiveServiceCommandArgsToReRegisterSignal(const std::vector<std::string>& args)
{
    for (const auto& i : args)
    {
        auto arg = rv2_interfaces::split(i, ":");
        if (arg.size() != 2)
            continue;
        try
        {
            if (arg[0] == "re_register")
                return stoi(arg[1]);
        }
        catch(...)
        {
            continue;
        }
    }
    return -1;
}



/**
 * ============================================================================
 * QoSNode functions and classes.
 * ============================================================================
 */

/**
 *      The class implements the communication to the QoS server. 
 *      For nodes that need to join the QoS management, they should inherit the QoSNode class.
 * 
 *      - Created Nodes:
 *          None.
 * 
 *      - Created Topics:
 *          None.
 * 
 *      - Created Services:
 *          <nodeName>_qos_isrv         : The InteractiveService service for the QoSNode.
 *          <nodeName>_qos_isrv_Req     : The InteractiveService status request service for the QoSNode.
 */
class QoSNode : virtual public rclcpp::Node
{
private:
    std::string mNodeName_;// Node name.
    std::string mQoSSrvName_;// The name of the QoSServer service.
    fs::path mQoSDirPath_;// The directory path of the QoS configuration files.

    std::shared_ptr<MapRegister<std::string, msg::TopicDeviceInfo> > mTopicDevInfoRegister_;// The map register for the topic device information.

    // Interactive service
    std::shared_ptr<InteractiveService> mISrv_;// Interactive service.

    std::atomic<bool> mNodeEnableF_;// Flag indicating whether the QoSNode is enabled.
    std::atomic<bool> mExitF_;// Flag indicating whether the service thread is running.

private:
    /**
     * @brief QoS update event handler.
     * @details The function is called when the interactive service server receives a QoS update request.
     * @param[in] iSrv Pointer to the interactive service. Should be MultiInteractiveTopic or InteractiveTopic.
     * @param[in] deviceID The device ID of the request sender.
     * @param[in] args The arguments of the service command function.
     */
    bool _qosUpdateEventHandler(InteractiveService *iSrv, const std::string deviceID, const std::vector<std::string> args)
    {
        auto qos = CvtInteractiveServiceCommandArgsToRMWQoS(args);
        if (auto qosNode = dynamic_cast<MultiInteractiveServiceWithQoS *>(iSrv))
        {
            qosNode->setQoS(CvtRmwQoSToRclQoS(qos));
        }
        else if (auto qosNode = dynamic_cast<InteractiveServiceWithQoS *>(iSrv))
        {
            qosNode->setQoS(CvtRmwQoSToRclQoS(qos));
        }
        std::string fileName = iSrv->getServiceName();
        rv2_interfaces::replace_all(fileName, "/", "_");
        nlohmann::json json;
        CvtRmwQoSToJSON(qos, json);
        DumpJSONToFile(mQoSDirPath_ / (fileName + ".json"), json);
        return true;
    }

    bool _topicDevInfoRegisterCb(const std::string keyName, const msg::TopicDeviceInfo value)
    {
        auto request = std::make_shared<srv::TopicDeviceInfoReg::Request>();
        request->request = value;
        auto response = ClientRequestHelperRawPtr<srv::TopicDeviceInfoReg>(this, QoSServer_TopicDeviceInfoRegSrvName(mQoSSrvName_), request, 500ms);
        if (response)
        {
            if (response->response == ServiceResponseStatus::SRV_RES_SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "[QoSNode::_topicDevInfoRegisterCb] %s[%s] registered to the QoS server.", 
                    request->request.topic_name.c_str(), 
                    request->request.qos_type == msg::TopicQosProfile::QOS_TYPE_PUBLISHER ? "Publisher" : 
                    (request->request.qos_type == msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION ? "Subscription" : "Unknown"));
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[QoSNode::_topicDevInfoRegisterCb] %s[%s] failed to register to the QoS server.", request->request.topic_name.c_str(), 
                    request->request.qos_type == msg::TopicQosProfile::QOS_TYPE_PUBLISHER ? "Publisher" : 
                    (request->request.qos_type == msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION ? "Subscription" : "Unknown"));
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::_topicDevInfoRegisterCb] Failed to connect to the QoS server.");
            return false;
        }
    }

    bool _reRegEventHandler(InteractiveService *iSrv, const std::string deviceID, const std::vector<std::string> args)
    {
        int status = CvtInteractiveServiceCommandArgsToReRegisterSignal(args);
        RCLCPP_INFO(this->get_logger(), "[QoSNode::_reRegEventHandler] Re-register signal received: %d", status);
        mTopicDevInfoRegister_->reRegister();
        return true;
    }

public:
    QoSNode(const std::string& nodeName, const rclcpp::NodeOptions & options) : 
        rclcpp::Node(nodeName, options), 
        mNodeEnableF_(false), 
        mExitF_(false)
    {
        // Get parameters.
        std::string qosDirPathName = "";
        GetParamRawPtr(this, "qosService", mQoSSrvName_, mQoSSrvName_, "qosService: ", false);
        GetParamRawPtr(this, "qosDirPath", qosDirPathName, qosDirPathName, "qosDirPath: ", false);
        mQoSDirPath_ = fs::path(qosDirPathName);
        mNodeName_ = this->get_name();

        if (mQoSSrvName_ == "" || qosDirPathName == "")
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode] Ignored.");
            return;
        }

        fs::create_directories(mQoSDirPath_);

        mTopicDevInfoRegister_ = std::make_shared<MapRegister<std::string, msg::TopicDeviceInfo> >(std::bind(&QoSNode::_topicDevInfoRegisterCb, this, std::placeholders::_1, std::placeholders::_2));

        // Interactive service
        InteractiveServiceInitProp iSrvProp;
        iSrvProp.targetAlive = msg::InteractiveService::TARGET_ALIVE_DISABLE;
        iSrvProp.targetActivity = msg::InteractiveService::TARGET_ACTIVITY_DISABLE;
        mISrv_ = std::make_shared<InteractiveService>(this, QoSNode_ISrvName(mNodeName_), iSrvProp);

        InteractiveServiceMasterPrivilege privi;
        privi.masterID = mQoSSrvName_;
        privi.targetAlive = InteractiveServiceMasterPrivilegeTargetAlive::TARGET_ALIVE_NONE;
        privi.targetActivity = InteractiveServiceMasterPrivilegeTargetActivity::TARGET_ACTIVITY_NONE;
        privi.serviceCommandSet = { "qos_rereg" };
        privi.requestInteractiveService = true;
        mISrv_->addMasterPrivilege(privi);
        mISrv_->addServiceCommandEventHandler("qos_rereg", std::bind(&QoSNode::_reRegEventHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "[QoSNode] Constructed.");
        mNodeEnableF_.store(true);
    }

    ~QoSNode()
    {
        mExitF_.store(true);
    }

    /**
     * @brief Add the QoS node command to the InteractiveService and record the topic device information.
     * @param[in] iSrv Pointer to the interactive service. Should be MultiInteractiveTopic or InteractiveTopic.
     * @param[in] tryReadQoSProfile If true, the function will try to read the QoS profile from the mQoSDirPath_.
     * @return True if the node command to the InteractiveService; false otherwise.
     */
    bool addQoSServiceCommand(std::shared_ptr<InteractiveService> iSrv, bool tryReadQoSProfile = true)
    {
        if (!mNodeEnableF_.load())
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSServiceCommand] Ignored.");
            return false;
        }

        if (mTopicDevInfoRegister_->isExist(iSrv->getServiceName()))
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSServiceCommand] %s is already added to the list.", iSrv->getServiceName().c_str());
            return false;
        }

        // Master privilege.
        InteractiveServiceMasterPrivilege privi;
        privi.masterID = mQoSSrvName_;
        privi.targetAlive = InteractiveServiceMasterPrivilegeTargetAlive::TARGET_ALIVE_ALL;
        privi.targetActivity = InteractiveServiceMasterPrivilegeTargetActivity::TARGET_ACTIVITY_ALL;
        privi.serviceCommandSet = { "qos_update" };
        privi.requestInteractiveService = false;

        // Add master privilege to the iSrv and add topic device information to the list.
        if (auto topicNode = std::dynamic_pointer_cast<MultiInteractiveServiceWithQoS>(iSrv))
        {
            topicNode->addMasterPrivilege(privi);// MultiInteractiveServiceWithQoS call addMasterPrivilege() to add init master target status.
            msg::TopicDeviceInfo tInfo;
            tInfo.node_name = topicNode->getServiceName();
            tInfo.topic_name = topicNode->getTopicName();
            tInfo.qos_type = topicNode->getQoSType();
            tInfo.manage_qos_node = mISrv_->getServiceName();
            mTopicDevInfoRegister_->add(tInfo.node_name, tInfo);
        }
        else if (auto topicNode = std::dynamic_pointer_cast<InteractiveServiceWithQoS>(iSrv))
        {
            topicNode->addMasterPrivilege(privi);
            msg::TopicDeviceInfo tInfo;
            tInfo.node_name = topicNode->getServiceName();
            tInfo.topic_name = topicNode->getTopicName();
            tInfo.qos_type = topicNode->getQoSType();
            mTopicDevInfoRegister_->add(tInfo.node_name, tInfo);
        }
        else// Neither MultiInteractiveServiceWithQoS nor InteractiveServiceWithQoS.
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSServiceCommand] %s is not a valid interactive topic node.", iSrv->getServiceName().c_str());
            return false;
        }

        // Try to read QoS profile.
        if (tryReadQoSProfile)
        {
            std::string fileName = iSrv->getServiceName();
            rv2_interfaces::replace_all(fileName, "/", "_");
            rmw_qos_profile_t qos = rmw_qos_profile_default;
            nlohmann::json json;
            if (LoadFileFromJSON(mQoSDirPath_ / (fileName + ".json"), json) && CvtJSONToRmwQoS(json, qos))
            {
                RCLCPP_INFO(this->get_logger(), "[QoSNode::addQoSServiceCommand] %s QoS profile loaded.", iSrv->getServiceName().c_str());
                RCLCPP_INFO(this->get_logger(), "QoS profile: %d/%d/%d/%d\n", qos.history, qos.depth, qos.reliability, qos.durability);
                uint8_t preStatus = iSrv->getTargetAlive();
                iSrv->callTargetAliveCbFunc(mQoSSrvName_, msg::InteractiveService::TARGET_ALIVE_DISABLE);
                if (auto topicNode = std::dynamic_pointer_cast<MultiInteractiveServiceWithQoS>(iSrv))
                {
                    topicNode->setQoS(CvtRmwQoSToRclQoS(qos));
                }
                else if (auto topicNode = std::dynamic_pointer_cast<InteractiveServiceWithQoS>(iSrv))
                {
                    topicNode->setQoS(CvtRmwQoSToRclQoS(qos));
                }
                iSrv->callTargetAliveCbFunc(mQoSSrvName_, preStatus);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSServiceCommand] %s QoS profile not found.", iSrv->getServiceName().c_str());
            }
        }

        iSrv->addServiceCommandEventHandler("qos_update", std::bind(&QoSNode::_qosUpdateEventHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(this->get_logger(), "[QoSNode::addQoSServiceCommand] Add %s to list.", iSrv->getServiceName().c_str());
        return true;
    }
};



/**
 * ============================================================================
 * TopicQoS functions and classes.
 * ============================================================================
 */

/**
 * @brief Result of the QoS comparison.
 * @details The enum is used to indicate the result of the QoS comparison.
 */
enum TopicQoSCompareResult
{
    QOS_CMP_SAME = 0,
    QOS_CMP_PUB_DIFF = 1,
    QOS_CMP_SUB_DIFF = 2,
    QOS_CMP_BOTH_DIFF = 3,
    QOS_CMP_TOPIC_DIFF = -1
};

/**
 * @brief The TopicQoSException enum.
 * @details The enum is used to indicate the exception of the TopicQoS class.
 */
enum TopicQoSException
{
    QOS_EXC_INVALID_TOPIC_NAME = 0, 
    QOS_EXC_INVALID_PUB_QOS, 
    QOS_EXC_INVALID_SUB_QOS, 
    QOS_EXC_INVALID_QOS_TYPE, 
    QOS_EXC_INVALID_PROPERTY
};

/**
 * @brief The TopicQoSExceptionMsg array.
 * @details The array is used to store the exception messages of the TopicQoS class.
 */
inline const char* TopicQoSExceptionMsg[] = 
{
    "Invalid topic name",
    "Invalid publisher QoS",
    "Invalid subscriber QoS",
    "Invalid topic type", 
    "Invalid property"
};



/**
 * The class store the QoS profiles for the topic, including the publisher and subscriber QoS profiles.
 * The class provides functions check the QoS profiles and compare the QoS profiles.
 */
class TopicQoS
{
private:
    std::string mTopicName_;// The name of the topic.
    std::shared_ptr<rmw_qos_profile_t> mPubQoS_;// The QoS profile for the publisher.
    std::shared_ptr<rmw_qos_profile_t> mSubQoS_;// The QoS profile for the subscriber.

    mutable std::string mErrInfo_;// Last error information.

public:
    /**
     * @brief Construct a new TopicQoS object without topic name and QoS profiles.
     * @details The function constructs a new TopicQoS object without topic name and QoS profiles.
     * @note The function does not allocate memory for the QoS profiles.
     * @note The topic name and QoS profiles should be set before using the object.
     */
    TopicQoS() : mTopicName_(""), mPubQoS_(nullptr), mSubQoS_(nullptr), mErrInfo_("") {}

    /**
     * @brief Construct a new TopicQoS object with the topic name.
     * @details The function constructs a new TopicQoS object with the topic name.
     * @param[in] topicName The name of the topic.
     * @note The function does not allocate memory for the QoS profiles.
     * @note The QoS profiles should be set before using the object.
     */
    TopicQoS(const std::string& topicName) : mTopicName_(topicName), mPubQoS_(nullptr), mSubQoS_(nullptr) {}

    /**
     * @brief Construct a new TopicQoS object with the topic name and QoS profiles.
     * @details The function constructs a new TopicQoS object with the topic name and QoS profiles.
     * @param[in] topicName The name of the topic.
     * @param[in] pubQoS The QoS profile for the publisher.
     * @param[in] subQoS The QoS profile for the subscriber.
     */
    TopicQoS(const std::string& topicName, const rmw_qos_profile_t& pubQoS, const rmw_qos_profile_t& subQoS) : 
        mTopicName_(topicName), 
        mPubQoS_(new rmw_qos_profile_t(pubQoS)), 
        mSubQoS_(new rmw_qos_profile_t(subQoS)) {}

    TopicQoS(const TopicQoS& q) : 
        mTopicName_(q.mTopicName_), 
        mPubQoS_(q.mPubQoS_ == nullptr ? nullptr : new rmw_qos_profile_t(*q.mPubQoS_.get())), 
        mSubQoS_(q.mSubQoS_ == nullptr ? nullptr : new rmw_qos_profile_t(*q.mSubQoS_.get())) {}

    /**
     * @brief Set the topic name.
     * @param[in] name The name of the topic.
     */
    void setTopicName(const std::string& name)
    {
        mTopicName_ = name;
    }

    /**
     * @brief Set the QoS profile for the publisher and subscriber.
     * @param[in] qos The QoS profile.
     */
    void setQoS(const rmw_qos_profile_t& qos)
    {
        if (mPubQoS_ == nullptr)
            mPubQoS_ = std::make_shared<rmw_qos_profile_t>(qos);
        else
            *mPubQoS_.get() = qos;
        if (mSubQoS_ == nullptr)
            mSubQoS_ = std::make_shared<rmw_qos_profile_t>(qos);
        else
            *mSubQoS_.get() = qos;
    }

    /**
     * @brief Set the QoS profile for the publisher.
     * @param[in] qos The QoS profile.
     */
    void setPubQoS(const rmw_qos_profile_t& qos)
    {
        if (mPubQoS_ == nullptr)
            mPubQoS_ = std::make_shared<rmw_qos_profile_t>(qos);
        else
            *mPubQoS_.get() = qos;
    }

    /**
     * @brief Set the QoS profile for the subscriber.
     * @param[in] qos The QoS profile.
     */
    void setSubQoS(const rmw_qos_profile_t& qos)
    {
        if (mSubQoS_ == nullptr)
            mSubQoS_ = std::make_shared<rmw_qos_profile_t>(qos);
        else
            *mSubQoS_.get() = qos;
    }

    /**
     * @brief Get the topic name.
     * @return The name of the topic.
     */
    std::string getTopicName() const
    {
        return mTopicName_;
    }

    /**
     * @brief Is topic name valid.
     * @return True if the topic name is valid; false otherwise.
     */
    bool isTopicNameValid() const
    {
        return mTopicName_ != "";
    }

    /**
     * @brief Is the QoS profile for the publisher valid.
     * @return True if the QoS profile is valid; false otherwise.
     */
    bool isPubQoSValid() const
    {
        return mPubQoS_ != nullptr;
    }

    /**
     * @brief Is the QoS profile for the subscriber valid.
     * @return True if the QoS profile is valid; false otherwise.
     */
    bool isSubQoSValid() const
    {
        return mSubQoS_ != nullptr;
    }

    std::string getErrInfo() const
    {
        return mErrInfo_;
    }

    /**
     * @brief Compare the QoS profiles.
     * @details The function compares the QoS profiles with the given QoS profile.
     * @param[in] q The QoS profile to compare.
     * @return True if the QoS profiles are the same; false otherwise.
     */
    bool operator==(const TopicQoS& q) const
    {
        if (mTopicName_ != q.mTopicName_)
            return false;

        bool myPubF = this->isPubQoSValid();
        bool mySubF = this->isSubQoSValid();
        bool qPubF = q.isPubQoSValid();
        bool qSubF = q.isSubQoSValid();
        bool pubF = false;
        bool subF = false;

        if (myPubF && qPubF)
            pubF = CompRMWQoS(*mPubQoS_.get(), *q.mPubQoS_.get()) == 0;
        else if (!myPubF && !qSubF)
            pubF = true;
        if (mySubF && qSubF)
            subF = CompRMWQoS(*mSubQoS_.get(), *q.mSubQoS_.get()) == 0;
        else if (!mySubF && !qSubF)
            subF = true;
        return pubF && subF;
    }

    /**
     * @brief Compare the QoS profiles.
     * @details The function compares the QoS profiles with the given QoS profile.
     * @param[in] q The QoS profile to compare.
     * @return 0 if the QoS profiles are the same; 
     * 1 if the publisher QoS profile is different; 
     * 2 if the subscriber QoS profile is different; 
     * 3 if both publisher and subscriber QoS profiles are different; 
     * -1 if the topic names are different.
     */
    TopicQoSCompareResult operator%(const TopicQoS& q) const
    {
        if (mTopicName_ != q.mTopicName_)
            return TopicQoSCompareResult::QOS_CMP_TOPIC_DIFF;

        bool myPubF = this->isPubQoSValid();
        bool mySubF = this->isSubQoSValid();
        bool qPubF = q.isPubQoSValid();
        bool qSubF = q.isSubQoSValid();
        bool pubF = false;
        bool subF = false;

        if (myPubF && qPubF)
            pubF = CompRMWQoS(*mPubQoS_.get(), *q.mPubQoS_.get()) == 0;
        else if (!myPubF && !qSubF)
            pubF = true;

        if (mySubF && qSubF)
            subF = CompRMWQoS(*mSubQoS_.get(), *q.mSubQoS_.get()) == 0;
        else if (!mySubF && !qSubF)
            subF = true;

        int ret = 0;
        if (pubF)
            ret += 1;
        if (subF)
            ret += 2;
        return (TopicQoSCompareResult)ret;
    }

    /**
     * @brief Get the QoS profile by the type QOS_TYPE_XXX under TopicQosProfile.msg.
     * @param[in] type The type of the QoS profile.
     * @return The QoS profile.
     * @note The function will throw TopicQoSException if the input type is invalid or the QoS profile is invalid.
     */
    const rmw_qos_profile_t& operator[](const uint8_t& type) const
    {
        if (type == msg::TopicQosProfile::QOS_TYPE_PUBLISHER)
        {
            if (mPubQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_PUB_QOS;
            return *mPubQoS_.get();
        }
        else if (type == msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION)
        {
            if (mSubQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_SUB_QOS;
            return *mSubQoS_.get();
        }
        else
            throw TopicQoSException::QOS_EXC_INVALID_QOS_TYPE;
    }

    /**
     * @brief Check TopicQoS is valid.
     * @return True if the TopicQoS is valid; false otherwise.
     */
    operator bool() const
    {
        if (mTopicName_ != "" && mPubQoS_ != nullptr && mSubQoS_ != nullptr)
        {
            rmw_qos_compatibility_type_t compType;
            char errInfo[2048];
            rmw_qos_profile_check_compatible(*mPubQoS_.get(), *mSubQoS_.get(), &compType, errInfo, 2048);
            mErrInfo_ = errInfo;
            return compType != RMW_QOS_COMPATIBILITY_ERROR;
        }
        return false;
    }
};

}


namespace std
{
    template <>
    struct less<rv2_interfaces::TopicQoS>
    {
        bool operator()(const rv2_interfaces::TopicQoS& lhs, const rv2_interfaces::TopicQoS& rhs) const
        {
            return lhs.getTopicName() < rhs.getTopicName();
        }
    };
}

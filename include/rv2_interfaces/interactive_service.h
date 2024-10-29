#pragma once
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

#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/utils.h>

#include <rv2_interfaces/msg/interactive_service.hpp>
#include <rv2_interfaces/msg/interactive_service_master_privilege.hpp>
#include <rv2_interfaces/srv/interactive_service.hpp>
#include <rv2_interfaces/srv/interactive_service_req.hpp>

#define InteractiveService_InteractiveServiceSrvName(serviceName) (std::string)serviceName
#define InteractiveService_InteractiveServiceReqSrvName(serviceName) (std::string)serviceName + "_Req"

// TODO: rename getNodeName to getServiceName.

namespace rv2_interfaces
{

enum InteractiveServiceMasterPrivilegeTargetAlive
{
    TARGET_ALIVE_NONE = 0, 
    TARGET_ALIVE_ENABLE_ONLY, 
    TARGET_ALIVE_DISABLE_ONLY, 
    TARGET_ALIVE_ALL
};

enum InteractiveServiceMasterPrivilegeTargetActivity
{
    TARGET_ACTIVITY_NONE = 0, 
    TARGET_ACTIVITY_ENABLE_ONLY, 
    TARGET_ACTIVITY_DISABLE_ONLY, 
    TARGET_ACTIVITY_ALL
};

/**
 * @brief Structure for master device privilege.
 * @details This structure stores the privilege for the master device, including the target alive privilege, target activity privilege, and service command privilege.
 */
struct InteractiveServiceMasterPrivilege
{
    std::string masterID;
    InteractiveServiceMasterPrivilegeTargetAlive targetAlive;// Target alive privilege.
    InteractiveServiceMasterPrivilegeTargetActivity targetActivity;// Target activity privilege.
    std::set<std::string> serviceCommandSet;// Service command privilege.
    bool requestInteractiveService;// Whether the master device can request the InteractiveService.
};

/**
 * @brief Check whether the master device has the privilege to change the target alive status.
 * @param[in] targetAliveSignal Target alive signal.
 * @param[in] masterPrivi Privilege for the master device.
 * @return True if the master device has the privilege, false otherwise.
 */
inline bool CheckMasterTargetAlivePrivilege(uint8_t targetAliveSignal, InteractiveServiceMasterPrivilege masterPrivi)
{
    switch (masterPrivi.targetAlive)
    {
        case InteractiveServiceMasterPrivilegeTargetAlive::TARGET_ALIVE_ENABLE_ONLY:
            if (targetAliveSignal == msg::InteractiveService::TARGET_ALIVE_ENABLE)
                return true;
            break;
        case InteractiveServiceMasterPrivilegeTargetAlive::TARGET_ALIVE_DISABLE_ONLY:
            if (targetAliveSignal == msg::InteractiveService::TARGET_ALIVE_DISABLE)
                return true;
            break;
        case InteractiveServiceMasterPrivilegeTargetAlive::TARGET_ALIVE_ALL:
            return true;
            break;
        default:
            return false;
    }
    return false;
}

/**
 * @brief Check whether the master device has the privilege to change the target activity status.
 * @param[in] targetActivitySignal Target activity signal.
 * @param[in] masterPrivi Privilege for the master device.
 * @return True if the master device has the privilege, false otherwise.
 */
inline bool CheckMasterTargetActivityPrivilege(uint8_t targetActivitySignal, InteractiveServiceMasterPrivilege masterPrivi)
{
    switch (masterPrivi.targetActivity)
    {
        case InteractiveServiceMasterPrivilegeTargetActivity::TARGET_ACTIVITY_ENABLE_ONLY:
            if (targetActivitySignal == msg::InteractiveService::TARGET_ACTIVITY_ENABLE)
                return true;
            break;
        case InteractiveServiceMasterPrivilegeTargetActivity::TARGET_ACTIVITY_DISABLE_ONLY:
            if (targetActivitySignal == msg::InteractiveService::TARGET_ACTIVITY_DISABLE)
                return true;
            break;
        case InteractiveServiceMasterPrivilegeTargetActivity::TARGET_ACTIVITY_ALL:
            return true;
            break;
        default:
            return false;
    }
    return false;
}

/**
 * @brief Check whether the master device has the privilege to run the service command.
 * @param[in] commandName Command name.
 * @param[in] masterPrivi Privilege for the master device.
 * @return True if the master device has the privilege, false otherwise.
 */
inline bool CheckMasterServiceCommandPrivilege(const std::string& commandName, InteractiveServiceMasterPrivilege masterPrivi)
{
    if (masterPrivi.serviceCommandSet.find(commandName) != masterPrivi.serviceCommandSet.end())
        return true;
    return false;
}

inline bool CheckMasterRequestInteractiveServicePrivilege(InteractiveServiceMasterPrivilege masterPrivi)
{
    return masterPrivi.requestInteractiveService;
}

inline msg::InteractiveServiceMasterPrivilege CvtInteractiveServiceMasterPrivilegeMsg(const InteractiveServiceMasterPrivilege& masterPrivi)
{
    msg::InteractiveServiceMasterPrivilege ret;
    ret.master_id = masterPrivi.masterID;
    ret.target_alive_privilege = masterPrivi.targetAlive;
    ret.target_activity_privilege = masterPrivi.targetActivity;
    for (const auto& command : masterPrivi.serviceCommandSet)
        ret.service_command.push_back(command);
    ret.request_interactive_service_privilege = masterPrivi.requestInteractiveService;
    return ret;
}

inline InteractiveServiceMasterPrivilege CvtInteractiveServiceMasterPrivilegeMsg(const msg::InteractiveServiceMasterPrivilege& masterPrivi)
{
    InteractiveServiceMasterPrivilege ret;
    ret.masterID = masterPrivi.master_id;
    ret.targetAlive = static_cast<InteractiveServiceMasterPrivilegeTargetAlive>(masterPrivi.target_alive_privilege);
    ret.targetActivity = static_cast<InteractiveServiceMasterPrivilegeTargetActivity>(masterPrivi.target_activity_privilege);
    for (const auto& command : masterPrivi.service_command)
        ret.serviceCommandSet.insert(command);
    ret.requestInteractiveService = masterPrivi.request_interactive_service_privilege;
    return ret;
}


/**
 * InteractiveService initialization property.
 * @details This structure stores the initialization properties for the InteractiveService.
 */
struct InteractiveServiceInitProp
{
    uint8_t targetAlive;
    uint8_t targetActivity;
    bool noInteract;
    bool useWhiteList;
    bool useReqServer;
    InteractiveServiceInitProp()
    {
        targetAlive = msg::InteractiveService::TARGET_ALIVE_ENABLE;
        targetActivity = msg::InteractiveService::TARGET_ACTIVITY_ENABLE;
        noInteract = false;
        useWhiteList = false;
        useReqServer = true;
    }
};



/**
 * InteractiveService class.
 * @details This class provides a service server for InteractiveService, which can be used to change the status of the target object and send commands to the target object.
 * @note The InteractiveService class is designed to be used as a base class for other classes, and the derived class should implement the callback functions for target alive event, target activity event, and service command.
 */
class InteractiveService
{
private:
    rclcpp::Service<srv::InteractiveService>::SharedPtr mInterSrv_;// Service server for InteractiveService.
    std::mutex mInterSrvMtx_;// Lock mInterSrv_ service callback function.

    std::function<bool(const std::string, const uint8_t)> mTargetAliveCbFunc_;// Callback function for target alive event.
    std::mutex mTargetAliveCbFuncMtx_;// Lock mTargetAliveCbFunc_ when setting or calling the callback function.
    std::atomic<bool> mTargetAliveCbFuncF_;// Flag indicating whether the callback function for target alive event is set.
    std::atomic<uint8_t> mTargetAlive_;// Store recent target alive status.

    std::function<bool(const std::string, const uint8_t)> mTargetActivityCbFunc_;// Callback function for target activity event.
    std::mutex mTargetActivityCbFuncMtx_;// Lock mTargetActivityCbFunc_ when setting or calling the callback function.
    std::atomic<bool> mTargetActivityCbFuncF_;// Flag indicating whether the callback function for target activity event is set.
    std::atomic<uint8_t> mTargetActivity_;// Store recent target activity status.

    std::map<std::string, std::function<bool(InteractiveService*, const std::string, const std::vector<std::string>)> > mServiceCommandFuncMap_;// Stored callback functions for each command.
    std::mutex mServiceCommandFuncMapMtx_;// Lock mServiceCommandFuncMap_.

    std::atomic<bool> mNoInteractF_;// Flag indicating whether the interactive method is disabled.
    std::atomic<bool> mReqServerF_;// Flag indicating whether to use InteractiveService request service server.

protected:
    // std::shared_ptr<rclcpp::Node> mParentNode_;// Parent node.
    rclcpp::Node *mParentNode_;// Parent node.
    const std::string mServiceName_;
    std::map<std::string, InteractiveServiceMasterPrivilege> mWhiteListMap_;// White list for master devices, including privileges for each master device.
    std::mutex mWhiteListMapMtx_;// Lock mWhiteListMap_.
    std::atomic<bool> mWhiteListMapF_;// Flag indicating whether to use white list.

    rclcpp::Service<srv::InteractiveServiceReq>::SharedPtr mInterReqSrv_;// Service server for InteractiveService request.
    std::mutex mInterReqSrvMtx_;// Lock mInterReqSrv_ service callback function.

private:
    /**
     * @brief Callback function for InteractiveService service server.
     * @note This function is using mWhiteListMapMtx_.
     * @note This function is using mServiceCommandFuncMapMtx_, mTargetAliveCbFuncMtx_ and mTargetActivityCbFuncMtx_.
     */
    void _interactiveServiceSrvCb(const std::shared_ptr<srv::InteractiveService::Request> request, 
                                std::shared_ptr<srv::InteractiveService::Response> response)
    {
        std::lock_guard<std::mutex> interServerLock(mInterSrvMtx_);
        response->response = true;
        if (mNoInteractF_)// If the interactive method is disabled, the function does nothing.
        {
            response->response = false;
            response->reason = "[Interaction disabled]";
            return;
        }

        auto whiteListMapCopy = SafeLoad(&mWhiteListMap_, mWhiteListMapMtx_);
        if (mWhiteListMapF_ && whiteListMapCopy.find(request->device_id) == whiteListMapCopy.end())// If the white list is used but the device ID is not found in the white list, the function does nothing.
        {
            response->response = false;
            response->reason = "[No privilege]";
            return;
        }

        {// Check target alive privilege and call the callback function.
            std::lock_guard<std::mutex> lock(mTargetAliveCbFuncMtx_);
            if (mTargetAliveCbFuncF_ && request->interactive_service.target_alive != msg::InteractiveService::TARGET_ALIVE_NONE)
            {
                if (mWhiteListMapF_ && !CheckMasterTargetAlivePrivilege(request->interactive_service.target_alive, whiteListMapCopy[request->device_id]))
                {
                    response->response = false;
                    response->reason = "[Target alive privilege denied]";
                }
                else
                    mTargetAlive_ = mTargetAliveCbFunc_(request->device_id, request->interactive_service.target_alive) ? request->interactive_service.target_alive : mTargetAlive_.load();
            }
        }

        {// Check target activity privilege and call the callback function.
            std::lock_guard<std::mutex> lock(mTargetActivityCbFuncMtx_);
            if (mTargetActivityCbFuncF_ && request->interactive_service.target_activity != msg::InteractiveService::TARGET_ACTIVITY_NONE)
            {
                if (mWhiteListMapF_ && !CheckMasterTargetActivityPrivilege(request->interactive_service.target_activity, whiteListMapCopy[request->device_id]))
                {
                    response->response = false;
                    response->reason = "[Target activity privilege denied]";
                }
                else
                    mTargetActivity_ = mTargetActivityCbFunc_(request->device_id, request->interactive_service.target_activity) ? request->interactive_service.target_activity : mTargetActivity_.load();
            }
        }

        if (request->service_command_name != "")
        {
            if (mWhiteListMapF_ && !CheckMasterServiceCommandPrivilege(request->service_command_name, whiteListMapCopy[request->device_id]))
            {
                response->response = false;
                response->reason = "[Service command privilege denied]";
            }
            else
            {
                std::lock_guard<std::mutex> lock(mServiceCommandFuncMapMtx_);
                if (mServiceCommandFuncMap_.find(request->service_command_name) != mServiceCommandFuncMap_.end())
                {
                    response->response = mServiceCommandFuncMap_[request->service_command_name](this, request->device_id, request->service_command_args);
                    if (!response->response)
                        response->reason = "[Service command returns false]";
                }
                else
                {
                    response->response = false;
                    response->reason = "[Service command not found]";
                }
            }
        }
    }

    /**
     * @brief Callback function for InteractiveServiceReq service server.
     * @note The callback function should be implemented in the derived class.
     * @note This function is using mWhiteListMapMtx_.
     */
    virtual void _interactiveServiceReqSrvCb(const std::shared_ptr<srv::InteractiveServiceReq::Request> request, 
                                            std::shared_ptr<srv::InteractiveServiceReq::Response> response)
    {
        std::lock_guard<std::mutex> interReqServerLock(mInterReqSrvMtx_);
        auto whiteListMapCopy = SafeLoad(&mWhiteListMap_, mWhiteListMapMtx_);
        if (mWhiteListMapF_ && whiteListMapCopy.find(request->device_id) == whiteListMapCopy.end())// If the white list is used but the device ID is not found in the white list, the function does nothing.
        {
            response->response = false;
            return;
        }
        if (mWhiteListMapF_ ? whiteListMapCopy[request->device_id].requestInteractiveService : true)
        {
            auto msg = this->getInteractiveServiceMsg();
            response->response = true;
            response->status = msg;
            return;
        }
        response->response = false;
    }

protected:
    /**
     * @brief Constructor of InteractiveService class. This constructor is protected and should only be called by the derived class.
     * @param[in] parentNode Parent node.
     * @param[in] serviceName Name of the services.
     * @param[in] prop Initialization property.
     * @param[in] isMultiF True if InteractiveService is inherited by MultiInteractiveService, false otherwise.
     * @note The service server name will be serviceName.
     */
    InteractiveService(rclcpp::Node *parentNode, const std::string& serviceName, InteractiveServiceInitProp prop, bool isMultiF) : 
        mServiceName_(serviceName), 
        mTargetAliveCbFuncF_(false), 
        mTargetAlive_(prop.targetAlive), 
        mTargetActivityCbFuncF_(false), 
        mTargetActivity_(prop.targetActivity), 
        mNoInteractF_(prop.noInteract), 
        mWhiteListMapF_(prop.useReqServer), 
        mReqServerF_(prop.useReqServer)
    {
        mParentNode_ = parentNode;

        mInterSrv_ = parentNode->create_service<srv::InteractiveService>(InteractiveService_InteractiveServiceSrvName(mServiceName_), 
            std::bind(&InteractiveService::_interactiveServiceSrvCb, this, std::placeholders::_1, std::placeholders::_2));

        if (mReqServerF_ && !isMultiF)// If the class inherited by MultiInteractiveService, the request server should be created in the MultiInteractiveService class.
            mInterReqSrv_ = parentNode->create_service<srv::InteractiveServiceReq>(InteractiveService_InteractiveServiceReqSrvName(mServiceName_), 
                std::bind(&InteractiveService::_interactiveServiceReqSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    }

public:
    /**
     * @brief Constructor of InteractiveService class.
     * @param[in] parentNode Parent node.
     * @param[in] serviceName Name of the services.
     * @param[in] prop Initialization property.
     * @note If useReqServer of prop set to true, InteractiveServiceReq service server will be created and named as serviceName + "_Req".
     */
    InteractiveService(rclcpp::Node *parentNode, const std::string& serviceName, InteractiveServiceInitProp prop) : 
        mServiceName_(serviceName), 
        mTargetAliveCbFuncF_(false), 
        mTargetAlive_(prop.targetAlive), 
        mTargetActivityCbFuncF_(false), 
        mTargetActivity_(prop.targetActivity), 
        mNoInteractF_(prop.noInteract), 
        mWhiteListMapF_(prop.useReqServer), 
        mReqServerF_(prop.useReqServer)
    {
        mInterSrv_ = parentNode->create_service<srv::InteractiveService>(InteractiveService_InteractiveServiceSrvName(mServiceName_), 
            std::bind(&InteractiveService::_interactiveServiceSrvCb, this, std::placeholders::_1, std::placeholders::_2));

        if (mReqServerF_)
            mInterReqSrv_ = parentNode->create_service<srv::InteractiveServiceReq>(InteractiveService_InteractiveServiceReqSrvName(mServiceName_), 
                std::bind(&InteractiveService::_interactiveServiceReqSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    }


    /**
     * @brief Set callback function for target alive event.
     * @param[in] func Function called when target alive event occurs.
     * @param[in] overwrite True to overwrite the callback function, false to do nothing if the callback function is already set.
     * @note The device ID and target alive status will be passed to the func, and the func should return whether the process is successful.
     * @note If return true, the target alive status will be updated to the new status. Otherwise, the target alive status will remain unchanged.
     * @note This function is using mTargetAliveCbFuncMtx_.
     */
    void setTargetAliveEventHandler(const std::function<bool(const std::string, const uint8_t)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(mTargetAliveCbFuncMtx_);
        if (mTargetAliveCbFuncF_ && !overwrite)// If the callback function is already set and overwrite is false, the function does nothing.
            return;
        mTargetAliveCbFunc_ = func;
        mTargetAliveCbFuncF_ = true;
    }

    /**
     * @brief Set callback function for target activity event.
     * @param[in] func Function called when target activity event occurs.
     * @param[in] overwrite True to overwrite the callback function, false to do nothing if the callback function is already set.
     * @note The device ID and target activity status will be passed to the func, and the func should return whether the process is successful.
     * @note If return true, the target activity status will be updated to the new status. Otherwise, the target activity status will remain unchanged.
     * @note This function is using mTargetActivityCbFuncMtx_.
     */
    void setTargetActivityEventHandler(const std::function<bool(const std::string, const uint8_t)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(mTargetActivityCbFuncMtx_);
        if (mTargetActivityCbFuncF_ && !overwrite)// If the callback function is already set and overwrite is false, the function does nothing.
            return;
        mTargetActivityCbFunc_ = func;
        mTargetActivityCbFuncF_ = true;
    }

    /**
     * @brief Add master privilege to the white list.
     * @param[in] masterPrivi Privilege for the master device.
     * @note The function will overwrite the privilege if the device ID is already in the white list.
     * @note This function is using mWhiteListMapMtx_.
     */
    virtual void addMasterPrivilege(const InteractiveServiceMasterPrivilege& masterPrivi)
    {
        std::lock_guard<std::mutex> lock(mWhiteListMapMtx_);
        mWhiteListMap_[masterPrivi.masterID] = masterPrivi;
    }

    /**
     * @brief Update master device privilege for service command.
     * @param[in] masterID Device ID of the master device.
     * @param[in] command Service command.
     * @param[in] add True to add the command to the privilege, false to remove the command from the privilege.
     * @note If the deviceID is found, the function will add or remove the command from the privilege.
     * @note This function is using mWhiteListMapMtx_.
     */
    void updateMasterDeviceCommand(const std::string& masterID, const std::string& command, bool add)
    {
        std::lock_guard<std::mutex> lock(mWhiteListMapMtx_);
        if (mWhiteListMap_.find(masterID) != mWhiteListMap_.end())
        {
            if (add)
                mWhiteListMap_[masterID].serviceCommandSet.insert(command);
            else
                mWhiteListMap_[masterID].serviceCommandSet.erase(command);
        }
    }

    /**
     * @brief Update master device privilege for service command set.
     * @param[in] masterID Device ID of the master device.
     * @param[in] commandSet Set of service commands.
     * @note The function will overwrite the command set if the deviceID is already in the white list.
     * @note This function is using mWhiteListMapMtx_.
     */
    void updateMasterDeviceCommandSet(const std::string& masterID, const std::set<std::string>& commandSet)
    {
        std::lock_guard<std::mutex> lock(mWhiteListMapMtx_);
        if (mWhiteListMap_.find(masterID) != mWhiteListMap_.end())
            mWhiteListMap_[masterID].serviceCommandSet = commandSet;
    }

    /**
     * @brief Add callback function for service command.
     * @param[in] command Command string.
     * @param[in] func Function called when the command event occurs. The arguments are InteractiveService pointer, device ID, and command arguments.
     * @note The device ID will be passed to the function.
     * @note This function is using mServiceCommandFuncMapMtx_.
     */
    void addServiceCommandEventHandler(const std::string& command, const std::function<bool(InteractiveService *iSrv, const std::string, const std::vector<std::string>)>& func)
    {
        std::lock_guard<std::mutex> lock(mServiceCommandFuncMapMtx_);
        mServiceCommandFuncMap_[command] = func;
    }

    /**
     * @brief Set whether enable the service interaction.
     */
    void noInteraction(bool flag)
    {
        mNoInteractF_ = !flag;
    }

    /**
     * @brief Set whether to use white list.
     */
    void useWhiteList(bool flag)
    {
        mWhiteListMapF_ = flag;
    }

    /**
     * @brief Set target alive status manually.
     */
    void setTargetAlive(uint8_t targetAlive)
    {
        mTargetAlive_ = targetAlive;
    }

    /**
     * @brief Set target activity status manually.
     */
    void setTargetActivity(uint8_t targetActivity)
    {
        mTargetActivity_ = targetActivity;
    }

    /**
     * @brief Get service name. TODO: rename to getNodeName.
     */
    std::string getNodeName() const
    {
        return mServiceName_;
    }

    /**
     * @brief Get target alive status.
     */
    uint8_t getTargetAlive()
    {
        return mTargetAlive_;
    }

    /**
     * @brief Get target activity status.
     */
    uint8_t getTargetActivity()
    {
        return mTargetActivity_;
    }

    /**
     * @brief Get InteractiveService message.
     * @return InteractiveService message.
     * @note This function is using mWhiteListMapMtx_.
     * @note The callback function should be implemented in the derived class.
     */
    virtual msg::InteractiveService getInteractiveServiceMsg()
    {
        msg::InteractiveService ret;
        ret.service_name = mServiceName_;
        ret.target_alive = mTargetAlive_;
        ret.target_activity = mTargetActivity_;
        ret.no_interaction = mNoInteractF_;
        ret.use_white_list = mWhiteListMapF_;
        auto whiteListMapCopy = SafeLoad(&mWhiteListMap_, mWhiteListMapMtx_);
        for (const auto& [master, privi] : whiteListMapCopy)
        {
            msg::InteractiveServiceMasterPrivilege masterPrivi;
            masterPrivi.master_id = privi.masterID;
            masterPrivi.target_alive_privilege = privi.targetAlive;
            masterPrivi.target_activity_privilege = privi.targetActivity;
            for (const auto& command : privi.serviceCommandSet)
                masterPrivi.service_command.push_back(command);
            masterPrivi.request_interactive_service_privilege = privi.requestInteractiveService;
            ret.white_list.push_back(masterPrivi);
        }
        return ret;
    }

    /**
     * @brief Call target alive callback function.
     * @param[in] deviceID Device ID of the master device.
     * @param[in] status Target alive status.
     * @return True if the callback function is set and the process is successful. Otherwise, the function returns false and the reason.
     */
    ReasonResult<bool> callTargetAliveCbFunc(std::string deviceID, uint8_t status)
    {
        auto req = std::make_shared<srv::InteractiveService::Request>();
        req->device_id = deviceID;
        req->interactive_service.target_alive = status;
        auto res = std::make_shared<srv::InteractiveService::Response>();
        this->_interactiveServiceSrvCb(req, res);
        return { res->response, res->reason };
    }

    /**
     * @brief Call target activity callback function.
     * @param[in] deviceID Device ID of the master device.
     * @param[in] status Target activity status.
     * @return True if the callback function is set and the process is successful. Otherwise, the function returns false and the reason.
     */
    ReasonResult<bool> callTargetActivityCbFunc(std::string deviceID, uint8_t status)
    {
        auto req = std::make_shared<srv::InteractiveService::Request>();
        req->device_id = deviceID;
        req->interactive_service.target_activity = status;
        auto res = std::make_shared<srv::InteractiveService::Response>();
        this->_interactiveServiceSrvCb(req, res);
        return { res->response, res->reason };
    }
};



/**
 * Multi-InteractiveService class.
 * @details This class derives from the InteractiveService class and provides additional functions to store the target alive status and target activity status for each master device.
 */
class MultiInteractiveService : public InteractiveService
{
private:
    std::map<std::string, uint8_t> mTargetAliveMap_;// Store target alive status for each master device.
    std::mutex mTargetAliveMapMtx_;// Lock mTargetAliveMap_.

    std::map<std::string, uint8_t> mTargetActivityMap_;// Store target activity status for each master device.
    std::mutex mTargetActivityMapMtx_;// Lock mTargetActivityMap_.

private:
    /**
     * @brief Callback function for InteractiveServiceReq service server.
     * @note This function is implemented in the derived class.
     * @note This function is using mWhiteListMapMtx_, mTargetAliveMapMtx_ and mTargetActivityMapMtx_.
     */
    void _interactiveServiceReqSrvCb(const std::shared_ptr<srv::InteractiveServiceReq::Request> request, 
                                            std::shared_ptr<srv::InteractiveServiceReq::Response> response) override
    {
        std::lock_guard<std::mutex> interReqServerLock(mInterReqSrvMtx_);
        auto whiteListMapCopy = SafeLoad(&mWhiteListMap_, mWhiteListMapMtx_);
        if (mWhiteListMapF_ && whiteListMapCopy.find(request->device_id) == whiteListMapCopy.end())// If the white list is used but the device ID is not found in the white list, the function does nothing.
        {
            response->response = false;
            return;
        }
        if (mWhiteListMapF_ ? whiteListMapCopy[request->device_id].requestInteractiveService : true)
        {
            auto msg = this->getInteractiveServiceMsg();
            response->response = true;
            response->status = msg;
            return;
        }
        response->response = false;
    }

public:
    /**
     * @brief Constructor of MultiInteractiveService class.
     * @param[in] parentNode Parent node.
     * @param[in] serviceName Name of the services.
     * @param[in] prop Initialization property.
     * @note If useReqServer of prop set to true,  InteractiveServiceReq service server will be created and named as serviceName + "_Req".
     */
    MultiInteractiveService(rclcpp::Node *parentNode, const std::string& serviceName, InteractiveServiceInitProp prop) : 
        InteractiveService(parentNode, serviceName, prop, true)
    {
        if (prop.useReqServer)
            mInterReqSrv_ = parentNode->create_service<srv::InteractiveServiceReq>(InteractiveService_InteractiveServiceReqSrvName(mServiceName_), 
                std::bind(&MultiInteractiveService::_interactiveServiceReqSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief Add master privilege to the white list.
     * @param[in] masterPrivi Privilege for the master device.
     * @note The function will overwrite the privilege if the device ID is already in the white list.
     * @note The function will add TARGET_ALIVE_NONE and TARGET_ACTIVITY_NONE to the status map for the master device by default.
     * @note If user wants to change the status of the master device, call addTargetAlive() or addTargetActivity() for target_alive and target_activity status respectively.
     * @note This function is using mWhiteListMapMtx_, mTargetAliveMapMtx_ and mTargetActivityMapMtx_.
     */
    void addMasterPrivilege(const InteractiveServiceMasterPrivilege& masterPrivi) override
    {
        std::lock_guard<std::mutex> lock(mWhiteListMapMtx_);
        mWhiteListMap_[masterPrivi.masterID] = masterPrivi;
        this->addTargetAlive(masterPrivi.masterID, msg::InteractiveService::TARGET_ALIVE_NONE);
        this->addTargetActivity(masterPrivi.masterID, msg::InteractiveService::TARGET_ACTIVITY_NONE);
    }

    /**
     * @brief Add target status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetAlive Target alive status.
     * @param[in] targetActivity Target activity status.
     * @note This function is using mTargetAliveMapMtx_ and mTargetActivityMapMtx_.
     */
    void addMasterTargetStatus(const std::string& master, uint8_t targetAlive, uint8_t targetActivity)
    {
        this->addTargetAlive(master, targetAlive);
        this->addTargetActivity(master, targetActivity);
    }

    /**
     * @brief Add target alive status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetAlive Target alive status.
     * @note This function is using mTargetAliveMapMtx_.
     */
    void addTargetAlive(const std::string& master, uint8_t targetAlive)
    {
        std::lock_guard<std::mutex> lock(mTargetAliveMapMtx_);
        mTargetAliveMap_[master] = targetAlive;
    }

    /**
     * @brief Modify target alive status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetAlive Target alive status.
     * @note If the master is not found in the map, the function does nothing.
     * @note This function is using mTargetAliveMapMtx_.
     */
    void modifyTargetAlive(const std::string& master, uint8_t targetAlive)
    {
        std::lock_guard<std::mutex> lock(mTargetAliveMapMtx_);
        if (mTargetAliveMap_.find(master) != mTargetAliveMap_.end())
            mTargetAliveMap_[master] = targetAlive;
    }

    /**
     * @brief Add target activity status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetActivity Target activity status.
     * @note This function is using mTargetActivityMapMtx_.
     */
    void addTargetActivity(const std::string& master, uint8_t targetActivity)
    {
        std::lock_guard<std::mutex> lock(mTargetActivityMapMtx_);
        mTargetActivityMap_[master] = targetActivity;
    }

    /**
     * @brief Modify target activity status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetActivity Target activity status.
     * @note If the master is not found in the map, the function does nothing.
     * @note This function is using mTargetActivityMapMtx_.
     */
    void modifyTargetActivity(const std::string& master, uint8_t targetActivity)
    {
        std::lock_guard<std::mutex> lock(mTargetActivityMapMtx_);
        if (mTargetActivityMap_.find(master) != mTargetActivityMap_.end())
            mTargetActivityMap_[master] = targetActivity;
    }

    /**
     * @brief Get target alive status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[out] outTargetAlive Output target alive status.
     * @return True if the master is found in the map. Otherwise, the function returns false.
     * @note This function is using mTargetAliveMapMtx_.
     */
    bool getTargetAlive(const std::string& master, uint8_t& outTargetAlive)
    {
        std::lock_guard<std::mutex> lock(mTargetAliveMapMtx_);
        if (mTargetAliveMap_.find(master) != mTargetAliveMap_.end())
        {
            outTargetAlive = mTargetAliveMap_[master];
            return true;
        }
        return false;
    }

    /**
     * @brief Get target alive status map.
     * @return Target alive status map.
     * @note This function is using mTargetAliveMapMtx_.
     */
    std::map<std::string, uint8_t> getAllTargetAlive()
    {
        std::lock_guard<std::mutex> lock(mTargetAliveMapMtx_);
        return mTargetAliveMap_;
    }

    /**
     * @brief Get target activity status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[out] outTargetActivity Output target activity status.
     * @return True if the master is found in the map. Otherwise, the function returns false.
     * @note This function is using mTargetActivityMapMtx_.
     */
    bool getTargetActivity(const std::string& master, uint8_t& outTargetActivity)
    {
        std::lock_guard<std::mutex> lock(mTargetActivityMapMtx_);
        if (mTargetActivityMap_.find(master) != mTargetActivityMap_.end())
        {
            outTargetActivity = mTargetActivityMap_[master];
            return true;
        }
        return false;
    }

    /**
     * @brief Get target activity status map.
     * @return Target activity status map.
     * @note This function is using mTargetActivityMapMtx_.
     */
    std::map<std::string, uint8_t> getAllTargetActivity()
    {
        std::lock_guard<std::mutex> lock(mTargetActivityMapMtx_);
        return mTargetActivityMap_;
    }

    /**
     * @brief Check all target alive status.
     * @return Target alive status.
     * @note If there are multiple target alive status, the function returns the following value:
     * @note - If there is at least one "enable" status and no "disable" status, the function returns "enable".
     * @note - If there is at least one "disable" status, the function returns "disable".
     * @note - Otherwise, the function returns "none".
     * @note This function is using mTargetAliveMapMtx_.
     */
    uint8_t checkAllTargetAlive()
    {
        std::lock_guard<std::mutex> lock(mTargetAliveMapMtx_);
        size_t targetAliveEnableCount = 0;
        size_t targetAliveDisableCount = 0;
        size_t targetAliveUnknownCount = 0;
        for (auto& i : mTargetAliveMap_)
        {
            switch (i.second)
            {
                case msg::InteractiveService::TARGET_ALIVE_ENABLE:
                    targetAliveEnableCount++;
                    break;
                case msg::InteractiveService::TARGET_ALIVE_DISABLE:
                    targetAliveDisableCount++;
                    break;
                default:
                    targetAliveUnknownCount++;
                    break;
            }
        }
        return (targetAliveEnableCount > 0 && targetAliveDisableCount == 0) ? msg::InteractiveService::TARGET_ALIVE_ENABLE : 
                (targetAliveDisableCount > 0) ? msg::InteractiveService::TARGET_ALIVE_DISABLE : 
                msg::InteractiveService::TARGET_ALIVE_NONE;
    }

    /**
     * @brief Check all target activity status.
     * @return Target activity status.
     * @note If there are multiple target activity status, the function returns the following value:
     * @note - If there is at least one "enable" status and no "disable" status, the function returns "enable".
     * @note - If there is at least one "disable" status, the function returns "disable".
     * @note - Otherwise, the function returns "none".
     * @note This function is using mTargetActivityMapMtx_.
     */
    uint8_t checkAllTargetActivity()
    {
        std::lock_guard<std::mutex> lock(mTargetActivityMapMtx_);
        size_t targetActivityEnableCount = 0;
        size_t targetActivityDisableCount = 0;
        size_t targetActivityUnknownCount = 0;
        for (auto& i : mTargetActivityMap_)
        {
            switch (i.second)
            {
                case msg::InteractiveService::TARGET_ACTIVITY_ENABLE:
                    targetActivityEnableCount++;
                    break;
                case msg::InteractiveService::TARGET_ACTIVITY_DISABLE:
                    targetActivityDisableCount++;
                    break;
                default:
                    targetActivityUnknownCount++;
                    break;
            }
        }
        return (targetActivityEnableCount > 0 && targetActivityDisableCount == 0) ? msg::InteractiveService::TARGET_ACTIVITY_ENABLE : 
                (targetActivityDisableCount > 0) ? msg::InteractiveService::TARGET_ACTIVITY_DISABLE : 
                msg::InteractiveService::TARGET_ACTIVITY_NONE;
    }

    msg::InteractiveService getInteractiveServiceMsg() override
    {
        auto msg = InteractiveService::getInteractiveServiceMsg();
        msg.is_multi = true;
        for (const auto& [master, alive] : this->getAllTargetAlive())
        {
            msg.target_alive_master_vec.push_back(master);
            msg.target_alive_vec.push_back(alive);
        }

        for (const auto& [master, activity] : this->getAllTargetActivity())
        {
            msg.target_activity_master_vec.push_back(master);
            msg.target_activity_vec.push_back(activity);
        }
        return msg;
    }
};

}

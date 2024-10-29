#pragma once
#include <rv2_interfaces/interactive_service.h>
#include <rv2_interfaces/msg/topic_device_info.hpp>
#include <rv2_interfaces/msg/topic_qos_profile.hpp>

namespace rv2_interfaces
{

/**
 * Interactive Methods.
 */

/**
 * @brief InteractiveServiceWithQoS inherits from InteractiveService, and is the base class of InteractivePublisher and InteractiveSubscription.
 * @details The class provides the following functionalities:
 * 1. Set and get Quality of Service.
 * 2. Get topic name and topic type.
 */
class InteractiveServiceWithQoS : public InteractiveService
{
private:
    const uint8_t mQoSType_;// Type of the qos. See rv2_interfaces/msg/TopicQosProfile.msg.
    std::shared_ptr<rclcpp::QoS> mQoS_;// Quality of Service.
    std::mutex mQoSMtx_;// Lock mQoS_.

protected:
    const std::string mTopicName_;// Name of the topic.

public:
    InteractiveServiceWithQoS(rclcpp::Node *parentNode, 
                                const std::string& serviceName, 
                                const std::string& topicName, 
                                const rclcpp::QoS& qos, 
                                const uint8_t type, 
                                const InteractiveServiceInitProp& prop) : 
        InteractiveService(parentNode, serviceName, prop), 
        mTopicName_(topicName), 
        mQoSType_(type) { this->setQoS(qos); }

    /**
     * @brief Set Quality of Service.
     * @param[in] qos Quality of Service.
     * @note This function is using mQoSMtx_.
     */
    void setQoS(const rclcpp::QoS& qos)
    {
        std::lock_guard<std::mutex> lock(mQoSMtx_);
        mQoS_ = std::make_shared<rclcpp::QoS>(qos);
    }

    /**
     * @brief Get Quality of Service.
     * @return Quality of Service.
     * @note This function is using mQoSMtx_.
     */
    rclcpp::QoS getQoS()
    {
        std::lock_guard<std::mutex> lock(mQoSMtx_);
        return *mQoS_;
    }

    std::string getTopicName() const
    {
        return mTopicName_;
    }

    // See rv2_interfaces/msg/TopicQosProfile.msg.
    uint8_t getQoSType() const
    {
        return mQoSType_;
    }

    msg::TopicDeviceInfo getTopicDeviceInfo()
    {
        msg::TopicDeviceInfo ret;
        ret.node_name = mServiceName_;
        ret.topic_name = mTopicName_;
        ret.qos_type = mQoSType_;
        return ret;
    }
};







/**
 * MultiInteractive Methods.
 */

/**
 * @brief MultiInteractiveServiceWithQoS inherits from MultiInteractiveService, and is the base class of MultiInteractivePublisher and MultiInteractiveSubscription.
 * @details The class provides the following functionalities:
 * 1. Set and get Quality of Service.
 * 2. Get topic name and topic type.
 * 3. Check all master devices' target alive and target activity status.
 * 4. Set target alive enable and disable event handlers. Which event will be raised was determined by checkAllTargetAlive().
 * 5. Set target activity enable and disable event handlers. Which event will be raised was determined by checkAllTargetActivity().
 */
class MultiInteractiveServiceWithQoS : public MultiInteractiveService
{
private:
    const uint8_t mQoSType_;// Type of the topic.
    std::shared_ptr<rclcpp::QoS> mQoS_;// Quality of Service.
    std::mutex mQoSMtx_;// Lock mQoS_.

    std::function<void(void)> mTargetAliveEnableCbFunc_;// Target activity enable callback function.
    std::mutex mTargetAliveEnableCbFuncMtx_;// Lock mTargetAliveEnableCbFunc_.
    std::atomic<bool> mTargetAliveEnableCbFuncF_;// Flag indicating whether the target activity enable callback function is set.
    std::function<void(void)> mTargetAliveDisableCbFunc_;// Target activity disable callback function.
    std::mutex mTargetAliveDisableCbFuncMtx_;// Lock mTargetAliveDisableCbFunc_.
    std::atomic<bool> mTargetAliveDisableCbFuncF_;// Flag indicating whether the target activity disable callback function is set.

    std::function<void(void)> mTargetActivityEnableCbFunc_;// Target activity enable callback function.
    std::mutex mTargetActivityEnableCbFuncMtx_;// Lock mTargetActivityEnableCbFunc_.
    std::atomic<bool> mTargetActivityEnableCbFuncF_;// Flag indicating whether the target activity enable callback function is set.
    std::function<void(void)> mTargetActivityDisableCbFunc_;// Target activity disable callback function.
    std::mutex mTargetActivityDisableCbFuncMtx_;// Lock mTargetActivityDisableCbFunc_.
    std::atomic<bool> mTargetActivityDisableCbFuncF_;// Flag indicating whether the target activity disable callback function is set.

protected:
    const std::string mTopicName_;// Name of the topic.

private:
    /**
     * @brief Target alive event handler.
     * @details The function is called when the interactive service server receives a target alive request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetAlive Requested target alive status.
     * @return For MultiInteractivePublisher, the function returns false.
     * @note This function is using targetAliveMapLock_, mTargetAliveEnableCbFuncMtx_ and mTargetAliveDisableCbFuncMtx_.
     */
    bool _targetAliveEventHandler(const std::string deviceID, const uint8_t targetAlive)
    {
        this->modifyTargetAlive(deviceID, targetAlive);
        uint8_t ret = this->checkAllTargetAlive();
        this->setTargetAlive(ret);

        std::lock_guard<std::mutex> targetAliveEnableCbFuncLock(mTargetAliveEnableCbFuncMtx_);
        std::lock_guard<std::mutex> targetAliveDisableCbFuncLock(mTargetAliveDisableCbFuncMtx_);

        if (mTargetAliveEnableCbFuncF_ && ret == msg::InteractiveService::TARGET_ALIVE_ENABLE)
            mTargetAliveEnableCbFunc_();
        else if (mTargetAliveDisableCbFuncF_ && ret == msg::InteractiveService::TARGET_ALIVE_DISABLE)
            mTargetAliveDisableCbFunc_();
        return false;
    }

    /**
     * @brief Target activity event handler.
     * @details The function is called when the interactive service server receives a target activity request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetActivity Requested target activity status.
     * @return For MultiInteractivePublisher, the function returns false.
     * @note This function is using targetActivityMapLock_, mTargetActivityEnableCbFuncMtx_ and mTargetActivityDisableCbFuncMtx_.
     */
    bool _targetActivityEventHandler(const std::string deviceID, const uint8_t targetActivity)
    {
        this->modifyTargetActivity(deviceID, targetActivity);
        uint8_t ret = this->checkAllTargetActivity();
        this->setTargetActivity(ret);

        std::lock_guard<std::mutex> targetActivityEnableCbFuncLock(mTargetActivityEnableCbFuncMtx_);
        std::lock_guard<std::mutex> targetActivityDisableCbFuncLock(mTargetActivityDisableCbFuncMtx_);

        if (mTargetActivityEnableCbFuncF_ && ret == msg::InteractiveService::TARGET_ACTIVITY_ENABLE)
            mTargetActivityEnableCbFunc_();
        else if (mTargetActivityDisableCbFuncF_ && ret == msg::InteractiveService::TARGET_ACTIVITY_DISABLE)
            mTargetActivityDisableCbFunc_();
        return false;
    }

public:
    MultiInteractiveServiceWithQoS(rclcpp::Node *parentNode, 
                                    const std::string& serviceName, 
                                    const std::string& topicName, 
                                    const rclcpp::QoS& qos, 
                                    const uint8_t type, 
                                    const InteractiveServiceInitProp& prop) : 
        MultiInteractiveService(parentNode, serviceName, prop), 
        mTopicName_(topicName), 
        mQoSType_(type), 
        mTargetAliveEnableCbFuncF_(false), 
        mTargetAliveDisableCbFuncF_(false), 
        mTargetActivityEnableCbFuncF_(false), 
        mTargetActivityDisableCbFuncF_(false)
    {
        this->setQoS(qos);
        this->setTargetAliveEventHandler(std::bind(&MultiInteractiveServiceWithQoS::_targetAliveEventHandler, this, std::placeholders::_1, std::placeholders::_2));
        this->setTargetActivityEventHandler(std::bind(&MultiInteractiveServiceWithQoS::_targetActivityEventHandler, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief Set target alive enable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using mTargetAliveEnableCbFuncMtx_.
     */
    void setTargetAliveEnableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(mTargetAliveEnableCbFuncMtx_);
        if (mTargetAliveEnableCbFuncF_ && !overwrite)
            return;
        mTargetAliveEnableCbFunc_ = func;
        mTargetAliveEnableCbFuncF_ = true;
    }

    /**
     * @brief Set target alive disable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using mTargetAliveDisableCbFuncMtx_.
     */
    void setTargetAliveDisableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(mTargetAliveDisableCbFuncMtx_);
        if (mTargetAliveDisableCbFuncF_ && !overwrite)
            return;
        mTargetAliveDisableCbFunc_ = func;
        mTargetAliveDisableCbFuncF_ = true;
    }

    /**
     * @brief Set target activity enable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using mTargetActivityEnableCbFuncMtx_.
     */
    void setTargetActivityEnableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(mTargetActivityEnableCbFuncMtx_);
        if (mTargetActivityEnableCbFuncF_ && !overwrite)
            return;
        mTargetActivityEnableCbFunc_ = func;
        mTargetActivityEnableCbFuncF_ = true;
    }

    /**
     * @brief Set target activity disable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using mTargetActivityDisableCbFuncMtx_.
     */
    void setTargetActivityDisableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(mTargetActivityDisableCbFuncMtx_);
        if (mTargetActivityDisableCbFuncF_ && !overwrite)
            return;
        mTargetActivityDisableCbFunc_ = func;
        mTargetActivityDisableCbFuncF_ = true;
    }

    /**
     * @brief Set Quality of Service.
     * @param[in] qos Quality of Service.
     * @note This function is using mQoSMtx_.
     */
    void setQoS(const rclcpp::QoS& qos)
    {
        std::lock_guard<std::mutex> lock(mQoSMtx_);
        mQoS_ = std::make_shared<rclcpp::QoS>(qos);
    }

    /**
     * @brief Get Quality of Service.
     * @return Quality of Service.
     * @note This function is using mQoSMtx_.
     */
    rclcpp::QoS getQoS()
    {
        std::lock_guard<std::mutex> lock(mQoSMtx_);
        return *mQoS_;
    }

    std::string getTopicName() const
    {
        return mTopicName_;
    }

    // See rv2_interfaces/msg/TopicQosProfile.msg.
    uint8_t getQoSType() const
    {
        return mQoSType_;
    }

    msg::TopicDeviceInfo getTopicDeviceInfo()
    {
        msg::TopicDeviceInfo ret;
        ret.node_name = mServiceName_;
        ret.topic_name = mTopicName_;
        ret.qos_type = mQoSType_;
        return ret;
    }
};

}

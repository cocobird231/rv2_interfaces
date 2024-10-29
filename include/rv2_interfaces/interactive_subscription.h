#pragma once
#include <rv2_interfaces/interactive_service_qos.h>

namespace rv2_interfaces
{

/**
 * @brief InteractiveSubscription inherits from InteractiveServiceWithQoS.
 * @note InteractiveSubscription class can be controlled by any master device which has the privilege to control the subscription.
 * @note The class provides a controlable subscription with following functionalities:
 * @note 1. Create and destroy subscription by requesting target alive event with enable and disable status.
 */
template <typename T>
class InteractiveSubscription : public InteractiveServiceWithQoS
{
private:
    std::shared_ptr<rclcpp::Subscription<T> > mSub_;// Subscription.
    std::mutex mSubMtx_;// Lock mSub_.
    std::atomic<bool> mSubscriptionF_;// Flag indicating whether the subscription is created.

    std::function<void(const std::shared_ptr<T>)> mSubCbFunc_;// Subscription callback function.

private:
    /**
     * @brief Target alive event handler.
     * @details The function is called when the interactive service server receives a target alive request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetAlive Requested target alive status.
     * @return Wehther the event is handled successfully. If return true, the target alive status will be updated. Otherwise, the target alive status will remain unchanged.
     * @note This function is using mSubMtx_.
     */
    bool _targetAliveEventHandler(const std::string deviceID, const uint8_t targetAlive)
    {
        if (targetAlive == msg::InteractiveService::TARGET_ALIVE_ENABLE)
        {
            std::lock_guard<std::mutex> lock(mSubMtx_);
            if (mSub_.use_count() <= 0)
                mSub_ = mParentNode_->create_subscription<T>(mTopicName_, this->getQoS(), mSubCbFunc_);
            mSubscriptionF_ = true;
        }
        else if (targetAlive == msg::InteractiveService::TARGET_ALIVE_DISABLE)
        {
            std::lock_guard<std::mutex> lock(mSubMtx_);
            if (mSub_.use_count() > 0)
                mSub_.reset();
            mSubscriptionF_ = false;
        }
        else
            return false;
        return true;
    }

public:
    /**
     * @brief Constructor of InteractiveSubscription class.
     * @param[in] parentNode Parent node.
     * @param[in] serviceName Name of the interactive service.
     * @param[in] topicName Name of the topic.
     * @param[in] qos Quality of Service for the subscription.
     * @param[in] callback Subscription callback function.
     * @param[in] prop InteractiveService initialization properties.
     * @note The interactive service will be created and named as serviceName.
     */
    InteractiveSubscription(rclcpp::Node *parentNode, 
                            const std::string& serviceName, 
                            const std::string& topicName, 
                            const rclcpp::QoS& qos, 
                            std::function<void(const std::shared_ptr<T>)> callback, 
                            InteractiveServiceInitProp prop = InteractiveServiceInitProp()) : 
        InteractiveServiceWithQoS(parentNode, serviceName, topicName, qos, msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION, prop), 
        mSubCbFunc_(callback), 
        mSubscriptionF_(false)
    {
        if (prop.targetAlive == msg::InteractiveService::TARGET_ALIVE_ENABLE)
        {
            mSub_ = mParentNode_->create_subscription<T>(mTopicName_, this->getQoS(), mSubCbFunc_);
            mSubscriptionF_ = true;
        }

        // Set target alive and target activity event handlers.
        this->setTargetAliveEventHandler(std::bind(&InteractiveSubscription::_targetAliveEventHandler, this, std::placeholders::_1, std::placeholders::_2));
    }
};



/**
 * @brief MultiInteractiveSubscription inherits from MultiInteractiveServiceWithQoS.
 * @note MultiInteractiveSubscription class can be controlled by any master device which has the privilege to control the subscription.
 * @note The class provides a controlable subscription with following functionalities:
 * @note 1. Create and destroy subscription by requesting target alive event with enable and disable status.
 * @note The target event will be raised by checking all master devices' target alive status.
 */
template <typename T>
class MultiInteractiveSubscription : public MultiInteractiveServiceWithQoS
{
private:
    std::shared_ptr<rclcpp::Subscription<T> > mSub_;// Subscription.
    std::mutex mSubMtx_;// Lock mSub_.
    std::atomic<bool> mSubscriptionF_;// Flag indicating whether the subscription is created.

    std::function<void(const std::shared_ptr<T>)> mSubCbFunc_;// Subscription callback function.

private:
    /**
     * @brief Target alive enable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to enable.
     * @note This function is using mSubMtx_.
     */
    void _targetAliveEnableCbFunc()
    {
        std::lock_guard<std::mutex> lock(mSubMtx_);
        if (mSub_.use_count() <= 0)
            mSub_ = mParentNode_->create_subscription<T>(mTopicName_, this->getQoS(), mSubCbFunc_);
        mSubscriptionF_ = true;
    }

    /**
     * @brief Target alive disable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to disable.
     * @note This function is using mSubMtx_.
     */
    void _targetAliveDisableCbFunc()
    {
        std::lock_guard<std::mutex> lock(mSubMtx_);
        if (mSub_.use_count() > 0)
            mSub_.reset();
        mSubscriptionF_ = false;
    }

public:
    /**
     * @brief Constructor of MultiInteractiveSubscription class.
     * @param[in] parentNode Parent node.
     * @param[in] serviceName Name of the node.
     * @param[in] topicName Name of the topic.
     * @param[in] qos Quality of Service for the subscription.
     * @param[in] callback Subscription callback function.
     * @param[in] prop InteractiveService initialization properties.
     * @note The interactive service will be created and named as serviceName.
     */
    MultiInteractiveSubscription(rclcpp::Node *parentNode, 
                                    const std::string& serviceName, 
                                    const std::string& topicName, 
                                    const rclcpp::QoS& qos, 
                                    std::function<void(const std::shared_ptr<T>)> callback, 
                                    InteractiveServiceInitProp prop = InteractiveServiceInitProp()) : 
        MultiInteractiveServiceWithQoS(parentNode, serviceName, topicName, qos, msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION, prop), 
        mSubCbFunc_(callback), 
        mSubscriptionF_(false)
    {
        if (prop.targetAlive == msg::InteractiveService::TARGET_ALIVE_ENABLE)
        {
            mSub_ = mParentNode_->create_subscription<T>(mTopicName_, this->getQoS(), mSubCbFunc_);
            mSubscriptionF_ = true;
        }

        // Set target alive event handlers.
        this->setTargetAliveEnableEventHandler(std::bind(&MultiInteractiveSubscription::_targetAliveEnableCbFunc, this));
        this->setTargetAliveDisableEventHandler(std::bind(&MultiInteractiveSubscription::_targetAliveDisableCbFunc, this));
    }
};

}

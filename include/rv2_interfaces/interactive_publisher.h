#pragma once
#include <rv2_interfaces/interactive_service_qos.h>

namespace rv2_interfaces
{

/**
 * @brief InteractivePublisher inherits from InteractiveServiceWithQoS. 
 * @note InteractivePublisher class can be controlled by any master device which has the privilege to control the publisher.
 * @note The class provides a controlable publisher with following functionalities:
 * @note 1. Create and destroy publisher by requesting target alive event with enable and disable status.
 * @note 2. Enable and disable publishing by requesting target activity event with enable and disable status.
 */
template <typename T>
class InteractivePublisher : public InteractiveServiceWithQoS
{
private:
    std::shared_ptr<rclcpp::Publisher<T> > mPub_;// Publisher.
    std::mutex mPubMtx_;// Lock mPub_.
    std::atomic<bool> mPublisherF_;// Flag indicating whether the publisher is created.
    std::atomic<bool> mPublishingF_;// Flag indicating whether the publishing is enabled.

private:
    /**
     * @brief Target alive event handler.
     * @details The function is called when the interactive service server receives a target alive request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetAlive Requested target alive status.
     * @return Wehther the event is handled successfully. If return true, the target alive status will be updated. Otherwise, the target alive status will remain unchanged.
     * @note This function is using mPubMtx_.
     */
    bool _targetAliveEventHandler(const std::string deviceID, const uint8_t targetAlive)
    {
        if (targetAlive == msg::InteractiveService::TARGET_ALIVE_ENABLE)
        {
            std::lock_guard<std::mutex> lock(mPubMtx_);
            if (mPub_.use_count() <= 0)
                mPub_ = mParentNode_->create_publisher<T>(mTopicName_, this->getQoS());
            mPublisherF_ = true;
        }
        else if (targetAlive == msg::InteractiveService::TARGET_ALIVE_DISABLE)
        {
            std::lock_guard<std::mutex> lock(mPubMtx_);
            if (mPub_.use_count() > 0)
                mPub_.reset();
            mPublisherF_ = false;
        }
        else
            return false;
        return true;
    }

    /**
     * @brief Target activity event handler.
     * @details The function is called when the interactive service server receives a target activity request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetActivity Requested target activity status.
     * @return Wehther the event is handled successfully. If return true, the target activity status will be updated. Otherwise, the target activity status will remain unchanged.
     */
    bool _targetActivityEventHandler(const std::string deviceID, const uint8_t targetActivity)
    {
        if (targetActivity == msg::InteractiveService::TARGET_ACTIVITY_ENABLE)
        {
            mPublishingF_ = true;
        }
        else if (targetActivity == msg::InteractiveService::TARGET_ACTIVITY_DISABLE)
        {
            mPublishingF_ = false;
        }
        else
            return false;
        return true;
    }

public:
    /**
     * @brief Constructor of InteractivePublisher class.
     * @param[in] parentNode Parent node.
     * @param[in] serviceName Name of the interactive service.
     * @param[in] topicName Name of the topic.
     * @param[in] qos Quality of Service for the publisher.
     * @param[in] prop Interactive service initialization properties.
     * @note The interactive service will be created and named as serviceName.
     */
    InteractivePublisher(rclcpp::Node *parentNode, 
                            const std::string& serviceName, 
                            const std::string& topicName, 
                            const rclcpp::QoS& qos, 
                            InteractiveServiceInitProp prop = InteractiveServiceInitProp()) : 
        InteractiveServiceWithQoS(parentNode, serviceName, topicName, qos, msg::TopicQosProfile::QOS_TYPE_PUBLISHER, prop), 
        mPublisherF_(false), 
        mPublishingF_(false)
    {
        if (prop.targetAlive == msg::InteractiveService::TARGET_ALIVE_ENABLE)
        {
            mPub_ = mParentNode_->create_publisher<T>(mTopicName_, this->getQoS());
            mPublisherF_ = true;
        }
        if (prop.targetActivity == msg::InteractiveService::TARGET_ACTIVITY_ENABLE)
            mPublishingF_ = true;

        // Set target alive and target activity event handlers.
        this->setTargetAliveEventHandler(std::bind(&InteractivePublisher::_targetAliveEventHandler, this, std::placeholders::_1, std::placeholders::_2));
        this->setTargetActivityEventHandler(std::bind(&InteractivePublisher::_targetActivityEventHandler, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief Publish message.
     * @param[in] msg Message to be published.
     * @note The message will be published only if the target alive and target activity status are "enable".
     * @note This function is using mPubMtx_.
     */
    void publish(const T& msg)
    {
        std::lock_guard<std::mutex> lock(mPubMtx_);
        if (mPublisherF_ && mPublishingF_)
            mPub_->publish(msg);
    }
};



/**
 * @brief MultiInteractivePublisher inherits from MultiInteractiveServiceWithQoS.
 * @note MultiInteractivePublisher class can be controlled by any master device which has the privilege to control the publisher.
 * @note The class provides a controlable publisher with following functionalities:
 * @note 1. Create and destroy publisher by requesting target alive event with enable and disable status.
 * @note 2. Enable and disable publishing by requesting target activity event with enable and disable status.
 * @note The target event will be raised by checking all master devices' target alive and target activity status.
 */
template <typename T>
class MultiInteractivePublisher : public MultiInteractiveServiceWithQoS
{
private:
    std::shared_ptr<rclcpp::Publisher<T> > mPub_;// Publisher.
    std::mutex mPubMtx_;// Lock mPub_.
    std::atomic<bool> mPublisherF_;// Flag indicating whether the publisher is created.
    std::atomic<bool> mPublishingF_;// Flag indicating whether the publishing is enabled.

private:
    /**
     * @brief Target alive enable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to enable.
     * @note This function is using mPubMtx_.
     */
    void _targetAliveEnableCbFunc()
    {
        std::lock_guard<std::mutex> lock(mPubMtx_);
        if (mPub_.use_count() <= 0)
            mPub_ = mParentNode_->create_publisher<T>(mTopicName_, this->getQoS());
        mPublisherF_ = true;
    }

    /**
     * @brief Target alive disable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to disable.
     * @note This function is using mPubMtx_.
     */
    void _targetAliveDisableCbFunc()
    {
        std::lock_guard<std::mutex> lock(mPubMtx_);
        if (mPub_.use_count() > 0)
            mPub_.reset();
        mPublisherF_ = false;
    }

    /**
     * @brief Target activity enable callback function.
     * @details The function is called when the interactive service server receives a target activity request, and all target activity status are set to enable.
     */
    void _targetActivityEnableCbFunc()
    {
        mPublishingF_ = true;
    }

    /**
     * @brief Target activity disable callback function.
     * @details The function is called when the interactive service server receives a target activity request, and all target activity status are set to disable.
     */
    void _targetActivityDisableCbFunc()
    {
        mPublishingF_ = false;
    }

public:
    /**
     * @brief Constructor of MultiInteractivePublisher class.
     * @param[in] parentNode Parent node.
     * @param[in] serviceName Name of the interactive service.
     * @param[in] topicName Name of the topic.
     * @param[in] qos Quality of Service for the publisher.
     * @param[in] prop Interactive service initialization properties.
     * @note The interactive service will be created and named as serviceName.
     */
    MultiInteractivePublisher(rclcpp::Node *parentNode, 
                                const std::string& serviceName, 
                                const std::string& topicName, 
                                const rclcpp::QoS& qos, 
                                InteractiveServiceInitProp prop = InteractiveServiceInitProp()) : 
        MultiInteractiveServiceWithQoS(parentNode, serviceName, topicName, qos, msg::TopicQosProfile::QOS_TYPE_PUBLISHER, prop), 
        mPublisherF_(false), 
        mPublishingF_(false)
    {
        if (prop.targetAlive == msg::InteractiveService::TARGET_ALIVE_ENABLE)
        {
            mPub_ = mParentNode_->create_publisher<T>(mTopicName_, this->getQoS());
            mPublisherF_ = true;
        }
        if (prop.targetActivity == msg::InteractiveService::TARGET_ACTIVITY_ENABLE)
            mPublishingF_ = true;

        // Set target alive and target activity event handlers.
        this->setTargetAliveEnableEventHandler(std::bind(&MultiInteractivePublisher::_targetAliveEnableCbFunc, this));
        this->setTargetAliveDisableEventHandler(std::bind(&MultiInteractivePublisher::_targetAliveDisableCbFunc, this));
        this->setTargetActivityEnableEventHandler(std::bind(&MultiInteractivePublisher::_targetActivityEnableCbFunc, this));
        this->setTargetActivityDisableEventHandler(std::bind(&MultiInteractivePublisher::_targetActivityDisableCbFunc, this));
    }

    /**
     * @brief Publish message.
     * @param[in] msg Message to be published.
     * @note The message will be published only if the target alive and target activity status are "enable".
     * @note This function is using mPubMtx_.
     */
    void publish(const T& msg)
    {
        std::lock_guard<std::mutex> lock(mPubMtx_);
        if (mPublisherF_ && mPublishingF_)
            mPub_->publish(msg);
    }
};

}

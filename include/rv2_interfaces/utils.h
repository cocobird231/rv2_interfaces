#pragma once

#include <iostream>
#include <memory>
#include <cmath>

#include <mutex>
#include <atomic>
#include <functional>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <regex>
#include <sstream>
#include <fstream>
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

// Ref: https://stackoverflow.com/a/55475023
#ifndef __has_include
  static_assert(false, "__has_include not supported");
#else
#  if __cplusplus >= 201703L && __has_include(<filesystem>)
#    include <filesystem>
     namespace fs = std::filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
     namespace fs = boost::filesystem;
#  endif
#endif

namespace rv2_interfaces
{

struct thread_deleter
{
    void operator()(std::thread* t)
    {
        if (t->joinable())
            t->join();
        delete t;
    }
};

using unique_thread = std::unique_ptr<std::thread, thread_deleter>;

template<typename F, typename... args>
unique_thread make_unique_thread(F&& f, args&&... a)
{
    return unique_thread(new std::thread(std::forward<F>(f), std::forward<args>(a)...));
}

using shared_thread = std::shared_ptr<std::thread>;

template<typename F, typename... args>
shared_thread make_shared_thread(F&& f, args&&... a)
{
    return shared_thread(new std::thread(std::forward<F>(f), std::forward<args>(a)...), thread_deleter());
}



struct file_deleter
{
    void operator()(FILE* p) const
    {
        if (p)
            fclose(p);
    }
};

using unique_file = std::unique_ptr<FILE, file_deleter>;

inline std::unique_ptr<FILE, file_deleter> make_unique_file(const char* filename, const char* mode)
{
    return std::unique_ptr<FILE, file_deleter>(fopen(filename, mode));
}

using shared_file = std::shared_ptr<FILE>;

inline std::shared_ptr<FILE> make_shared_file(const char* filename, const char* mode)
{
    return std::shared_ptr<FILE>(fopen(filename, mode), file_deleter());
}



// Ref: https://stackoverflow.com/a/9407521
template<typename T>
struct has_const_iterator
{
private:
    typedef char                      yes;
    typedef struct { char array[2]; } no;

    template<typename C> static yes test(typename C::const_iterator*);
    template<typename C> static no  test(...);
public:
    static const bool value = sizeof(test<T>(0)) == sizeof(yes);
    typedef T type;
};

template <typename T>
struct has_begin_end
{
    template<typename C> static char (&f(typename std::enable_if<
        std::is_same<decltype(static_cast<typename C::const_iterator (C::*)() const>(&C::begin)),
        typename C::const_iterator(C::*)() const>::value, void>::type*))[1];

    template<typename C> static char (&f(...))[2];

    template<typename C> static char (&g(typename std::enable_if<
        std::is_same<decltype(static_cast<typename C::const_iterator (C::*)() const>(&C::end)),
        typename C::const_iterator(C::*)() const>::value, void>::type*))[1];

    template<typename C> static char (&g(...))[2];

    static bool const beg_value = sizeof(f<T>(0)) == 1;
    static bool const end_value = sizeof(g<T>(0)) == 1;
};

template<typename T> 
struct is_container : std::integral_constant<bool, has_const_iterator<T>::value && has_begin_end<T>::beg_value && has_begin_end<T>::end_value> 
{ };

template<typename T>
inline std::stringstream CvtSString(T t)
{
    std::stringstream ss;
    if constexpr (is_container<T>::value && !std::is_same<T, std::string>::value)
    {
        ss << "[";
        for (const auto & i : t)
            ss << i << ", ";
        ss << "]";
        return ss;
    }
    else
    {
        ss << t;
        return ss;
    }
}




class HierarchicalPrint
{
private:
    int maxHierarchy_;
    std::deque<std::pair<int, std::string> > que_;

private:
    std::string _getHierarchyStr()
    {
        std::vector<int> level(this->maxHierarchy_ + 1, 0);
        std::deque<std::string> printQue;

        // Search the hierarchy level from the end of the queue.
        for (std::deque<std::pair<int, std::string> >::reverse_iterator it{this->que_.rbegin()}; it != this->que_.rend(); ++it)
        {
            const auto& [hierarchy, str] = *it;
            bool changeF = level[hierarchy] == 0;// If the hierarchy level appears for the first time, change the symbol.
            level[hierarchy] = 1;// Mark the hierarchy level as appeared.
            std::string hierarchyStr = "";// Hierarcy prefix string.
            if (hierarchy == 0)// Root.
                hierarchyStr += "---";
            else
            {
                for (int j = 0; j < hierarchy; j++)// Find parent hierarchy level.
                {
                    if (j != 0 && level[j] == 1)// If the hierarchy level appears, add the symbol.
                        hierarchyStr += "|  ";
                    else
                        hierarchyStr += "   ";
                }
                if (changeF)// If the hierarchy level appears for the first time, change the symbol.
                    hierarchyStr += "\\__";
                else
                    hierarchyStr += "|--";
            }
            for (int j = hierarchy + 1; j < level.size(); j++)// Reset the hierarchy level after the current hierarchy level.
                level[j] = 0;
            printQue.emplace_front(hierarchyStr + str);// Add the hierarchy prefix string to the message.
        }
        std::string ret = "";
        for (const auto& i : printQue)// Combine the message with the hierarchy prefix string.
            ret += i + "\n";
        return ret;
    }

    friend std::ostream &operator<<(std::ostream &os, HierarchicalPrint &hp)
    {
        os << hp._getHierarchyStr();
        return os;
    }

    friend std::ostream &operator<<(std::ostream &os, HierarchicalPrint&& hp)
    {
        os << hp._getHierarchyStr();
        return os;
    }

public:
    HierarchicalPrint() : maxHierarchy_(0) {}

    HierarchicalPrint(HierarchicalPrint&& hp) : maxHierarchy_(hp.maxHierarchy_), que_(std::move(hp.que_)) {}

    void push(const int hierarchy, const std::string& str)
    {
        this->que_.push_back({ hierarchy, str });
        this->maxHierarchy_ = std::max(this->maxHierarchy_, hierarchy);
    }

    void push(const int hierarchy, const char* fmt, ...)
    {
        char printBuf[1024];
        va_list args;
        va_start(args, fmt);
        vsprintf(printBuf, fmt, args);
        va_end(args);
        this->que_.push_back({ hierarchy, printBuf });
        this->maxHierarchy_ = std::max(this->maxHierarchy_, hierarchy);
    }

    void append(const int startHierarchy, const HierarchicalPrint& hp)
    {
        int hpMinHierarchy = hp.maxHierarchy_;
        for (const auto& [hierarchy, str] : hp.que_)
            hpMinHierarchy = std::min(hpMinHierarchy, hierarchy);
        for (const auto& [hierarchy, str] : hp.que_)
            this->que_.push_back({ hierarchy - hpMinHierarchy + startHierarchy, str });
        this->maxHierarchy_ = std::max(this->maxHierarchy_, hp.maxHierarchy_ + startHierarchy - hpMinHierarchy);
    }

    void print()
    {
        std::cout << this->_getHierarchyStr();
    }

    void clear() { this->que_.clear(); }

    HierarchicalPrint& operator<<(HierarchicalPrint& hp)
    {
        for (const auto& [hierarchy, str] : hp.que_)
            this->que_.push_back({ hierarchy, str });
        this->maxHierarchy_ = std::max(this->maxHierarchy_, hp.maxHierarchy_);
        return *this;
    }
};

template<typename T>
struct ReasonResult
{
    T result;
    std::string reason;
    ReasonResult() {}
    ReasonResult(const T& res) : result(res), reason("") {}
    ReasonResult(const T& res, const std::string& reason) : result(res), reason(reason) {}
};

template<typename T>
struct DescriptiveValue
{
    T value = 0;
    std::string str = "";
    DescriptiveValue() {}
    DescriptiveValue(T value, std::string str) : value(value), str(str) {}
    bool operator==(const DescriptiveValue& tp)
    {
        return this->value == tp.value && this->str == tp.str;
    }
};

template<typename msgT, typename srvT>
class MsgDataSender : public rclcpp::Node
{
private:
    const std::string nodeName_;
    const std::string senderName_;

    std::shared_ptr<rclcpp::Publisher<msgT> > pub_;// Publisher for active mode.
    std::shared_ptr<rclcpp::Service<srvT> > srv_;// Service for passive mode.

    std::function<void(const std::shared_ptr<typename srvT::Request>, std::shared_ptr<typename srvT::Response>)> srvCbFunc_;// Passive mode callback function.
    bool isPassiveModeF_;
    bool srvCbFuncF_;

private:
    void _srvCbFunc(const std::shared_ptr<typename srvT::Request> req, std::shared_ptr<typename srvT::Response> res)
    {
        if (this->srvCbFuncF_ && this->isPassiveModeF_)
            this->srvCbFunc_(req, res);
        else
            RCLCPP_WARN(this->get_logger(), "[MsgDataSender::_srvCbFunc] No callback function for service: %s", this->senderName_.c_str());
    }

public:
    MsgDataSender(const std::string nodeName, const std::string senderName, rclcpp::QoS qos, bool passiveMode) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        senderName_(senderName), 
        isPassiveModeF_(passiveMode), 
        srvCbFuncF_(false)
    {
        if (passiveMode)
            this->srv_ = this->create_service<srvT>(senderName, std::bind(&MsgDataSender::_srvCbFunc, this, std::placeholders::_1, std::placeholders::_2));
        else
            this->pub_ = this->create_publisher<msgT>(senderName, qos);
    }

    void setSrvCbFunc(std::function<void(const std::shared_ptr<typename srvT::Request>, std::shared_ptr<typename srvT::Response>)> srvCbFunc)
    {
        this->srvCbFunc_ = srvCbFunc;
        this->srvCbFuncF_ = true;
    }

    void publish(const msgT& msg)
    {
        if (!this->isPassiveModeF_)
            this->pub_->publish(msg);
        else
            RCLCPP_WARN(this->get_logger(), "[MsgDataSender::publish] This is passive mode. Cannot publish message to service: %s", this->senderName_.c_str());
    }
};

template<typename msgT, typename srvT>
class MsgDataReceiver : public rclcpp::Node
{
private:
    const std::string nodeName_;
    const std::string receiverName_;

    std::shared_ptr<rclcpp::Subscription<msgT> > sub_;// Subscriber for active mode.
    std::shared_ptr<rclcpp::Client<srvT> > cli_;// Client for passive mode.
    rclcpp::Node::SharedPtr cliNode_;// Client node for passive mode.

    std::function<void(const std::shared_ptr<msgT>)> subCbFunc_;// Active mode callback function.
    bool isPassiveModeF_;
    bool subCbFuncF_;

private:
    void _subCbFunc(const std::shared_ptr<msgT> msg)
    {
        if (this->subCbFuncF_ && !this->isPassiveModeF_)
            this->subCbFunc_(msg);
        else
            RCLCPP_WARN(this->get_logger(), "[MsgDataReceiver::_subCbFunc] No callback function for subscription: %s", this->receiverName_.c_str());
    }

public:
    MsgDataReceiver(const std::string nodeName, const std::string receiverName, rclcpp::QoS qos, bool passiveMode) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        receiverName_(receiverName), 
        isPassiveModeF_(passiveMode), 
        subCbFuncF_(false)
    {
        if (passiveMode)
        {
            this->cli_ = this->create_client<srvT>(receiverName);
            this->cliNode_ = std::make_shared<rclcpp::Node>(nodeName + "_cli_node");
        }
        else
            this->sub_ = this->create_subscription<msgT>(receiverName, qos, std::bind(&MsgDataReceiver::_subCbFunc, this, std::placeholders::_1));
    }

    void setSubCbFunc(std::function<void(const std::shared_ptr<msgT>)> subCbFunc)
    {
        this->subCbFunc_ = subCbFunc;
        this->subCbFuncF_ = true;
    }

    bool request(const std::shared_ptr<typename srvT::Request> req, double timeout_ms = 500)
    {
        if (this->isPassiveModeF_)
        {
            auto result = this->cli_->async_send_request(req);

            if (rclcpp::spin_until_future_complete(this->cliNode_, result, std::chrono::duration<double, std::milli>(timeout_ms)) == rclcpp::FutureReturnCode::SUCCESS)
            {
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[MsgDataReceiver::request] Request service timeout: %s", this->receiverName_.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[MsgDataReceiver::request] This is active mode. Cannot request service: %s", this->receiverName_.c_str());
            return false;
        }
    }
};



template<typename keyT, typename valT>
class MapRegister
{
private:
    std::map<keyT, std::pair<valT, bool> > elemStatusMap_;// { key, { value, regStatus } }
    std::mutex elemStatusMapMtx_;
    std::atomic<bool> procF_;// Process flag for registration. True if element need to be registered.
    std::atomic<bool> idleF_;// Whether the registration is idle or not.
    std::function<bool(const keyT, const valT)> regFunc_;// Custom registration function.

    std::thread regTh_;// Check registration. Condition variable is used to notify the thread.
    std::mutex regThMtx_;
    std::condition_variable cv_;
    std::atomic<bool> exitF_;

private:
    void _reg()
    {
        while (!this->exitF_.load())
        {
            std::unique_lock<std::mutex> regThLock(this->regThMtx_, std::defer_lock);
            regThLock.lock();
            this->cv_.wait(regThLock);
            regThLock.unlock();

            this->idleF_.store(false);

            if (this->exitF_.load())
                break;

            while (this->procF_.load())
            {
                this->procF_.store(false);
                std::map<keyT, std::pair<valT, bool> > tmpElemStatusMap;
                {
                    std::lock_guard<std::mutex> elemStatusMapLock(this->elemStatusMapMtx_);
                    tmpElemStatusMap = this->elemStatusMap_;
                }

                std::map<keyT, std::future<bool> > futureMap;
                for (auto& [key, valPair] : tmpElemStatusMap)
                {
                    const auto& [val, status] = valPair;
                    if (status)
                        continue;
                    futureMap[key] = std::async(std::launch::async, this->regFunc_, key, val);
                }

                bool isAllDone = true;
                for (auto& [key, future] : futureMap)
                {
                    if (future.get())
                    {
                        std::lock_guard<std::mutex> elemStatusMapLock(this->elemStatusMapMtx_);
                        if (this->elemStatusMap_.find(key) == this->elemStatusMap_.end())// Ignore if the key is removed
                            continue;
                        this->elemStatusMap_[key].second = true;
                    }
                    else
                    {
                        isAllDone = false;
                    }
                }
                if (!isAllDone)
                    this->procF_.store(true);// Retry if the registration is failed.
            }
            this->idleF_.store(true);
        }
    }

public:
    MapRegister(std::function<bool(const keyT, const valT)> func) : regFunc_(func), procF_(false), idleF_(true), exitF_(false)
    {
        this->regTh_ = std::thread(&MapRegister::_reg, this);
    }

    ~MapRegister()
    {
        this->exitF_.store(true);
        this->cv_.notify_one();
        if (this->regTh_.joinable())
            this->regTh_.join();
    }

    void add(const keyT& key, const valT& value)
    {
        std::lock_guard<std::mutex> elemStatusMapLock(this->elemStatusMapMtx_);
        if (this->elemStatusMap_.find(key) != this->elemStatusMap_.end())// Ignore if the key is already added
            return;
        this->elemStatusMap_[key] = { value, false };
        this->procF_.store(true);

        std::lock_guard<std::mutex> regThLock(this->regThMtx_);
        this->cv_.notify_one();
    }

    void remove(const keyT& key)
    {
        std::lock_guard<std::mutex> elemStatusMapLock(this->elemStatusMapMtx_);
        if (this->elemStatusMap_.find(key) == this->elemStatusMap_.end())// Ignore if the key is not found
            return;
        this->elemStatusMap_.erase(key);
    }

    void reRegister()
    {
        std::lock_guard<std::mutex> elemStatusMapLock(this->elemStatusMapMtx_);
        for (auto& [key, valPair] : this->elemStatusMap_)
            valPair.second = false;
        this->procF_.store(true);

        std::lock_guard<std::mutex> regThLock(this->regThMtx_);
        this->cv_.notify_one();
    }

    bool isExist(const keyT& key)
    {
        std::lock_guard<std::mutex> elemStatusMapLock(this->elemStatusMapMtx_);
        return this->elemStatusMap_.find(key) != this->elemStatusMap_.end();
    }

    bool isAllDone() const
    {
        return this->idleF_.load();
    }

    std::map<keyT, valT> getMap()
    {
        std::map<keyT, valT> ret;
        std::lock_guard<std::mutex> elemStatusMapLock(this->elemStatusMapMtx_);
        for (const auto& [key, valPair] : this->elemStatusMap_)
            ret[key] = valPair.first;
        return ret;
    }
};

/**
 * MapRegister Function
 * template<typename keyT, typename valT>
 * bool RegFuncTemplate(const keyT keyName, const valT value)
 * {
 *     // Return true if registered successfully.
 *     // If the registration is failed, return false, the MapRegister will try to register the pair again.
 *     return true;
 * }
 */

template<typename T>
void GetParam(std::shared_ptr<rclcpp::Node> node, std::string paramName, T defValue, T & outVal, std::string log_info, bool dynamic)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = !dynamic;

    node->declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

    if (!node->get_parameter(paramName, outVal))
    {
        RCLCPP_WARN_STREAM(node->get_logger(),
            "The parameter '"
            << paramName
            << "' is not available or is not valid, using the default value: "
            << rv2_interfaces::CvtSString(defValue).str());
    }

    if (!log_info.empty())
    {
        RCLCPP_INFO_STREAM(node->get_logger(), log_info << rv2_interfaces::CvtSString(outVal).str());
    }
}

template<typename T>
void GetParamRawPtr(rclcpp::Node *node, std::string paramName, T defValue, T & outVal, std::string log_info, bool dynamic)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = !dynamic;

    node->declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

    if (!node->get_parameter(paramName, outVal))
    {
        RCLCPP_WARN_STREAM(node->get_logger(),
            "The parameter '"
            << paramName
            << "' is not available or is not valid, using the default value: "
            << rv2_interfaces::CvtSString(defValue).str());
    }

    if (!log_info.empty())
    {
        RCLCPP_INFO_STREAM(node->get_logger(), log_info << rv2_interfaces::CvtSString(outVal).str());
    }
}



template<typename T>
void SetParam(std::shared_ptr<rclcpp::Node> node, std::string paramName, T defValue, bool dynamic)
{
    try
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = !dynamic;
        node->declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) { }
    catch (...)
    {
        std::cerr << "Failed to set parameter: " << paramName << std::endl;
        return;
    }

    node->set_parameter(rclcpp::Parameter(paramName, defValue));
    RCLCPP_INFO_STREAM(node->get_logger(), rv2_interfaces::CvtSString(defValue).str());
}



class ExecNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
    rv2_interfaces::unique_thread th_;

public:
    ExecNode(const std::string& nodeName, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    {
        this->node_ = rclcpp::Node::make_shared(nodeName, options);
        this->exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }

    ExecNode(rclcpp::Node::SharedPtr node)
    {
        this->node_ = node;
        this->exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    }

    // Disable copy constructor
    ExecNode(const ExecNode&) = delete;

    ~ExecNode()
    {
        this->stop();
    }

    // Disable copy assignment
    ExecNode& operator=(const ExecNode&) = delete;

    void start()
    {
        if (!this->node_)
        {
            std::cerr << "[ExecNode] Node is not initialized" << std::endl;
            return;
        }

        if (this->exec_->is_spinning())
        {
            std::cerr << "[ExecNode] Executor is already running" << std::endl;
            return;
        }

        this->exec_->add_node(node_);
        this->th_ = rv2_interfaces::make_unique_thread([this]() {
            exec_->spin();
        });
    }

    void stop()
    {
        if (this->exec_->is_spinning())
        {
            this->exec_->cancel();
            this->th_.reset();
            this->exec_->remove_node(this->node_);
        }
    }

    std::shared_ptr<rclcpp::Node> getNode() const
    {
        return this->node_;
    }

    using SharedPtr = std::shared_ptr<ExecNode>;
};





class MultiExecNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr exec_;
    rv2_interfaces::unique_thread th_;

public:
    MultiExecNode(const std::string& nodeName, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    {
        this->node_ = rclcpp::Node::make_shared(nodeName, options);
        this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    }

    MultiExecNode(rclcpp::Node::SharedPtr node)
    {
        this->node_ = node;
        this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    }

    // Disable copy constructor
    MultiExecNode(const MultiExecNode&) = delete;

    ~MultiExecNode()
    {
        this->stop();
    }

    // Disable copy assignment
    MultiExecNode& operator=(const MultiExecNode&) = delete;

    void start()
    {
        if (!this->node_)
        {
            std::cerr << "[MultiExecNode] Node is not initialized" << std::endl;
            return;
        }

        if (this->exec_->is_spinning())
        {
            std::cerr << "[MultiExecNode] Executor is already running" << std::endl;
            return;
        }

        this->exec_->add_node(node_);
        this->th_ = rv2_interfaces::make_unique_thread([this]() {
            exec_->spin();
        });
    }

    void stop()
    {
        if (this->exec_->is_spinning())
        {
            this->exec_->cancel();
            this->th_.reset();
            this->exec_->remove_node(this->node_);
        }
    }

    std::shared_ptr<rclcpp::Node> getNode() const
    {
        return this->node_;
    }

    using SharedPtr = std::shared_ptr<MultiExecNode>;
};




template <typename RCL_NODE_TYPE>
std::shared_ptr<RCL_NODE_TYPE> GenTmpNode(std::string prefix = "tmp_", std::string suffix = "_node", bool use_global_arguments = false)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(use_global_arguments);
    auto ts = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    return std::make_shared<RCL_NODE_TYPE>(prefix + std::to_string(ts) + suffix, options);
}

template<typename srvT>
using srvTReqPtr = std::shared_ptr<typename srvT::Request>;

template<typename srvT>
using srvTResPtr = std::shared_ptr<typename srvT::Response>;



/**
 * ClientRequestHelper is a helper function to request service.
 * @param[in] node: The node to request the service.
 * @param[in] serviceName: The name of the service. Should not include the namespace or '/'.
 * @param[in] req: The request message.
 * @param[in] timeout: The timeout duration.
 * @return The response message. If the request is timeout or failed, return `nullptr`.
 */
template<typename srvT>
srvTResPtr<srvT> ClientRequestHelper(std::shared_ptr<rclcpp::Node> node, const std::string& serviceName, const srvTReqPtr<srvT> req, std::chrono::milliseconds timeout)
{
    auto cbg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto client = node->create_client<srvT>(serviceName, rmw_qos_profile_services_default, cbg);
    auto result = client->async_send_request(req);
    auto fStatus = result.wait_for(timeout);
    if (fStatus == std::future_status::ready)
        return result.get();
    return nullptr;
}



/**
 * ClientRequestHelperRawPtr is a helper function to request service.
 * @param[in] node: The node to request the service.
 * @param[in] serviceName: The name of the service. Should not include the namespace or '/'.
 * @param[in] req: The request message.
 * @param[in] timeout: The timeout duration.
 * @return The response message. If the request is timeout or failed, return `nullptr`.
 */
template<typename srvT>
srvTResPtr<srvT> ClientRequestHelperRawPtr(rclcpp::Node *node, const std::string& serviceName, const srvTReqPtr<srvT> req, std::chrono::milliseconds timeout)
{
    auto client = node->create_client<srvT>(serviceName);
    auto result = client->async_send_request(req);
    auto fStatus = result.wait_for(timeout);
    if (fStatus == std::future_status::ready)
        return result.get();
    return nullptr;
}



/**
 * ClientRequestHelperCbG is a helper function to request service.
 * @param[in] node: The node to request the service.
 * @param[in] cbg: The callback group to request the service.
 * @param[in] serviceName: The name of the service. Should not include the namespace or '/'.
 * @param[in] req: The request message.
 * @param[in] timeout: The timeout duration.
 * @return The response message. If the request is timeout or failed, return `nullptr`.
 */
template<typename srvT>
srvTResPtr<srvT> ClientRequestHelperCbG(std::shared_ptr<rclcpp::Node> node, rclcpp::CallbackGroup::SharedPtr cbg, const std::string& serviceName, const srvTReqPtr<srvT> req, std::chrono::milliseconds timeout)
{
    auto client = node->create_client<srvT>(serviceName, rmw_qos_profile_services_default, cbg);
    auto result = client->async_send_request(req);
    auto fStatus = result.wait_for(timeout);
    if (fStatus == std::future_status::ready)
        return result.get();
    return nullptr;
}



/**
 * ClientRequestHelperRawPtrCbG is a helper function to request service.
 * @param[in] node: The node to request the service.
 * @param[in] cbg: The callback group to request the service.
 * @param[in] serviceName: The name of the service. Should not include the namespace or '/'.
 * @param[in] req: The request message.
 * @param[in] timeout: The timeout duration.
 * @return The response message. If the request is timeout or failed, return `nullptr`.
 */
template<typename srvT>
srvTResPtr<srvT> ClientRequestHelperRawPtrCbG(rclcpp::Node* node, rclcpp::CallbackGroup::SharedPtr cbg, const std::string& serviceName, const srvTReqPtr<srvT> req, std::chrono::milliseconds timeout)
{
    auto client = node->create_client<srvT>(serviceName, rmw_qos_profile_services_default, cbg);
    auto result = client->async_send_request(req);
    auto fStatus = result.wait_for(timeout);
    if (fStatus == std::future_status::ready)
        return result.get();
    return nullptr;
}



inline bool ConnToService(rclcpp::ClientBase::SharedPtr client, bool& stopF, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000), int retry = 5)
{
    printf("[ConnToService] Connect to service: %s (%d)\n", client->get_service_name(), retry);
    if (retry > 0)
    {
        while (!client->wait_for_service(timeout) && retry-- > 0 && !stopF)
        {
            if (!rclcpp::ok())
            {
                printf("[ConnToService (%s)] Interrupted while waiting for the service. Exiting.\n", client->get_service_name());
                return false;
            }
            printf("[ConnToService (%s)] Service not available, waiting again... (%d)\n", client->get_service_name(), retry);
        }
        if (retry < 0 || stopF)
        {
            printf("[ConnToService (%s)] Connect to service failed.\n", client->get_service_name());
            return false;
        }
        printf("[ConnToService (%s)] Service connected.\n", client->get_service_name());
        return true;
    }
    else
    {
        while (!client->wait_for_service(timeout) && !stopF)
        {
            if (!rclcpp::ok())
            {
                printf("[ConnToService (%s)] Interrupted while waiting for the service. Exiting.\n", client->get_service_name());
                return false;
            }
            printf("[ConnToService (%s)] Service not available, waiting again...\n", client->get_service_name());
        }
        if (stopF)
        {
            printf("[ConnToService (%s)] Connect to service failed.\n", client->get_service_name());
            return false;
        }
        printf("[ConnToService (%s)] Service connected.\n", client->get_service_name());
        return true;
    }
}

template<typename T>
T SafeLoad(T* ptr, std::mutex& mtx)
{
    std::lock_guard<std::mutex> lock(mtx);
    return *ptr;
}

template<typename T>
void SafeStore(T* ptr, const T& val, std::mutex& mtx)
{
    std::lock_guard<std::mutex> lock(mtx);
    *ptr = val;
}



/**
 * ================================================================
 * String Operation
 * ================================================================
 */



inline std::vector<std::string> split(const std::string& str, const std::string& delimiter)
{
    std::vector<std::string> splitStrings;
    int encodingStep = 0;
    for (size_t i = 0; i < str.length(); i++)
    {
        bool isDelimiter = false;
        for (auto& j : delimiter)
            if (str[i] == j)
            {
                isDelimiter = true;
                break;
            }
        if (!isDelimiter)// Is the spliting character
        {
            encodingStep++;
            if (i == str.length() - 1)
                splitStrings.push_back(str.substr(str.length() - encodingStep, encodingStep));
        }
        else// Is delimiter
        {
            if (encodingStep > 0)// Have characters need to split
                splitStrings.push_back(str.substr(i - encodingStep, encodingStep));
            encodingStep = 0;
        }
    }
    return splitStrings;
}

/**
 * This code is referenced from: https://en.cppreference.com/w/cpp/string/basic_string/replace
*/
inline std::size_t replace_all(std::string& inout, std::string what, std::string with)
{
    std::size_t count{};
    for (std::string::size_type pos{};
            inout.npos != (pos = inout.find(what.data(), pos, what.length()));
            pos += with.length(), ++count)
    {
        inout.replace(pos, what.length(), with.data(), with.length());
    }
    return count;
}



/**
 * ================================================================
 * File Operation
 * ================================================================
 */



inline fs::path GetHomePath()
{
    const static uint16_t BUFF_SIZE = 256;
    char buf[BUFF_SIZE];
    FILE* fp = popen("echo $HOME", "r");
    std::string retStr = "";
    if (fp != NULL)
    {
        while (fgets(buf, BUFF_SIZE, fp) != NULL)
            retStr += buf;
        pclose(fp);
    }
    if (retStr.back() == '\n')
        retStr.pop_back();
    printf("Found home path: %s\n", retStr.c_str());
    return retStr;
}

inline fs::path GetCurrentPath()
{
    const static uint16_t BUFF_SIZE = 256;
    char buf[BUFF_SIZE];
    FILE* fp = popen("echo $PWD", "r");
    std::string retStr = "";
    if (fp != NULL)
    {
        while (fgets(buf, BUFF_SIZE, fp) != NULL)
            retStr += buf;
        pclose(fp);
    }
    if (retStr.back() == '\n')
        retStr.pop_back();
    printf("Found current path: %s\n", retStr.c_str());
    return retStr;
}



inline bool GetHostname(std::string& outHostname)
{
    char buf[128];
    bool retF = false;

    // Get hostname
    FILE* fp = popen("hostname | awk '{print \"^\"$0\"!\"}'", "r");
    if (fp != NULL)
    {
        while (fgets(buf, 128, fp) != NULL)
        {
            std::string recvStr(buf);
            recvStr = recvStr.substr(recvStr.find('^') + 1, recvStr.rfind('!') - 1);
            if (recvStr.length() > 0)
            {
                outHostname = recvStr;
                retF = true;
            }
        }
        pclose(fp);
    }
    return retF;
}



inline bool GetIPv4Addr(const std::string& ifName, std::string& outIPv4)
{
    char buf[128];
    char cmdBuf[128];
    bool retF = false;

    // Get IP address
    sprintf(cmdBuf, "ip addr show dev %s | grep -Po \"(?<=inet )((\\d{1,3}\\.){3}\\d{1,3})\" | awk '{print \"^\"$0\"!\"}'", ifName.c_str());
    FILE* fp = popen(cmdBuf, "r");
    if (fp != NULL)
    {
        while (fgets(buf, 128, fp) != NULL)
        {
            std::string recvStr(buf);
            recvStr = recvStr.substr(recvStr.find('^') + 1, recvStr.rfind('!') - 1);
            if (recvStr.length() > 0)
            {
                std::smatch match;
                std::regex_match(recvStr, match, std::regex("(\\d{1,3}\\.){3}\\d{1,3}"));
                if (match.size() > 0)
                {
                    outIPv4 = recvStr;
                    retF = true;
                }
            }
        }
        pclose(fp);
    }
    return retF;
}



inline bool GetMACAddr(const std::string& ifName, std::string& outMAC)
{
    char buf[128];
    char cmdBuf[128];
    bool retF = false;

    // Get MAC address
    sprintf(cmdBuf, "ip addr show dev %s | grep -Po \"(?<=link/ether )(([A-Za-z0-9]{2}:){5}[A-Za-z0-9]{2})(?= brd)\" | awk '{print \"^\"$0\"!\"}'", ifName.c_str());
    FILE* fp = popen(cmdBuf, "r");
    if (fp != NULL)
    {
        while (fgets(buf, 128, fp) != NULL)
        {
            std::string recvStr(buf);
            recvStr = recvStr.substr(recvStr.find('^') + 1, recvStr.rfind('!') - 1);
            if (recvStr.length() > 0)
            {
                std::smatch match;
                std::regex_match(recvStr, match, std::regex("([A-Za-z0-9]{2}:){5}[A-Za-z0-9]{2}"));
                if (match.size() > 0)
                {
                    outMAC = recvStr;
                    retF = true;
                }
            }
        }
        pclose(fp);
    }
    return retF;
}



inline bool LoadFileFromJSON(const fs::path& filePath, nlohmann::json& outJSON)
{
    try
    {
        outJSON.update(nlohmann::json::parse(std::ifstream(filePath)));
        return true;
    }
    catch(...)
    {
        return false;
    }
}



inline bool DumpJSONToFile(const fs::path& filePath, const nlohmann::json& json)
{
    try
    {
        std::ofstream outFile(filePath);
        outFile << json;
        return true;
    }
    catch (...)
    {
        return false;
    }
}



/**
 * ================================================================
 * Value Mapping Operation
 * ================================================================
 */



inline double LinearMapping1d(double value, double from_start, double from_end, double to_start, double to_end)
{
    double a = (to_end - to_start) / (from_end - from_start);
    return a * (value - from_start) + to_start;
}

inline double GammaCorrection(double value, double gamma, double min_bound = 0, double max_bound = 1)
{
    if (value >= min_bound && value <= max_bound && max_bound > min_bound)
    {
        if (min_bound == 0 && max_bound == 1)
            return std::pow(value, gamma);
        double ratio = (value - min_bound) / (max_bound - min_bound);
        return std::pow(ratio, gamma) * (max_bound - min_bound) + min_bound;
    }
    throw "Boundary value error.";
}

}
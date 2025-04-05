#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "parameter_info_msgs/msg/parameter_info.hpp"

#define CLIENT_GET_VAL(typeName)                                                                                                                            \
    {                                                                                                                                                       \
        std::string type;                                                                                                                                   \
        std::string input_type = #typeName;                                                                                                                 \
        if (GetType(name, type))                                                                                                                            \
        {                                                                                                                                                   \
            if (type == input_type)                                                                                                                         \
            {                                                                                                                                               \
                for (const auto &valInfo : parameterInfo_->typeName##_infos)                                                                                \
                {                                                                                                                                           \
                    if (valInfo.name == name)                                                                                                               \
                    {                                                                                                                                       \
                        val = valInfo.val;                                                                                                                  \
                        return true;                                                                                                                        \
                    }                                                                                                                                       \
                }                                                                                                                                           \
            }                                                                                                                                               \
            else                                                                                                                                            \
            {                                                                                                                                               \
                printf("[ERROR] ParameterService type error, input type: %s, but parameter %s type: %s\n", input_type.c_str(), name.c_str(), type.c_str()); \
            }                                                                                                                                               \
        }                                                                                                                                                   \
        else                                                                                                                                                \
        {                                                                                                                                                   \
            printf("[ERROR] ParameterService, parameter %s not exist in type_info_list!\n", name.c_str());                                                  \
        }                                                                                                                                                   \
        return false;                                                                                                                                       \
    }

#define CLIENT_GET_VAL_LIST(typeName)                                                                                                                       \
    {                                                                                                                                                       \
        std::string type;                                                                                                                                   \
        std::string input_type = #typeName;                                                                                                                 \
        if (GetType(name, type))                                                                                                                            \
        {                                                                                                                                                   \
            if (type == input_type)                                                                                                                         \
            {                                                                                                                                               \
                for (const auto &valInfo : parameterInfo_->typeName##_infos)                                                                                \
                {                                                                                                                                           \
                    if (valInfo.name == name)                                                                                                               \
                    {                                                                                                                                       \
                        val.assign(valInfo.val.begin(), valInfo.val.end());                                                                                 \
                        return true;                                                                                                                        \
                    }                                                                                                                                       \
                }                                                                                                                                           \
            }                                                                                                                                               \
            else                                                                                                                                            \
            {                                                                                                                                               \
                printf("[ERROR] ParameterService type error, input type: %s, but parameter %s type: %s\n", input_type.c_str(), name.c_str(), type.c_str()); \
            }                                                                                                                                               \
        }                                                                                                                                                   \
        else                                                                                                                                                \
        {                                                                                                                                                   \
            printf("[ERROR] ParameterService, parameter %s not exist in type_info_list!\n", name.c_str());                                                  \
        }                                                                                                                                                   \
        return false;                                                                                                                                       \
    }

namespace ParameterService
{
    class Client
    {
    public:
        Client(std::string serviceName)
        {
            node_  = rclcpp::Node::make_shared("parameter_client");
            subscription_ = node_->create_subscription<parameter_info_msgs::msg::ParameterInfo>(
                serviceName, rclcpp::QoS(1).transient_local(), std::bind(&Client::topic_callback_, this, std::placeholders::_1));
        }
        Client(rclcpp::Node::SharedPtr node, std::string serviceName) : node_(node)
        {
            subscription_ = node_->create_subscription<parameter_info_msgs::msg::ParameterInfo>(
                serviceName, rclcpp::QoS(1).transient_local(), std::bind(&Client::topic_callback_, this, std::placeholders::_1));
        }
        ~Client() = default;

        void GetParameter()
        {
            while (!hasGet_)
            {
                rclcpp::spin_some(node_);
                usleep(10000);
            }
            subscription_.reset();
            node_.reset();
        }

        bool ParseParameter(const std::string &name, int8_t &val)
        {
            CLIENT_GET_VAL(int8);
        }
        bool ParseParameter(const std::string &name, int16_t &val)
        {
            CLIENT_GET_VAL(int16);
        }
        bool ParseParameter(const std::string &name, int32_t &val)
        {
            CLIENT_GET_VAL(int32);
        }
        bool ParseParameter(const std::string &name, int64_t &val)
        {
            CLIENT_GET_VAL(int64);
        }
        bool ParseParameter(const std::string &name, uint8_t &val)
        {
            CLIENT_GET_VAL(uint8);
        }
        bool ParseParameter(const std::string &name, uint16_t &val)
        {
            CLIENT_GET_VAL(uint16);
        }
        bool ParseParameter(const std::string &name, uint32_t &val)
        {
            CLIENT_GET_VAL(uint32);
        }
        bool ParseParameter(const std::string &name, uint64_t &val)
        {
            CLIENT_GET_VAL(uint64);
        }
        bool ParseParameter(const std::string &name, float &val)
        {
            CLIENT_GET_VAL(float32);
        }
        bool ParseParameter(const std::string &name, double &val)
        {
            CLIENT_GET_VAL(float64);
        }
        bool ParseParameter(const std::string &name, std::string &val)
        {
            CLIENT_GET_VAL(string);
        }

        // list
        bool ParseParameter(const std::string &name, std::vector<int8_t> &val)
        {
            CLIENT_GET_VAL_LIST(int8_list);
        }
        bool ParseParameter(const std::string &name, std::vector<int16_t> &val)
        {
            CLIENT_GET_VAL_LIST(int16_list);
        }
        bool ParseParameter(const std::string &name, std::vector<int32_t> &val)
        {
            CLIENT_GET_VAL_LIST(int32_list);
        }
        bool ParseParameter(const std::string &name, std::vector<int64_t> &val)
        {
            CLIENT_GET_VAL_LIST(int64_list);
        }
        bool ParseParameter(const std::string &name, std::vector<uint8_t> &val)
        {
            CLIENT_GET_VAL_LIST(uint8_list);
        }
        bool ParseParameter(const std::string &name, std::vector<uint16_t> &val)
        {
            CLIENT_GET_VAL_LIST(uint16_list);
        }
        bool ParseParameter(const std::string &name, std::vector<uint32_t> &val)
        {
            CLIENT_GET_VAL_LIST(uint32_list);
        }
        bool ParseParameter(const std::string &name, std::vector<uint64_t> &val)
        {
            CLIENT_GET_VAL_LIST(uint64_list);
        }
        bool ParseParameter(const std::string &name, std::vector<float> &val)
        {
            CLIENT_GET_VAL_LIST(float32_list);
        }
        bool ParseParameter(const std::string &name, std::vector<double> &val)
        {
            CLIENT_GET_VAL_LIST(float64_list);
        }

    private:
        void topic_callback_(parameter_info_msgs::msg::ParameterInfo::ConstSharedPtr msg)
        {
            parameterInfo_ = msg;
            hasGet_ = true;
        }

        bool GetType(const std::string &name, std::string &type)
        {
            for (const auto &typeInfo : parameterInfo_->type_info_list)
            {
                if (typeInfo.name == name)
                {
                    type = typeInfo.type;
                    return true;
                }
            }
            return false;
        }

        bool hasGet_ = false;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<parameter_info_msgs::msg::ParameterInfo>::SharedPtr subscription_;
        parameter_info_msgs::msg::ParameterInfo::ConstSharedPtr parameterInfo_;
    };

}
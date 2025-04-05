#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "parameter_info_msgs/msg/parameter_info.hpp"

#define SEVER_SET_VAL(typeName)                       \
    {                                                 \
        auto &info = parameterInfo_.typeName##_infos; \
        int index = CheckParameterExist(name, info);  \
        if (index < 0)                                \
        {                                             \
            infoValue.name = name;                    \
            infoValue.val = val;                      \
            info.push_back(infoValue);                \
            SetTypeInfo(name, #typeName);             \
        }                                             \
        else                                          \
        {                                             \
            info[index].val = val;                    \
        }                                             \
        return true;                                  \
    }

#define SEVER_SET_VAL_LIST(typeName)                      \
    {                                                     \
        auto &info = parameterInfo_.typeName##_infos;     \
        int index = CheckParameterExist(name, info);      \
        if (index < 0)                                    \
        {                                                 \
            infoValue.name = name;                        \
            infoValue.val.assign(val.begin(), val.end()); \
            info.push_back(infoValue);                    \
            SetTypeInfo(name, #typeName);                 \
        }                                                 \
        else                                              \
        {                                                 \
            info[index].val = val;                        \
        }                                                 \
        return true;                                      \
    }

namespace ParameterService
{
    class Server
    {
    public:
        Server(const std::string &serviceName)
        {
            node_  = rclcpp::Node::make_shared("parameter_server");
            publisher_ = node_->create_publisher<parameter_info_msgs::msg::ParameterInfo>(
                serviceName, rclcpp::QoS(1).transient_local());
            parameterInfo_ = parameter_info_msgs::msg::ParameterInfo();
        }
        Server(rclcpp::Node::SharedPtr node, const std::string &serviceName) : node_(node)
        {
            publisher_ = node_->create_publisher<parameter_info_msgs::msg::ParameterInfo>(
                serviceName, rclcpp::QoS(1).transient_local());
            parameterInfo_ = parameter_info_msgs::msg::ParameterInfo();
        }

        ~Server()
        {
        }

        void Call()
        {
            publisher_->publish(parameterInfo_);
        }

        bool SetParameter(const std::string &name, int8_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Int8Info();
            SEVER_SET_VAL(int8);
        }
        bool SetParameter(const std::string &name, int16_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Int16Info();
            SEVER_SET_VAL(int16);
        }
        bool SetParameter(const std::string &name, int32_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Int32Info();
            SEVER_SET_VAL(int32);
        }
        bool SetParameter(const std::string &name, int64_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Int64Info();
            SEVER_SET_VAL(int64);
        }
        bool SetParameter(const std::string &name, uint8_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint8Info();
            SEVER_SET_VAL(uint8);
        }
        bool SetParameter(const std::string &name, uint16_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint16Info();
            SEVER_SET_VAL(uint16);
        }
        bool SetParameter(const std::string &name, uint32_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint32Info();
            SEVER_SET_VAL(uint32);
        }
        bool SetParameter(const std::string &name, uint64_t val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint64Info();
            SEVER_SET_VAL(uint64);
        }
        bool SetParameter(const std::string &name, float val)
        {
            auto infoValue = parameter_info_msgs::msg::Float32Info();
            SEVER_SET_VAL(float32);
        }
        bool SetParameter(const std::string &name, double val)
        {
            auto infoValue = parameter_info_msgs::msg::Float64Info();
            SEVER_SET_VAL(float64);
        }
        bool SetParameter(const std::string &name, std::string val)
        {
            auto infoValue = parameter_info_msgs::msg::StringInfo();
            SEVER_SET_VAL(string);
        }

        // list
        bool SetParameter(const std::string &name, std::vector<int8_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Int8ListInfo();
            SEVER_SET_VAL_LIST(int8_list);
        }

        bool SetParameter(const std::string &name, std::vector<int16_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Int16ListInfo();
            SEVER_SET_VAL_LIST(int16_list);
        }

        bool SetParameter(const std::string &name, std::vector<int32_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Int32ListInfo();
            SEVER_SET_VAL_LIST(int32_list);
        }

        bool SetParameter(const std::string &name, std::vector<int64_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Int64ListInfo();
            SEVER_SET_VAL_LIST(int64_list);
        }

        bool SetParameter(const std::string &name, std::vector<uint8_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint8ListInfo();
            SEVER_SET_VAL_LIST(uint8_list);
        }

        bool SetParameter(const std::string &name, std::vector<uint16_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint16ListInfo();
            SEVER_SET_VAL_LIST(uint16_list);
        }

        bool SetParameter(const std::string &name, std::vector<uint32_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint32ListInfo();
            SEVER_SET_VAL_LIST(uint32_list);
        }

        bool SetParameter(const std::string &name, std::vector<uint64_t> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Uint64ListInfo();
            SEVER_SET_VAL_LIST(uint64_list);
        }

        bool SetParameter(const std::string &name, std::vector<float> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Float32ListInfo();
            SEVER_SET_VAL_LIST(float32_list);
        }

        bool SetParameter(const std::string &name, std::vector<double> &val)
        {
            auto infoValue = parameter_info_msgs::msg::Float64ListInfo();
            SEVER_SET_VAL_LIST(float64_list);
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<parameter_info_msgs::msg::ParameterInfo>::SharedPtr publisher_;
        parameter_info_msgs::msg::ParameterInfo parameterInfo_;

        template <typename ValueType>
        int CheckParameterExist(const std::string &name, ValueType infos)
        {
            for (size_t i = 0; i < infos.size(); i++)
            {
                if (infos[i].name == name)
                {
                    return i;
                }
            }
            return -1;
        }

        void SetTypeInfo(const std::string &name, const std::string &type)
        {
            auto infoType = parameter_info_msgs::msg::TypeInfo();
            infoType.name = name;
            infoType.type = type;
            parameterInfo_.type_info_list.push_back(infoType);
        }
    };

}
#ifndef FUSION_READ_PARAM_BASE_H_
#define FUSION_READ_PARAM_BASE_H_

#include "rclcpp/rclcpp.hpp"

namespace Fusion
{
    class ReadParamBase
    {
    public:
        ReadParamBase(rclcpp::Node &nh, const std::string &ns) : namespace_(ns), nhPtr_(&nh) {}
        virtual ~ReadParamBase() = default;

    protected:
        template <typename T>
        inline bool GetParam(const std::string &paramName, T &param)
        {
            return GetParamByDefault(paramName, param, param);
        }

        template <typename T>
        inline bool GetParamByDefault(const std::string &paramName, const T &paramDefault, T &param)
        {
            nhPtr_->declare_parameter<T>(paramName, paramDefault);
            if (!nhPtr_->get_parameter(paramName, param))
            {
                std::cout << "The Param: " << paramName << " cannot be read" << std::endl;
                return false;
            }
            std::cout << "Get the param " << paramName << " is " << param << std::endl;
            return true;
        }

        template <typename T>
        inline bool GetParams(const std::string &paramName, T &param)
        {
            return GetParamsByDefault(paramName, param, param);
        }

        template <typename T>
        inline bool GetParamsByDefault(const std::string &paramName, const T &paramDefault, T &param)
        {
            nhPtr_->declare_parameter<T>(paramName, paramDefault);
            rclcpp::Parameter params(paramName);
            if (!nhPtr_->get_parameter(paramName, params))
            {
                std::cout << "The Param: " << paramName << " cannot be read" << std::endl;
                return false;
            }
            param = params.as_double_array();

            std::cout << "Get the param " << paramName << " is ";
            for (auto item : param)
                std::cout << item << " ";
            std::cout << "\n";
            return true;
        }

    protected:
        std::string namespace_;
        rclcpp::Node *nhPtr_;
    };
}

#endif
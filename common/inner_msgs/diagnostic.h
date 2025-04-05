#ifndef FUSION_INNER_MSGS_DIAGNOSITC_H_
#define FUSION_INNER_MSGS_DIAGNOSITC_H_

#include "inner_msgs/msg_base.h"
#include "vector"
#include "eigen3/Eigen/Eigen"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace Fusion {

using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticArrayMsgPtr = DiagnosticArrayMsg::SharedPtr;

struct Diagnostic : MsgBase {
    using Ptr = std::shared_ptr<Diagnostic>;
    
    Diagnostic() = default;

    ~Diagnostic() = default;
   
   Diagnostic(const DiagnosticArrayMsgPtr& msg) : MsgBase(msg->header.stamp)
   {

        diagnostic_msgs::msg::KeyValue keyValue;
        keyValue.key = "source";

        for(size_t i = 0; i < msg->status.size(); i++) {
            if(msg->status[i].hardware_id == "03320003") {
                keyValue.value = "03320003";
                addKeyValue(keyValue);
            }
            else if(msg->status[i].hardware_id == "03320004") {
                keyValue.value = "03320004";
                addKeyValue(keyValue);
            }
            else if(msg->status[i].hardware_id == "03090001") {
                keyValue.value = "03090001";
                addKeyValue(keyValue);
            }
        }
    }

   Diagnostic(const DiagnosticArrayMsg& msg) : MsgBase(msg.header.stamp)
   {

        diagnostic_msgs::msg::KeyValue keyValue;
        keyValue.key = "source";

        for(size_t i = 0; i < msg.status.size(); i++) {
            if(msg.status[i].hardware_id == "03320003") {
                keyValue.value = "03320003";
                addKeyValue(keyValue);
            }
            else if(msg.status[i].hardware_id == "03320004") {
                keyValue.value = "03320004";
                addKeyValue(keyValue);
            }
            else if(msg.status[i].hardware_id == "03090001") {
                keyValue.value = "03090001";
                addKeyValue(keyValue);
            }
        }
    }
    
    Diagnostic(const diagnostic_msgs::msg::KeyValue& keyValue)
    {
        addKeyValue(keyValue);
    }


    void addKeyValue(const diagnostic_msgs::msg::KeyValue& keyValue)
    {
        for(size_t i = 0; i < keyValuesList_.size(); i++) {
            if(keyValuesList_[i].value == keyValue.value) {
                return;
            }
        }

        keyValuesList_.push_back(keyValue);
    }

    Diagnostic(const Diagnostic& inDiag) : MsgBase(inDiag.timestamp)
    {
        keyValuesList_ = inDiag.keyValuesList_;
    }




    std::vector<diagnostic_msgs::msg::KeyValue> keyValuesList_;
};

}

#endif /* FUSION_INNER_MSGS_DIAGNOSITC_H_ */

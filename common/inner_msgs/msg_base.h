#ifndef FUSION_MSG_BASE_H_
#define FUSION_MSG_BASE_H_

#include "builtin_interfaces/msg/time.hpp"
#include "utils/time_stamp.h"

namespace Fusion {
struct MsgBase {
    MsgBase(): timestamp{0ULL} {}
    MsgBase(const TimeStamp& tIn) : timestamp{tIn} {}
    MsgBase(const builtin_interfaces::msg::Time& tIn) : timestamp(tIn) {}
    ~MsgBase() =  default;

    inline const TimeStamp& GetTime() const
    {
        return timestamp;
    }

    TimeStamp timestamp;
};
}
#endif
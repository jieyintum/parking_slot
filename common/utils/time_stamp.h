/*
 * @Author: lchaohua
 * @Date: 2022-08-25 16:15:13
 * @LastEditTime: 2022-08-30 09:37:35
 * @LastEditors: lchaohua
 * @Description: 
 * @FilePath: /hdmap_localization/common/utils/time_stamp.h
 */
#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_

#include "builtin_interfaces/msg/time.hpp"

namespace Fusion {
struct TimeStamp {
    TimeStamp() : nanostamp(0ULL) {}
    TimeStamp(const builtin_interfaces::msg::Time& tIn) : nanostamp(static_cast<uint64_t>(tIn.sec) * static_cast<uint64_t>(1000000000U) + static_cast<uint64_t>(tIn.nanosec)) {}
    TimeStamp(const uint64_t tIn) : nanostamp(tIn) {}
    TimeStamp(const TimeStamp& t) : nanostamp(t.nanostamp) {}
    
    builtin_interfaces::msg::Time ToMsg() const
    {
        builtin_interfaces::msg::Time stampMsg;
        stampMsg.sec = nanostamp / static_cast<uint64_t>(1000000000U);
        stampMsg.nanosec = nanostamp % static_cast<uint64_t>(1000000000);

        return stampMsg;
    }
    double Sec() const
    {
        return static_cast<double>(nanostamp) * 1e-9;
    }
    uint64_t NSec() const
    {
        return nanostamp;
    }

    bool operator == (const TimeStamp& t) const
    {
        return this->nanostamp == t.nanostamp;
    }

    bool operator != (const TimeStamp& t) const
    {
        return this->nanostamp != t.nanostamp;
    }

    void operator = (const TimeStamp& t)
    {
        nanostamp = t.nanostamp;
    }

    void operator = (const builtin_interfaces::msg::Time& tIn)
    {
        nanostamp = tIn.sec * 1e9 + tIn.nanosec;
    }

    bool operator > (const TimeStamp& t) const
    {
        return nanostamp > t.nanostamp;
    }

    bool operator < (const TimeStamp& t) const
    {
        return nanostamp < t.nanostamp;
    }

    bool operator <= (const TimeStamp& t) const
    {
        return nanostamp <= t.nanostamp;
    }

    bool operator >= (const TimeStamp& t) const
    {
        return nanostamp >= t.nanostamp;
    }

    TimeStamp operator - (const TimeStamp& t1) const
    {
        TimeStamp result(this->nanostamp - t1.nanostamp);
        return result;
    }
    
    uint64_t nanostamp;
};
}

#endif
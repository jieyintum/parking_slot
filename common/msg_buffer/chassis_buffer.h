#ifndef FUSION_CHASSIS_MSG_BUFFER_H_
#define FUSION_CHASSIS_MSG_BUFFER_H_

#include "inner_msg/chassis.h"
#include "utils/fixed_size_deque.h"

namespace Fusion
{
    class ChassisMsgBuffer : public FixedSizeDeque<ChassisMsgPtr>
    {
    public:
        ChassisMsgBuffer(const std::uint16_t bufferSize) : FixedSizeDeque<ChassisMsgPtr>(bufferSize) {}
        ~ChassisMsgBuffer() = default;

        bool PopFrontToInnerMsg(Chassis &chassisOutput)
        {
            ChassisMsgPtr msgFront;
            if (this->PopFront(msgFront))
            {
                Chassis inner(msgFront);
                std::swap(inner, chassisOutput);
                return true;
            }
            return false;
        }
    };

    using ChassisBuffer = FixedSizeDeque<Chassis>;
    struct ChassisPtrBuffer : public FixedSizeDeque<Chassis::Ptr>
    {
        ChassisPtrBuffer(const std::uint16_t bufferSize) : FixedSizeDeque<Chassis::Ptr>(bufferSize) {}
        ~ChassisPtrBuffer() = default;

        Chassis::Ptr PopGetBetween(const TimeStamp &timestamp, const TimeStamp &nextTimestamp)
        {
            std::unique_lock<std::mutex> lock(mtx);

            Chassis::Ptr chassisPtrOut = nullptr;
            while (!this->empty() && (this->front()->timestamp <= nextTimestamp))
            {
                const auto &tmpPtr = this->front();
                chassisPtrOut = (timestamp <= tmpPtr->timestamp) ? tmpPtr : nullptr;
                this->pop_front();
            }
            return chassisPtrOut;
        }
    };
}

#endif
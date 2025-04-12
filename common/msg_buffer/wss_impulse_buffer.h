#ifndef FUSION_WSS_IMPULSE_BUFFER_H_
#define FUSION_WSS_IMPULSE_BUFFER_H_

#include "inner_msg/wss_impulse.h"
#include "utils/fixed_size_deque.h"

namespace Fusion
{

    class WssImpulseBuffer : public FixedSizeDeque<WssImpulseMsgPtr>
    {
    public:
        WssImpulseBuffer(const std::uint16_t bufferSize) : FixedSizeDeque<WssImpulseMsgPtr>(bufferSize) {}
        ~WssImpulseBuffer() = default;

        bool PopFrontToInnerMsg(WssImpulse &output)
        {
            WssImpulseMsgPtr msgFront;
            if (this->PopFront(msgFront))
            {
                WssImpulse inner(msgFront);
                std::swap(inner, output);
                return true;
            }
            return false;
        }
    };

    using ImpulseBuffer = FixedSizeDeque<WssImpulse>;
    struct ImpulsePtrBuffer : public FixedSizeDeque<WssImpulse::Ptr>
    {
        ImpulsePtrBuffer(const std::uint16_t bufferSize) : FixedSizeDeque<WssImpulse::Ptr>(bufferSize) {}
        ~ImpulsePtrBuffer() = default;

        std::vector<WssImpulse::Ptr> PopGetBetween(const TimeStamp &nextTimestamp)
        {
            std::unique_lock<std::mutex> lock(mtx);
            std::vector<WssImpulse::Ptr> impulseSelected;
            while (!this->empty())
            {
                const auto &tmpPtr = this->front();
                if ((tmpPtr->timestamp.Sec() - nextTimestamp.Sec()) < 0.0)
                {
                    impulseSelected.push_back(tmpPtr);
                    this->pop_front();
                }
                else
                {
                    break;
                }
            }
            return impulseSelected;
        }

        WssImpulse::Ptr GetBetween(const TimeStamp &timestamp, const TimeStamp &nextTimestamp)
        {
            std::unique_lock<std::mutex> lock(mtx);
            WssImpulse::Ptr chassisPtrOut = nullptr;
            for (auto iter = this->begin(); iter != this->end(); ++iter)
            {
                const auto &tmpPtr = *iter;
                if ((timestamp < tmpPtr->timestamp) && (tmpPtr->timestamp < nextTimestamp))
                {
                    chassisPtrOut = tmpPtr;
                    break;
                }
            }
            return chassisPtrOut;
        }
    };
}

#endif
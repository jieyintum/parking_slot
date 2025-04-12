#ifndef FUSION_IMU_MSG_BUFFER_H_
#define FUSION_IMU_MSG_BUFFER_H_

#include "inner_msg/imu.h"
#include "utils/fixed_size_deque.h"

namespace Fusion
{

    class ImuMsgBuffer : public FixedSizeDeque<ImuMsgPtr>
    {
    public:
        ImuMsgBuffer(const std::uint16_t bufferSize) : FixedSizeDeque<ImuMsgPtr>(bufferSize) {}
        ~ImuMsgBuffer() = default;

        bool PopFrontToInnerMsg(Imu &imuOutput, const bool isTransformToBase)
        {
            ImuMsgPtr msgFront;
            if (this->PopFront(msgFront))
            {
                Imu inner(msgFront, isTransformToBase);
                std::swap(inner, imuOutput);
                return true;
            }
            return false;
        }
    };

    using ImuBuffer = FixedSizeDeque<Imu>;
    using ImuPtrBuffer = FixedSizeDeque<Imu::Ptr>;
}

#endif
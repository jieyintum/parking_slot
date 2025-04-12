
#ifndef CEM_POSITION_BUFFER_H
#define CEM_POSITION_BUFFER_H

#include "utils/time_fixed_size_deque.h"
#include "gloc_hdm.h"

namespace Fusion
{

    class CEMPositionBuffer : public TimeFixedSizeDeque<CEMPosition::Ptr>
    {
    public:
        static CEMPositionBuffer &GetInstance()
        {
            static CEMPositionBuffer instance(200U);
            return instance;
        }

    private:
        CEMPositionBuffer(const std::uint16_t fixedSize) : TimeFixedSizeDeque<CEMPosition::Ptr>(fixedSize) {}
        CEMPositionBuffer(CEMPositionBuffer &) = delete;
        CEMPositionBuffer &operator=(const CEMPositionBuffer &) = delete;

        CEMPosition::Ptr Interpolation(const CEMPosition::Ptr &ins_1, const CEMPosition::Ptr &ins_2, const TimeStamp &timestamp)
        {
            return ins_1;
        }
    };
}

#endif
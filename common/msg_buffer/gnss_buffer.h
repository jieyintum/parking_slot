#ifndef FUSION_GNSS_BUFFER_H
#define FUSION_GNSS_BUFFER_H

#include "inner_msg/gnss.h"
#include "utils/fixed_size_deque.h"

namespace Fusion {
class GnssMsgBuffer : public FixedSizeDeque<GnssMsgPtr> {
public:
    GnssMsgBuffer(const std::uint16_t bufferSize) : FixedSizeDeque<GnssMsgPtr>(bufferSize) {}
    ~GnssMsgBuffer() = default;

    bool PopFrontToInnerMsg(Gnss& gnssOutput)
    {
        GnssMsgPtr msgFront;
        if (this->PopFront(msgFront)) {
            Gnss inner(msgFront);
            std::swap(inner, gnssOutput);
            return true;
        }
        return false;
    }
};

using GnssBuffer = FixedSizeDeque<Gnss>;
using GnssPtrBuffer = FixedSizeDeque<Gnss::Ptr>;
}


#endif
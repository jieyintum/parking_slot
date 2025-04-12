#ifndef FUSION_WSS_IMPULSE_H_
#define FUSION_WSS_IMPULSE_H_

#include <cstdint>
#include <cmath>
#include "vehicle_msgs/msg/can_copy_cp2_ap10ms_struct.hpp"
#include "utils/time_stamp.h"

namespace Fusion
{
    using WssImpulseMsg = vehicle_msgs::msg::CanCopyCp2Ap10msStruct;
    using WssImpulseMsgPtr = WssImpulseMsg::SharedPtr;

    struct WssImpulse
    {
        WssImpulse() = default;
        WssImpulse(const WssImpulseMsgPtr &msgPtr) : wssImpulseCountRl(msgPtr->ibs10mspdu01.ildrvnwhlrotldistplsctr),
                                                     wssImpulseCountRr(msgPtr->ibs10mspdu01.irdrvnwhlrotldistplsctr),
                                                     wssImpulseCountFl(msgPtr->ibs10mspdu02.ilnondrvnwhlrotldistpc),
                                                     wssImpulseCountFr(msgPtr->ibs10mspdu02.irnondrvnwhlrotldistpc)
        {
            timestamp.nanostamp = (static_cast<uint64_t>(msgPtr->imcu10mspdu39.iimcu_10ms_group39_reserve01) << 48) +
                                  (static_cast<uint64_t>(msgPtr->rbm10mspdu05.irbmwhldrvndircn_group01_reserve02) << 32) +
                                  (static_cast<uint64_t>(msgPtr->epssfcanfd010mspdu01.ieps_sfcanfd_010ms_group01_reserve06) << 16) +
                                  msgPtr->epssfcanfd010mspdu01.ieps_sfcanfd_010ms_group01_reserve05;
            timestamp.nanostamp *= 1000000U;
        }

        void SetReturn(const bool back)
        {
            isReturn = back;
        }

        TimeStamp timestamp;

        uint16_t wssImpulseCountRl = 0U;
        uint16_t wssImpulseCountRr = 0U;
        uint16_t wssImpulseCountFl = 0.0F;
        uint16_t wssImpulseCountFr = 0.0F;
        bool isReturn = false;

        using Ptr = std::shared_ptr<WssImpulse>;
    };
}

#endif
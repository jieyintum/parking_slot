#ifndef FUSION_PSE_FRAMEBASE_H_
#define FUSION_PSE_FRAMEBASE_H_

#include "utils/time_stamp.h"
#include "inner_msgs/odometry.h"
namespace Fusion
{
    namespace PSE
    {
        struct FrameBase
        {
            TimeStamp stamp;
            Odometry poseOb;
        };
    }
}
#endif
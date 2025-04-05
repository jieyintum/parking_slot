#ifndef FUSION_PSE_PLDFRAME_H_
#define FUSION_PSE_PLDFRAME_H_
#include <array>
#include <Eigen/Eigen>
#include <memory>

#include "frame_base.h"
#include "vision_freespace.h"
#include "fusion_slot.h"

namespace Fusion {
namespace PSE {
struct PldFrame : public FrameBase {
    PerceptSlot::Ptr pld_;
    VisionFreespace::Ptr vsFs_ = nullptr;
    visionObjVector::Ptr vsObject_ = nullptr;
    UssMsg::Ptr uss_;
    Odometry ussPoseOb;
    FSStruct::Ptr fs_;

    bool isParkSystemWork;
    bool isLastFrameSystemWork;


    std::vector<FusionSlot::Ptr> detectSlots_;
    std::vector<FusionSlot::Ptr> outputSlots_;
};
}
}

#endif
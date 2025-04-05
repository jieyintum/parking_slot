#ifndef FUSION_PSE_FUSIONBASE_H_
#define FUSION_PSE_FUSIONBASE_H_

#include <memory>
#include "rclcpp/rclcpp.hpp"

#if DEBUG_MODE

#include <visualization_msgs/msg/marker_array.hpp>

#endif

#include "frame_manager.h"

namespace Fusion {
namespace PSE {
enum class FusionType {
    PLD = 0U
};

template<typename T>
class FusionBase {
public:
    using Ptr = std::shared_ptr<FusionBase>;

    FusionBase(rclcpp::Node& nh, FrameManager& frameManager) :
            nh_(nh), manager_(frameManager)
    {}

    virtual ~FusionBase() = default;

    virtual void Process(T& frame)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (MakeFrame(frame)) {
            Detect(frame);
            Track(frame);
            Output(frame);
#if DEBUG_MODE
            Debug(frame);
#endif
        }
    }

    virtual void PubOut()
    {
        std::unique_lock<std::mutex> lock(mtx);
        T frame;
        if (!OdomBuffer::GetInstance().Empty()) {
            frame.poseOb = * OdomBuffer::GetInstance().Back();
            Publish(frame);
        }
    }

protected:
    virtual bool MakeFrame(T& frame) = 0;

    virtual void Detect(T& frame) = 0;

    virtual void Track(T& frame) = 0;

    virtual void Output(T& frame) = 0;

    virtual void Debug(T& frame) = 0;

    virtual void Publish(T& frame) = 0;

    std::mutex mtx;

protected:
    rclcpp::Node& nh_;
    FrameManager& manager_;
};
}
}
#endif
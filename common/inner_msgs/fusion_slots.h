#ifndef INNER_MSGS_FUSION_SLOTS_H_
#define INNER_MSGS_FUSION_SLOTS_H_

#include "inner_msgs/msg_base.h"
#include "cem_park_msgs/msg/slots_list.hpp"

namespace Fusion {

using SlotMsg = cem_park_msgs::msg::Slot;
using SlotMsgPtr = SlotMsg::SharedPtr;
struct SingleSlot {
    uint64_t id;
    bool isOccupied;
    float confidence;
    uint8_t type;
    uint8_t location;   // left or right
    uint8_t contributingSensors;
    uint8_t slotSideState;
    float width;
    float depth;
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d p3;
    float angle;
    uint8_t slot_bottom_margin_type = 0U;
    bool hasWheelStop;
    Eigen::Vector3d wheelStopPoint;

    uint8_t occupiedType; //1: by vehicle 2:by groundLocker 3:by others 0:not occupied
    bool releaseState;


    SlotMsg ToMsg()
    {
        SlotMsg slotMsg;
        slotMsg.slot_id = this->id;
        slotMsg.is_slot_being_occupied = this->isOccupied;
        slotMsg.slot_confidence = this->confidence;
        slotMsg.slot_type = this->type;
        slotMsg.location_to_ego_car = this->location;
        slotMsg.contributing_sensors = this->contributingSensors;
        slotMsg.slot_side_state = this->slotSideState;
        slotMsg.width = this->width;
        slotMsg.depth[0] = depth;
        slotMsg.depth[1] = depth;
        slotMsg.slot_p0p3_angle = this->angle;
        slotMsg.slot_p1p2_angle = this->angle;
        slotMsg.slot_bottom_edge_angle = M_PI - this->angle;

        slotMsg.p0.x = p0.x();
        slotMsg.p0.y = p0.y();
        slotMsg.p0.z = p0.z();
        slotMsg.p1.x = p1.x();
        slotMsg.p1.y = p1.y();
        slotMsg.p1.z = p1.z();
        slotMsg.p2.x = p2.x();
        slotMsg.p2.y = p2.y();
        slotMsg.p2.z = p2.z();
        slotMsg.p3.x = p3.x();
        slotMsg.p3.y = p3.y();
        slotMsg.p3.z = p3.z();

        slotMsg.wheel_stop_flag = this->hasWheelStop;
        slotMsg.wheel_stop.x = this->wheelStopPoint.x();
        slotMsg.wheel_stop.y = this->wheelStopPoint.y();
        slotMsg.wheel_stop.z = this->wheelStopPoint.z();

        slotMsg.slot_bottom_margin_type = slot_bottom_margin_type;

        slotMsg.occupied_type = this->occupiedType;
        slotMsg.release_state = this->releaseState;

        return slotMsg;
    }

    // bool left_edge_point_flag
    // Point3f left_edge_point
    // bool right_edge_point_flag
    // Point3f right_edge_point
};


using SlotsListMsg = cem_park_msgs::msg::SlotsList;
using SlotsListMsgPtr = SlotsListMsg::SharedPtr;
struct FusionSlots : MsgBase {
    using Ptr = std::shared_ptr<FusionSlots>;
    
    FusionSlots(const SlotsListMsgPtr& msg) {
        timestamp.nanostamp = msg->lads_perception_fusion_header.time_stamp;
    }

public:
    FusionSlots() = default;
    ~FusionSlots() = default;


};



}
#endif
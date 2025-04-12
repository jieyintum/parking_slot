#ifndef FUSION_INNER_MSG_PERCEPT_SLOT_H_
#define FUSION_INNER_MSG_PERCEPT_SLOT_H_

#include <cstring>
#include "vision_msgs/msg/common_struct.hpp"
#include "inner_msgs/msg_base.h"
#include "utils/pld_utils.h"

namespace Fusion
{
    using PerceptSlotMsg = vision_msgs::msg::CommonStruct;
    using PerceptSlotMsgPtr = PerceptSlotMsg::SharedPtr;

    // slot's block
    struct Block
    {
        PldUtils::Point2f BlockPot;
        uint8_t Block_type; // 0:Default, without block, 1:predictive block, 2:detective block
        float Block_conf;
        Block()
        {
            Block_type = 0;
            Block_conf = 0.0f;
        }
    };

    // parking slot corner point
    struct CornerPts
    {
        PldUtils::Point2f CornerPot;
        float CornerPot_conf;
        CornerPts()
        {
            CornerPot_conf = 0.0f;
        }
    };

    // dangerous points
    struct EdgePts
    {
        PldUtils::Point2f EdgePot;
        uint8_t EdgePot_type; // 0:Default, without EdgePoint, 1:predictive EdgePoint, 2:detective EdgePoint
        float EdgePot_conf;
        EdgePts()
        {
            EdgePot_type = 0;
            EdgePot_conf = 0.0f;
        }
    };

    // one single parking slot
    struct PLD_DL_Result
    {
        CornerPts slot_GroundPts[4];
        float slot_confidence;
        uint8_t slot_loc;  // 0--default, 1--Left, 2--Right,four corners is on right side of vehicle-coordinate, 3--unknown
        uint8_t slot_type; // 0--default, 1--paralel, 2--vertical  3--diagonal
        uint16_t slot_id;
        uint8_t slot_status;        // 0--default, 1--available, 2--occupied
        uint8_t corner_points_type; // 0--default 1--kf.predict 2--kf.correct && detect_result_uncorrelated
        Block block_GroundPt;       // under vehicle coordinate
        EdgePts edge_GroundPt[2];   // under vehicle coordinate

        PLD_DL_Result()
        {
            slot_confidence = 0.0f;
            slot_loc = 0;
            slot_type = 0;
            slot_id = 0;
            slot_status = 0;
            corner_points_type = 0;
        }
    };

    struct PLD_DL_Result_One_Frame
    {
        PLD_DL_Result DL_Result_one_frame[16];
        uint16_t slot_num;
        uint32_t pld_frameID;             // current frameID
        unsigned long long pld_timestamp; // time stamp

        PLD_DL_Result_One_Frame()
        {
            slot_num = 0;
            pld_frameID = 0;
            pld_timestamp = 0;
        }
    };

    struct PerceptSlot : MsgBase
    {
        using Ptr = std::shared_ptr<PerceptSlot>;

        PerceptSlot(const PerceptSlotMsgPtr &msg, const Fusion::veh_params &vehicleParams) : MsgBase(msg->header.stamp),
                                                                                             frameId(msg->frameid), cameraId(msg->cameraid), structSize(msg->struct_size)
        {
            if (static_cast<uint64_t>(structSize) > sizeof(PLD_DL_Result_One_Frame))
            {
                structSize = sizeof(PLD_DL_Result_One_Frame);
            }
            memcpy(&frame, &msg->data[0], structSize);
            for (uint16_t i = 0U; i < this->frame.slot_num; ++i)
            {
                auto &slot = this->frame.DL_Result_one_frame[i];
                for (int j = 0; j < 4; ++j)
                {
                    PldUtils::PointCoordinateTransfer(slot.slot_GroundPts[j].CornerPot, vehicleParams);
                }
                for (int j = 0; j < 2; ++j)
                {
                    PldUtils::PointCoordinateTransfer(slot.edge_GroundPt[j].EdgePot, vehicleParams);
                }
                PldUtils::PointCoordinateTransfer(slot.block_GroundPt.BlockPot, vehicleParams);
            }
        }

        int32_t frameId;
        int32_t cameraId;
        int32_t structSize;
        PLD_DL_Result_One_Frame frame;
    };
}
#endif

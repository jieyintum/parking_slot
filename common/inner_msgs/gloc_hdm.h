
#ifndef GLOBAL_LOC_HDM_H_
#define GLOBAL_LOC_HDM_H_

#include "loc_msgs/msg/hd_localization.hpp"
#include "loc_msgs/msg/hdm_fusion.hpp"
#include "map_pose.h"
#include "map_engine_struct.h"

namespace Fusion {

enum MatchFlag {
    match_false = 0,
    fvcmlane_match = 1,
    roadedge_match = 2,
    bevlane_match = 3,
};

enum MatchState {
    kmatch_fail = 0,
    kmatch_both = 1,
    kmatch_lateral = 2,
    kmatch_longitudinal = 3
};

enum FailureMatchState {
    kcheck_none = 0, 
    kcheck_sensor_fail = 1,
    kcheck_roadedge_fail = 2,
    kcheck_lanedistace_fail = 3,
    kcheck_lane_slope_fail = 4,
    kcheck_lanecurve_fail = 5,
    kcheck_lanewidth_fail = 6,
    kcheck_trafficsign_fail = 7
};

enum FailureFvcmState {
    kfvcm_none = 0,
    kfvcm_nodata = 1,
    kfvcm_host_lost = 2,
    kfvcm_host_short = 3,
    kfvcm_data_invalid = 4
};

enum FailureLaneState {
    klane_no_data = 0,
    klane_both_sides = 1,
    klane_one_side_only = 2,
};

struct HDMapMatchState {
    HDMapMatchState()
    {
        state = MatchState::kmatch_fail;
        failure_match_state = FailureMatchState::kcheck_none;
        failure_fvcm_state = FailureFvcmState::kfvcm_none;
        failure_hdlane_state = FailureLaneState::klane_no_data;
    }
    MatchState state;
    FailureMatchState failure_match_state;
    FailureFvcmState  failure_fvcm_state;
    FailureLaneState  failure_hdlane_state;
};

enum HDLocFailState {
    kvalid = 0,
    kfailprediction = 1,
    kinvalid = 2,
    kRoadLevelLoc = 3
};

class FusionResult {
public:
    FusionResult():
    imutmsputc{0}, hdmap_state{Hdm::HDMapState::khdmap_nolink}, init_heading{0.0}, fusion_heading{0.0},
    hd_ego_left{0,0,0}, hd_ego_right{0,0,0}, me_ego_left{0,0,0}, me_ego_right{0,0,0},
    fusion_PosLL{0,0,}, delta_p{0,0}, init_PosLL{0,0}
    {
        match_state = HDMapMatchState();
    }
    ~FusionResult() = default;
    using Ptr = std::shared_ptr<FusionResult>;

    loc_msgs::msg::HdmFusion ToFusionResultMsg() const
    {
        loc_msgs::msg::HdmFusion msg;
        msg.imutmsputc = imutmsputc;
        loc_msgs::msg::HdMapState hdmap_state_;
        hdmap_state_.state = hdmap_state;
        msg.hdmap_state = hdmap_state_;
        loc_msgs::msg::HdMapMatchState match_state_;
        match_state_.state = match_state.state;
        match_state_.failure_fvcm_state = match_state.failure_fvcm_state;
        match_state_.failure_match_state = match_state.failure_match_state;
        msg.match_state = match_state_;
        msg.init_heading = init_heading;
        std::copy(hd_ego_left.begin(), hd_ego_left.end(), msg.hd_ego_left.begin());
        std::copy(hd_ego_right.begin(), hd_ego_right.end(), msg.hd_ego_right.begin());
        std::copy(me_ego_left.begin(), me_ego_left.end(), msg.me_ego_left.begin());
        std::copy(me_ego_right.begin(), me_ego_right.end(), msg.me_ego_right.begin());
        std::copy(fusion_PosLL.begin(), fusion_PosLL.end(), msg.fusion_posll.begin());
        std::copy(delta_p.begin(), delta_p.end(), msg.delta_p.begin());
        std::copy(init_PosLL.begin(), init_PosLL.end(), msg.init_posll.begin());

        return msg;
    }

    uint64_t imutmsputc;
    Hdm::HDMapState hdmap_state;    
    HDMapMatchState  match_state;
    double init_heading;
    double fusion_heading;
    std::array<double,3> hd_ego_left;
    std::array<double,3> hd_ego_right;
    std::array<double,3> me_ego_left;
    std::array<double,3> me_ego_right;
    std::array<double,2> fusion_PosLL;
    std::array<double,2> delta_p;
    std::array<double,2> init_PosLL;
};


enum LocalizationState {
    klocalization_invalid = 0,
    kroad_localization = 1,
    kips_localization = 2,
    kfusion_localization = 3,
};

enum FailState_BevLane {
    kdata_valid = 0,
    kdata_lost = 1,
    kdata_invalid = 2
};

struct NoCam_Pos {
    uint64_t NoCam_SysTime;
    uint64_t NoCam_LinkID;
    std::uint8_t NoCam_LaneNum;
    int32_t NoCam_Lon;
    int32_t NoCam_Lat;
    float NoCam_Heading;
    uint32_t NoCam_Confidence;
};

class CEMLocState {
public:
    CEMLocState()
    {
        localization_state = klocalization_invalid;
        failstate_bevlane = kdata_valid;
        hdmapfusion_state = 0;
    }
    ~CEMLocState() = default;

    LocalizationState localization_state;
    FailState_BevLane failstate_bevlane;
    std::uint8_t  hdmapfusion_state;
};

class EHPNoCamPosition {
public:
    EHPNoCamPosition() = default;
    ~EHPNoCamPosition() = default;

    EHPNoCamPosition(const NoCam_Pos& msg):
        system_time{msg.NoCam_SysTime}, relapos_linkid{msg.NoCam_LinkID},
        relapos_lanenum{msg.NoCam_LaneNum},  absopos_heading{msg.NoCam_Heading},
        absopos_confidence{msg.NoCam_Confidence}
    {
        absopos_long = static_cast<double>(msg.NoCam_Lon * 360.0 / pow(2,32));
        absopos_lat = static_cast<double>(msg.NoCam_Lat * 360.0 / pow(2,32));
    }
    
    uint64_t system_time = 0;
    uint64_t relapos_linkid = 0;
    std::uint8_t  relapos_lanenum = 0;
    double absopos_long = 0.0;
    double absopos_lat = 0.0;
    float absopos_heading = 0.0;
    uint32_t absopos_confidence = 0;
};

class CEMPosition: public MsgBase {
public:
    CEMPosition() = default;
    ~CEMPosition() = default;
    using Ptr = std::shared_ptr<CEMPosition>;
    uint64_t timestamp = 0;
    uint64_t relapos_linkid = 0;
    std::uint8_t relapos_lanenum = 0;
    uint32_t relapos_disleft = 0.0;
    uint32_t relapos_disright = 0.0;
    float relapos_headingleft = 0.0;
    float relapos_headingright = 0.0;
    double absopos_long = 0.0;
    double absopos_lat = 0.0;
    float absopos_heading = 0.0;
    float absopos_long_std = 0.0;
    float absopos_lat_std = 0.0;
    float absopos_heading_std = 0.0;
    std::uint8_t absopos_pos_confidence = 0;
    float absopos_imu_acc[3] = {0.0, 0.0, 0.0};
    float absopos_imu_gyro[3] = {0.0, 0.0, 0.0};
    uint64_t pos_timestamp = 0;
    uint64_t pos_age = 0;
    uint32_t pos_pathid = 0;
    uint32_t pos_offset = 0;
    uint32_t pos_accuracy = 0;
    int32_t pos_deviation = 0; 
    float pos_speed = 0.0;
    float pos_rela_heading = 0.0;
    float pos_probability = 0.0;
    std::uint8_t pos_current_lane = 0;
    uint32_t pos_prefer_path = 0;
    std::uint8_t fail_state_localization = 0;
    std::uint8_t fail_state_ips = 0;
    std::uint8_t fail_state_camera = 0;
    std::uint8_t fail_state_hdmap = 0;
    std::uint8_t fail_state_vehcle = 0;
    std::uint8_t fail_state_imu = 0;
    std::uint8_t fail_state_other = 0;
    std::uint16_t fail_state_wheel = 0;
    std::uint8_t fail_state_gnss = 0;
    EHPNoCamPosition nocam_position;
    CEMLocState loc_state;

    loc_msgs::msg::HdLocalization ToMsg() const
    {
        loc_msgs::msg::HdLocalization msg;
        msg.imu_time_stamp_utc = timestamp / static_cast<uint64_t>(1e3);
        msg.system_time = timestamp / static_cast<uint64_t>(1e3);
        msg.relative_position_link_id = relapos_linkid;
        msg.relative_position_lane_number = relapos_lanenum;
        msg.relative_position_distance_left = relapos_disleft;
        msg.relative_position_distance_right = relapos_disright;
        msg.relative_position_heading_left = relapos_headingleft;
        msg.relative_position_heading_right = relapos_headingright;
        msg.absolute_position_longitude = absopos_long;
        msg.absolute_position_latitude = absopos_lat;
        msg.absolute_position_heading = absopos_heading;
        msg.absolute_position_longitude_std = absopos_long_std;
        msg.absolute_position_latitude_std = absopos_lat_std;
        msg.absolute_position_heading_std = absopos_heading_std;
        msg.absolute_position_confidence = absopos_pos_confidence;

        loc_msgs::msg::HdLocNoCamPosition msg_nocam;
        msg_nocam.system_time = nocam_position.system_time / static_cast<uint64_t>(1e3);
        msg_nocam.relative_position_link_id = nocam_position.relapos_linkid;
        msg_nocam.relative_position_lane_number = nocam_position.relapos_lanenum;
        msg_nocam.absolute_position_longitude = nocam_position.absopos_long; 
        msg_nocam.absolute_position_latitude = nocam_position.absopos_lat; 
        msg_nocam.absolute_position_heading = nocam_position.absopos_heading; 
        msg_nocam.absolute_position_confidence = nocam_position.absopos_confidence; 
        msg.absolute_position_imu_acc[0] = absopos_imu_acc[0];
        msg.absolute_position_imu_acc[1] = absopos_imu_acc[1];
        msg.absolute_position_imu_acc[2] = absopos_imu_acc[2];
        msg.absolute_position_imu_gyro[0] = absopos_imu_gyro[0];
        msg.absolute_position_imu_gyro[1] = absopos_imu_gyro[1];
        msg.absolute_position_imu_gyro[2] = absopos_imu_gyro[2];
        msg.no_camera_position = msg_nocam;
        msg.position_timestamp = timestamp;
        msg.position_age = pos_age;
        msg.position_path_id = pos_pathid;
        msg.position_offset = pos_offset;
        msg.position_accuracy = pos_accuracy;
        msg.position_deviation = pos_deviation;
        msg.position_speed = pos_speed;
        msg.position_relative_heading = pos_rela_heading;
        msg.position_probability = pos_probability;
        msg.position_current_lane = pos_current_lane;
        msg.position_prefer_path = pos_prefer_path;
        msg.fail_state_localization = fail_state_localization;
        msg.fail_state_ips = fail_state_ips;
        msg.fail_state_camera = fail_state_camera;
        msg.fail_state_hdmap = fail_state_hdmap;
        msg.fail_state_vehcle = fail_state_vehcle;
        msg.fail_state_imu = fail_state_imu;
        msg.fail_state_other = fail_state_other;
        msg.fail_state_wheel = fail_state_wheel;
        msg.fail_state_gnss = fail_state_gnss;
        msg.hdlocalization_state.localization_state = loc_state.localization_state;
        msg.hdlocalization_state.hdmapfusion_state = loc_state.hdmapfusion_state;
        msg.hdlocalization_state.failstate_bevlanes = loc_state.failstate_bevlane;
        return msg;
    } 
};

} 
#endif
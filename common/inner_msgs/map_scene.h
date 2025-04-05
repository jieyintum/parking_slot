/*
 * @Author: guxiaojie
 * @Date: 2022-07-18 13:57:23
 * @LastEditTime: 2022-07-18 14:47:23
 * @LastEditors: guxiaojie
 * @Description: Do not edit
 * @FilePath: /loc_hdm/include/common/data_type/sensor/mapscene.h
 */
#ifndef FUSION_MAP_SCENE_MSG_H_
#define FUSION_MAP_SCENE_MSG_H_
#include <memory>
#include "cem_interfaces/msg/map_scene.hpp"

namespace Fusion {
using MapSceneMsg = cem_interfaces::msg::MapScene;
using MapSceneMsgPtr = MapSceneMsg::SharedPtr;

struct MapScene {
using Ptr = std::shared_ptr<MapScene>;
public:
    MapScene(const MapSceneMsgPtr& msg)
    {
        currentRoadType = msg->current_road_type;
        nextRoadType = msg->next_road_type;
        distanceToNextRoad = msg->distance_to_next_road_type;
    }
    MapScene() = default;
    ~MapScene() = default;

    void UpdateByMsg(const cem_interfaces::msg::MapScene::SharedPtr& msg) {
        std::lock_guard<std::mutex> lck(mapSceneMutex_);
        currentRoadType = msg->current_road_type;
        nextRoadType = msg->next_road_type;
        distanceToNextRoad = msg->distance_to_next_road_type;
    }

    std::uint8_t GetCurrentRoadType() const
    {
        return currentRoadType;
    }

public:
    std::mutex mapSceneMutex_;
    std::uint8_t currentRoadType = 3u;
    std::uint8_t nextRoadType = 3u;
    double  distanceToNextRoad = 3000.0;
};
} // namespace Fusion

#endif

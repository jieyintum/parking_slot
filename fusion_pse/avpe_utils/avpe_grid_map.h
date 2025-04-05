#ifndef AVPE_GRID_MAP
#define AVPE_GRID_MAP

#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <Eigen/Eigen>
#include <opencv4/opencv2/opencv.hpp>

#include "utils/geom_utils.h"
#include "utils/grid_map.h"

namespace Fusion {

struct AvpeGrid : public Grid {
    using Ptr = std::shared_ptr<AvpeGrid>;

    void AddCoords(Eigen::Vector3d coord)
    {
        coords.push_back(coord);
    }

    void AddTypes(int type)
    {
        types.push_back(type);
    }

    Eigen::Vector3d GetCenterCoord()
    {
        Eigen::Vector3d gridSum{0., 0., 0.};
        for (const auto& coord : coords) {
            gridSum += coord;
        }
        return gridSum / coords.size();
    }

    GridIndex GetGridIndex()
    {
        return this->index;
    }

    std::vector<Eigen::Vector3d> coords;

    std::vector<int> types{};

    uint32_t pointNum = 0;

    uint32_t nbrNum = 0;

    GridIndex index;


};

class AvpeGridMap : public GridMap<AvpeGrid> {

public:

    using Ptr = std::shared_ptr<AvpeGridMap>;

    AvpeGridMap() : GridMap<AvpeGrid>(0., 0., 0., 0., 1.)
    {};

    AvpeGridMap(const float minX, const float minY, const float deltaX, const float deltaY, const float precision) :
            GridMap<AvpeGrid>(minX, minY, deltaX, deltaY, precision)
    {
    //    std::cout << "Set grid map" << std::endl;
    //    std::cout << "minx: " << minX << std::endl;
    //    std::cout << "miny: " << minY << std::endl;
    //    std::cout << "deltax: " << deltaX << std::endl;
    //    std::cout << "deltay: " << deltaY << std::endl;
    //    std::cout << "precision: " << precision << std::endl;
    }

    bool InsertElem(Eigen::Vector3d coord, int type)
    {
        double xCoord = coord.x();
        double yCoord = coord.y();
        GridIndex index = this->GetRowCol(static_cast<float>(xCoord), static_cast<float>(yCoord));
        if (IsInRange(index)) {
            this->map_[index.row][index.col]->AddCoords(coord);
            this->map_[index.row][index.col]->AddTypes(type);
            ++this->map_[index.row][index.col]->pointNum;
            activateIndex_.insert(index);
            return true;
        }
        return false;
    }

    bool InsertElem(Eigen::Vector3d coord)
    {
        double xCoord = coord.x();
        double yCoord = coord.y();
        GridIndex index = this->GetRowCol(static_cast<float>(xCoord), static_cast<float>(yCoord));
        if (IsInRange(index)) {
            this->map_[index.row][index.col]->AddCoords(coord);
            ++this->map_[index.row][index.col]->pointNum;
            activateIndex_.insert(index);
            return true;
        }
        return false;
    }

    bool GetElem(GridIndex index, AvpeGrid::Ptr& elem)
    {
        if (IsInRange(index)) {
            elem = this->map_[index.row][index.col];
            elem->index = index;
            return true;
        }
        return false;
    }

    void SortBoundary(std::vector<Eigen::Vector3d>& boundaryPoints)
    {
        struct {
            bool operator()(Eigen::Vector3d& pt1, Eigen::Vector3d& pt2) const
            {
                return atan2(pt1.y(), pt1.x()) < atan2(pt2.y(), pt2.x());
            }
        } angleSorter;

        Eigen::Vector3d boundaryBase{0., 0., 0.};
        for (const auto& point : boundaryPoints) {
            boundaryBase += point;
        }
        boundaryBase /= boundaryPoints.size();

        for (auto& point : boundaryPoints) {
            point -= boundaryBase;
        }

        std::sort(boundaryPoints.begin(), boundaryPoints.end(), angleSorter);

        for (auto& point : boundaryPoints) {
            point += boundaryBase;
        }

    }

    void FindContour(std::vector<std::vector<Eigen::Vector3d>>& contours)
    {
        SearchMethod clusterMethod = SearchMethod::FOUR_NEIGHBOUR;
        // cluster
        std::vector<Cluster> clusters = this->Clustering(clusterMethod);

        for (const auto& cluster : clusters) {
            AvpeGrid::Ptr avpGridPtr = nullptr;
            std::vector<Eigen::Vector3d> boundaryPoints{};
            for (const auto& gridIndex : cluster) {
                this->GetElem(gridIndex, avpGridPtr);
                if (avpGridPtr == nullptr) {
                    continue;
                }
                // innner point
                if (avpGridPtr->nbrNum == 4U) {
                    continue;
                }
                boundaryPoints.emplace_back(avpGridPtr->GetCenterCoord());
            }
            contours.push_back(boundaryPoints);
        }
    }

    void FindCounters(std::vector<std::vector<Eigen::Vector3d>>& counters, std::vector<std::vector<int>>& clusterTypess)
    {
        SearchMethod clusterMethod = SearchMethod::FOUR_NEIGHBOUR;
        // cluster
        std::vector<Cluster> clusters = this->Clustering(clusterMethod);

        for (const auto& cluster : clusters) {
            AvpeGrid::Ptr avpGridPtr = nullptr;
            std::vector<Eigen::Vector3d> boundaryPoints{};
            std::vector<int> types{};
            for (const auto& gridIndex : cluster) {
                this->GetElem(gridIndex, avpGridPtr);
                if (!avpGridPtr->types.empty()) {
                    types.push_back(avpGridPtr->types.front());
                }
                if (avpGridPtr == nullptr) {
                    continue;
                }
                // innner point
                if (clusterMethod == SearchMethod::FOUR_NEIGHBOUR && avpGridPtr->nbrNum == 4U) {
                    continue;
                }
                boundaryPoints.push_back(avpGridPtr->GetCenterCoord());
            }
            clusterTypess.push_back(types);
            boundaryPoints.push_back(boundaryPoints[0]);
            std::vector<Eigen::Vector3d> convexHull;
            Geometry::MakeConvexHull(boundaryPoints, convexHull);
            counters.push_back(convexHull);
        }
    }
};

class AvpeGridMapImg : public AvpeGridMap {

public:

    using Ptr = std::shared_ptr<AvpeGridMapImg>;

    AvpeGridMapImg() : AvpeGridMap(0., 0., 0., 0., 1.)
    {};

    AvpeGridMapImg(const float minX, const float minY, const float deltaX, const float deltaY, const float precision) :
            AvpeGridMap(minX, minY, deltaX, deltaY, precision)
    {
       mask_ = cv::Mat::zeros(size_.row, size_.col, CV_8UC1);
    }

    bool InsertElem(Eigen::Vector3d coord, int type)
    {
        double xCoord = coord.x();
        double yCoord = coord.y();
        GridIndex index = this->GetRowCol(static_cast<float>(xCoord), static_cast<float>(yCoord));
        if (IsInRange(index)) {
            this->map_[index.row][index.col]->AddCoords(coord);
            this->map_[index.row][index.col]->AddTypes(type);
            mask_.at<uchar>(index.row, index.col) = 255;
            ++this->map_[index.row][index.col]->pointNum;
            activateIndex_.insert(index);
            return true;
        }
        return false;
    }

    void FindCounters(std::vector<std::vector<Eigen::Vector3d>>& counters, std::vector<std::vector<int>>& clusterTypes)
    {
        std::vector<std::vector<cv::Point>> contours; //轮廓点集
        cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours.size(); i++) {
            std::vector<Eigen::Vector3d> boundaryPoints{};
            std::vector<int> types{};
            for (const auto& point : contours[i]) {
                AvpeGrid::Ptr avpGridPtr = nullptr;
                auto gridIndex = GridIndex(point.y, point.x);
                this->GetElem(gridIndex, avpGridPtr);
                if (avpGridPtr == nullptr) {
                    continue;
                }
                if (!avpGridPtr->types.empty()) {
                    types.push_back(avpGridPtr->types.front());
                }
                if (avpGridPtr->coords.empty()) {
                    continue;
                }
                boundaryPoints.push_back(avpGridPtr->GetCenterCoord());
            }
            boundaryPoints.push_back(boundaryPoints[0]);
            clusterTypes.push_back(types);
            std::cout << boundaryPoints.front().transpose() << std::endl;
            counters.push_back(boundaryPoints);
        }
    }

    void Reset()
    {
        for (const auto& index : activateIndex_) {
            map_[index.row][index.col] = std::make_shared<AvpeGrid>();
        }
        activateIndex_.clear();
        mask_ = 0;
    }

private:
    cv::Mat mask_ {};
};

}


#endif // AVPE_GRID_MAP

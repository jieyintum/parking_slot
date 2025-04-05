//
// Created by igs on 2023/1/9.
//

#ifndef PARKING_SE_FUSION_POLY_LINE_FITTING_H
#define PARKING_SE_FUSION_POLY_LINE_FITTING_H

#include <random>
#include <iostream>
#include <ctime>
#include <set>
#include <cassert>
#include <climits>
#include "lane_fusion/lane_frame.h"

namespace Fusion {
namespace PSE {

class DouglasPeucker {
public:
    DouglasPeucker() = default;

    ~DouglasPeucker() = default;
    /**
    * @brief init input date, set all point's isRemoved to false
    * @param input -> dataIn
    * @param output -> dataIn_
    * @return void
    */
    void Init(const std::vector<Point::Ptr>& dataIn);

    /**
    * @brief Generate segmented points according dist threshold, begin point, end point
     * set segmented point's isRemoved to false, other points's isRemoved set to true
    * @param input -> begin
    * @param input -> end
    * @param input -> threshold
    * @param output -> dataIn_[i]->isRemoved
    * @return void
    */
    void GenerateSegPoints(const int& begin,
                           const int& end,
                           const double& threshold);
    /**
    * @brief Get params
    * @param input -> lineParams_
    * @param output -> lineParams_
    * @return std::vector<LineParam>
    */
    std::vector<LineParam> GetParams();

    /**
    * @brief Generate result
    * @param input -> dataIn_
    * @param output -> lineParams_
    * @return void
    */
    void Output();

    /**
    * @brief According segmented points to generate poly lines
    * @param input -> points
    * @param input -> isInverse
    * @param output -> param
    * @return void
    */
    static void Polyfit(const std::vector<Eigen::Vector3d>& points,
                        bool& isInverse,
                        LineParam& param);

private:
    /**
    * @brief Accroding K to judge line is inverse or not
    * @param input -> start
    * @param input -> end
    * @param output -> bool
    * @return true -> inverse, false -> not inverse
    */
    bool IsInverse(const Eigen::Vector3d& start,
                   const Eigen::Vector3d& end);

    /**
    * @brief Calculate distance between point to line
    * @param input -> p1
    * @param input -> p2
    * @param input -> p3
    * @param output -> dist
    * @return double
    */
    static double CalcPtLineDis(const Point::Ptr& p1,
                                const Point::Ptr& p2,
                                const Point::Ptr& p3);

    /**
    * @brief Get max distance between point to line
    * @param input -> points
    * @param input -> beginId
    * @param input -> endId
    * @param output -> maxDist
    * @return double
    */
    static double GetMaxDist(std::vector<Point::Ptr>& points,
                             const int& beginId,
                             const int& endId);

    /**
    * @brief Get max distance's id
    * @param input -> points
    * @param input -> beginId
    * @param input -> endId
    * @param output -> id
    * @return int
    */
    static int GetMaxDistId(std::vector<Point::Ptr>& points,
                            const int& beginId,
                            const int& endId);

private:
    std::vector<LineParam> lineParams_;
    std::vector<Point::Ptr> allSegPoints_;
    std::vector<Point::Ptr> dataIn_;
};
}
}


#endif //PARKING_SE_FUSION_POLY_LINE_FITTING_H



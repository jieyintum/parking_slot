#ifndef TRACK_LINE_HPP_INCLUDED
#define TRACK_LINE_HPP_INCLUDED

#include "line_process/line_common_function.hpp"

namespace Fusion
{

class CTrackLine : public CLineModel
{
public:

enum OpenLineStatus {
    unknown = 0,
    occupiedByWheelStop = 1, //highest occupied prioriy
    occupiedByNeighborSlot = 2, //middle occupied prioriy
    occupiedByFs = 3,  //lowest occupied priority
    oppositeOccupiedByNeighborSlot = 4
};


    using Ptr = std::shared_ptr<CTrackLine>;
    
    CTrackLine() = default;
    ~CTrackLine() = default;

    CTrackLine(const Eigen::Vector3d startPoint, const Eigen::Vector3d endPoint):matchedLine(nullptr)
    {
        initWithTwoWorldPoints(startPoint, endPoint);
    }


    inline void getVerticalLine(const CTrackLine::Ptr verticalLine)
    {
        Eigen::Vector3d veticalUV = Eigen::Vector3d(-getDefaultUnitVec()[1], getDefaultUnitVec()[0], 0.0f);
        Eigen::Vector3d verticalEnd = getStartPoint() + veticalUV;

        verticalLine->initWithTwoWorldPoints(getStartPoint(), verticalEnd);
    }



    CTrackLine::Ptr matchedLine;
    OpenLineStatus couldBeingAsOpenLine = OpenLineStatus::unknown;

    float lastFrameLength = 0.0f;
};

}

#endif /* TRACK_LINE_HPP_INCLUDED */

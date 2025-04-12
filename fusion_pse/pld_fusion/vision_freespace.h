#ifndef FUSION_PSE_VISIONFREESPACE_H_
#define FUSION_PSE_VISIONFREESPACE_H_

#include "inner_msgs/avpe.h"
#include "msg_buffer/odom_buffer.h"

namespace Fusion
{
    namespace PSE
    {
        struct VisionFreespace
        {
            using Ptr = std::shared_ptr<VisionFreespace>;
            VisionFreespace(const AvpeFreespace::Ptr &vsFs, const TimeStamp &t) : timestamp(t), inputFs(vsFs)
            {
                const auto odomPtr = OdomBuffer::GetInstance().GetInterpolation(timestamp);
                if (odomPtr != nullptr)
                {
                    for (const auto &pt : inputFs->points)
                    {
                        Eigen::Vector2d odomPoint = odomPtr->orientation.matrix().block<2U, 2U>(0, 0) * pt.point + odomPtr->translation.segment<2>(0);
                        worldPoints.push_back(std::move(odomPoint));
                    }
                }
            }
            TimeStamp timestamp;
            AvpeFreespace::Ptr inputFs;
            std::vector<Eigen::Vector2d> worldPoints;
        };

        struct visionAvpeObject
        {
            using Ptr = std::shared_ptr<visionAvpeObject>;

            visionAvpeObject(const AvpeObjects &vsObject, const Odometry::Ptr &odomPtr)
            {
                for (const auto &pt : vsObject.rotatedBbox)
                {
                    // std::cout<<"corner vehicle: "<<pt.point[0]<<", "<<pt.point[1]<<"\n";
                    Eigen::Vector2d odomPoint =
                        odomPtr->orientation.matrix().block<2U, 2U>(0, 0) * pt.point + odomPtr->translation.segment<2>(0);
                    worldPoints.push_back(std::move(odomPoint));
                }

                for (size_t i = 0; i < worldPoints.size(); i++)
                {
                    centralPosition[0] += worldPoints[i][0];
                    centralPosition[1] += worldPoints[i][1];
                }

                centralPosition /= (float)worldPoints.size();
                centralPosition[2] = 0.0f;
                // std::cout<<"central world coord: "<<centralPosition[0]<<", "<<centralPosition[1]<<"\n";
            }

            std::vector<Eigen::Vector2d> worldPoints;
            Eigen::Vector3d centralPosition = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
        };

        struct visionObjVector
        {
            using Ptr = std::shared_ptr<visionObjVector>;
            visionObjVector(const Avpe::Ptr &avpe_, const TimeStamp &t) : timestamp(t)
            {
                vsObjVector.clear();
                const Odometry::Ptr odomPtr = OdomBuffer::GetInstance().GetInterpolation(timestamp);
                if (odomPtr != nullptr)
                {
                    for (const auto &obj : avpe_->objects)
                    {
                        if (obj.type == 17) // floor lock = 17
                        {
                            auto visionAvpeObjectPtr = std::make_shared<visionAvpeObject>(obj, odomPtr);
                            vsObjVector.push_back(visionAvpeObjectPtr);
                        }
                    }
                }
            }

            TimeStamp timestamp;
            std::vector<visionAvpeObject::Ptr> vsObjVector;
        };

    }
}

#endif
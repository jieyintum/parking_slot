#include "arrow_fusion/arrow_fusion.h"
#include "utils/geom_utils.h"
#include <time.h>

namespace Fusion
{
    namespace PSE
    {

#define ARROW_TYPE_COUNT 14U

        void ArrowFusion::CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg)
        {
            if ((0 == (int)msg->icc50mssiggwpdu01.idrvrdooropensts_chcanfd) && (0 == (int)msg->icc50mssiggwpdu01.ifrtpsngdooropensts_chcanfd) && (0 == (int)msg->icc50mssiggwpdu01.ildspcopensts) && (0 == (int)msg->icc50mssiggwpdu01.irldooropensts_chcanfd) && (0 == (int)msg->icc50mssiggwpdu01.irrdooropensts_chcanfd))
            {
                isDoorStatusOk_ = true;
            }
            else
            {
                isDoorStatusOk_ = false;
            }
        }
        void ArrowFusion::DetectParkingStatus(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg)
        {
            isParking_ = (msg->hmi_pnc_info.parkin_slot_id != 0) && (((msg->sm_pnc_info.apa_interior_status >= 9) && (msg->sm_pnc_info.apa_interior_status <= 12)) || ((msg->sm_pnc_info.rpastsin >= 5) && (msg->sm_pnc_info.rpastsin <= 12)));
            // std::cout << "Parking status is :" << isParking_ << std::endl;
        }

        void ArrowFusion::InitLocalMap(AvpeArrowFrame &frame)
        {
            double vehicleX = frame.poseOb.translation.x();
            double vehicleY = frame.poseOb.translation.y();
            double minX = vehicleX - this->mapRange_ * 0.5;
            double minY = vehicleY - this->mapRange_ * 0.5;
            this->gMapPtr_->SetLowerBound(minX, minY);
        }

        void ArrowFusion::Publish(AvpeArrowFrame &frame)
        {
        }

        bool ArrowFusion::MakeFrame(AvpeArrowFrame &frame)
        {
            return manager_.MakeFrame(frame);
            ;
        }

        bool ArrowFusion::CreateGridMap(AvpeGridMap::Ptr &gMapPtr)
        {
            constexpr double percesion = 0.05; // m
            double maxIndexX = -1e10;
            double maxIndexY = -1e10;
            double minIndexX = +1e10;
            double minIndexY = +1e10;
            uint32_t pointCnt = 0U;
            for (const auto &fusionArrowPtr : multiFrameArrows_)
            {
                pointCnt += fusionArrowPtr->worldPoints.size();
                for (const auto &point : fusionArrowPtr->worldPoints)
                {
                    maxIndexX = std::max(maxIndexX, point.x());
                    maxIndexY = std::max(maxIndexY, point.y());
                    minIndexX = std::min(minIndexX, point.x());
                    minIndexY = std::min(minIndexY, point.y());
                }
            }
            double deltaX = abs(maxIndexX - minIndexX);
            double deltaY = abs(maxIndexY - minIndexY);
            if (pointCnt > 0U)
            {
                gMapPtr = std::make_shared<AvpeGridMap>(minIndexX, minIndexY, deltaX, deltaY, percesion);
                return true;
            }
            return false;
        }

        int ArrowFusion::GetSubType(std::vector<int> types)
        {
            std::array<int, ARROW_TYPE_COUNT> typeSlot{};
            for (size_t i = 0; i < types.size(); ++i)
            {
                int arrowSubType = types[i];
                if (arrowSubType >= (int)ARROW_TYPE_COUNT)
                {
                    continue;
                }
                typeSlot[arrowSubType] += 1;
            }
            int maxType = 1;
            uint32_t maxCount = 0;
            for (size_t curType = 0; curType < ARROW_TYPE_COUNT; ++curType)
            {
                if (typeSlot[curType] > maxCount)
                {
                    maxType = curType;
                    maxCount = typeSlot[curType];
                }
            }
            return maxType;
        }

        void ArrowFusion::ClusterArrowPoint(AvpeArrowFrame &frame)
        {
            if (multiFrameArrows_.size() < 5U || frame.detectArrow_->points.empty())
            {
                return;
            }
            // create map
            InitLocalMap(frame);

            size_t pointNum = 0;
            for (const auto &fusionArrowPtr : multiFrameArrows_)
            {
                for (size_t i = 0; i < fusionArrowPtr->worldPoints.size(); ++i)
                {
                    gMapPtr_->InsertElem(fusionArrowPtr->worldPoints[i], fusionArrowPtr->pixleTypes[i]);
                }
                pointNum += fusionArrowPtr->worldPoints.size();
            }

            std::cout << "Current Process Arrow Point Count is " << pointNum << std::endl;

            std::vector<std::vector<Eigen::Vector3d>> counters{};
            std::vector<std::vector<int>> types{};
            gMapPtr_->FindCounters(counters, types);

            for (size_t i = 0; i < counters.size(); ++i)
            {
                auto cnt = counters[i];
                FusionArrow::Ptr fusionArrowPtr = std::make_shared<FusionArrow>();
                fusionArrowPtr->SetWorldEdgePoints(cnt);
                fusionArrowPtr->SetMinAreaRect();
                if (fusionArrowPtr->CalcBoxArea() < 0.25)
                {
                    continue;
                }
                fusionArrowPtr->subType = GetSubType(types[i]);
                frame.clusterArrows_.push_back(fusionArrowPtr);
            }
            gMapPtr_->Reset();
        }

        void ArrowFusion::StoreMultiFrameArrow(AvpeArrowFrame &frame)
        {
            constexpr double minGapDist = 0.05;
            Odometry curOdom = frame.poseOb;
            double gapDist = (curOdom.translation - preOdom_.translation).norm();
            if (!frame.detectArrow_->points.empty() && gapDist >= minGapDist)
            {
                multiFrameArrows_.Push(frame.detectArrow_);
                preOdom_ = frame.poseOb;
            }
        }

        bool ArrowFusion::IsMatched(const FusionArrow::Ptr &trackArrowPtr, const FusionArrow::Ptr &fusionArrowPtr)
        {
            if (fusionArrowPtr->isVisited)
            {
                return false;
            }
            bool isIntersect = Geometry::IsPolygonIntersect(trackArrowPtr->worldEdgePoints, fusionArrowPtr->worldEdgePoints);
            double centerOffset = (trackArrowPtr->GetWorldCenterPoint() - fusionArrowPtr->GetWorldCenterPoint()).norm();
            bool isArrowRange = centerOffset < 2.0;
            return isIntersect || isArrowRange;
        }

        void ArrowFusion::MatchArrows(AvpeArrowFrame &frame)
        {
            if (frame.clusterArrows_.empty())
            {
                return;
            }

            if (trackArrows_.empty())
            {
                trackArrows_.assign(frame.clusterArrows_.begin(), frame.clusterArrows_.end());
                for (const auto &fusionArrowPtr : trackArrows_)
                {
                    fusionArrowPtr->id = (++this->arrowId_);
                    fusionArrowPtr->isVisited = true;
                }
            }
            else
            {
                for (const auto &trackArrowPtr : trackArrows_)
                {
                    trackArrowPtr->isActive = false;
                    for (const auto &fusionArrowPtr : frame.clusterArrows_)
                    {
                        bool matchResult = IsMatched(trackArrowPtr, fusionArrowPtr);
                        if (matchResult)
                        {
                            if (!trackArrowPtr->IsConfirm())
                            {
                                trackArrowPtr->relativeArrows.push_back(fusionArrowPtr);
                            }
                            ++trackArrowPtr->avaliableCount;
                            trackArrowPtr->isActive = true;
                            fusionArrowPtr->isVisited = true;
                            trackArrowPtr->AddArrowType(fusionArrowPtr->subType);
                        }
                    }
                }
            }
        }

        void ArrowFusion::UpdateTrackArrows()
        {
            for (const auto &trackArrowPtr : trackArrows_)
            {
                if (!trackArrowPtr->isActive)
                {
                    --trackArrowPtr->avaliableCount;
                }
                trackArrowPtr->isActive = false;
            }
        }

        void ArrowFusion::AddNewFoundArrows(AvpeArrowFrame &frame)
        {
            for (const auto &fusionArrowPtr : frame.clusterArrows_)
            {
                if (!fusionArrowPtr->isVisited)
                {
                    fusionArrowPtr->id = (++this->arrowId_);
                    trackArrows_.push_back(fusionArrowPtr);
                }
            }
        }

        void ArrowFusion::CalcOneDimNormProbParam(const std::vector<double> &xs, double &mean, double &stdVar)
        {
            uint32_t count = xs.size();
            double meanSum = 0.0;
            for (size_t i = 0; i < count; ++i)
            {
                meanSum += xs[i];
            }
            mean = meanSum / static_cast<double>(count);
            double varSum = 0.0;
            for (size_t i = 0U; i < count; ++i)
            {
                varSum += std::pow((xs[i] - mean), 2U);
            }
            stdVar = std::sqrt(varSum / static_cast<double>(count));
        }

        void ArrowFusion::FusionByNormProbDistibute(const FusionArrow::Ptr &fusionArrowPtr, std::vector<FusionArrow::Ptr> &fusionArrows)
        {
            std::vector<double> angles{};
            std::vector<double> widths{};
            for (size_t i = 0U; i < fusionArrowPtr->relativeArrows.size(); ++i)
            {
                angles.push_back(fusionArrowPtr->relativeArrows[i]->GetWorldArrowDirection());
                widths.push_back(fusionArrowPtr->relativeArrows[i]->GetWorldRectWidth());
            }

            double angleMean = 0.0;
            double widthMean = 0.0;
            double angleStdVar = 0.0;
            double widthStdVar = 0.0;
            CalcOneDimNormProbParam(angles, angleMean, angleStdVar);
            CalcOneDimNormProbParam(widths, widthMean, widthStdVar);
            // 1-sigma
            double angleLowerBound = angleMean - 1. * angleStdVar;
            double angleUpperBound = angleMean + 1. * angleStdVar;
            double widthLowerBound = widthMean - 1. * widthStdVar;
            double widthUpperBound = widthMean + 1. * widthStdVar;

            bool angleStdVarIsZero = (angleStdVar < 1e-20);
            // 计算宽度和角度的正态联合概率分布 过滤掉后10%的长尾噪声
            for (const auto &relArrowPtr : fusionArrowPtr->relativeArrows)
            {
                double angle = relArrowPtr->GetWorldArrowDirection();
                double width = relArrowPtr->GetWorldRectWidth();
                bool isNormAngle = (angleLowerBound < angle && angle < angleUpperBound);
                bool isNormWidth = (widthLowerBound < width && width < widthUpperBound);
                if (angleStdVarIsZero && isNormWidth)
                {
                    fusionArrows.push_back(relArrowPtr);
                }
                else if ((!angleStdVarIsZero) && isNormAngle && isNormWidth)
                {
                    fusionArrows.push_back(relArrowPtr);
                }
            }
        }

        void ArrowFusion::FusionMultiObjectArrow(FusionArrow::Ptr &fusionArrowPtr)
        {

            struct
            {
                bool operator()(FusionArrow::Ptr fusionArrowPtr1, FusionArrow::Ptr fusionArrowPtr2) const
                {
                    double error1 = abs(fusionArrowPtr1->GetWorldRectLength() - 3.0);
                    double error2 = abs(fusionArrowPtr2->GetWorldRectLength() - 3.0);
                    return error1 < error2;
                }
            } lengthErrorSorter;
            // std::vector<FusionArrow::Ptr> fusionRelArrows{};
            std::vector<FusionArrow::Ptr> fusionRelArrows = {fusionArrowPtr->relativeArrows.begin(),
                                                             fusionArrowPtr->relativeArrows.end()};
            // FusionByNormProbDistibute(fusionArrowPtr, fusionRelArrows);
            if (!fusionRelArrows.empty())
            {
                Eigen::Vector3d avgCenter{0.0, 0.0, 0.0};
                for (const auto &relArrow : fusionRelArrows)
                {
                    avgCenter += relArrow->GetWorldCenterPoint();
                }
                avgCenter /= fusionRelArrows.size();
                fusionArrowPtr->SetWorldCenterPoint(avgCenter);
            }

            if (!fusionRelArrows.empty())
            {
                std::cout << fusionArrowPtr->id << " Update! " << std::endl;
                std::sort(fusionRelArrows.begin(), fusionRelArrows.end(), lengthErrorSorter);
                FusionArrow::Ptr trackedFusionArrowPtr = fusionRelArrows.front();
                fusionArrowPtr->worldEdgePoints.assign(trackedFusionArrowPtr->worldEdgePoints.begin(),
                                                       trackedFusionArrowPtr->worldEdgePoints.end());
                fusionArrowPtr->worldPoints.assign(trackedFusionArrowPtr->worldPoints.begin(),
                                                   trackedFusionArrowPtr->worldPoints.end());
                fusionArrowPtr->SetMinAreaRect();
            }
        }

        void ArrowFusion::ClearUntrackObject(AvpeArrowFrame &frame)
        {
            constexpr double maxDist = 15.;
            for (auto trackIter = trackArrows_.begin(); trackIter != trackArrows_.end();)
            {
                Eigen::Vector3d arrowCoord = (*trackIter)->GetWorldCenterPoint();
                arrowCoord.z() = 0.0;
                Eigen::Vector3d odom2d = {frame.poseOb.translation.x(), frame.poseOb.translation.y(), 0.0};
                double curFrameToArrowDist = (odom2d - arrowCoord).norm();
                if ((*trackIter)->avaliableCount == 0U || curFrameToArrowDist > maxDist)
                {
                    std::cout << odom2d.transpose() << "  |  " << arrowCoord.transpose() << std::endl;
                    std::cout << "Clear track arrow object, lifecout is " << (*trackIter)->avaliableCount << " and distance is " << curFrameToArrowDist << std::endl;
                    (*trackIter)->relativeArrows.clear();
                    trackIter = trackArrows_.erase(trackIter);
                }
                else
                {
                    ++trackIter;
                }
            }
        }

        void ArrowFusion::FusionTrackedObjects()
        {
            for (auto &trackArrowPtr : trackArrows_)
            {
                if (trackArrowPtr->IsOutputPub())
                {
                    FusionMultiObjectArrow(trackArrowPtr);
                }
            }
        }

        void ArrowFusion::Detect(AvpeArrowFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }

            if (frame.detectArrow_ == nullptr)
            {
                return;
            }
            clock_t start = clock();
            StoreMultiFrameArrow(frame);
            ClusterArrowPoint(frame);
            clock_t end = clock();
            std::cout << "Detect time is " << (double(end - start) / CLOCKS_PER_SEC) << " s" << std::endl;
        }

        void ArrowFusion::Track(AvpeArrowFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            clock_t start = clock();
            MatchArrows(frame);
            UpdateTrackArrows();
            AddNewFoundArrows(frame);
            ClearUntrackObject(frame);
            std::cout << "Cluster arrow count is :" << frame.clusterArrows_.size() << std::endl;
            FusionTrackedObjects();
            std::cout << "Track arrow count is :" << trackArrows_.size() << std::endl;
            GenArrowDir(frame);
            GenVirtualBox();
            clock_t end = clock();
            std::cout << "Track time is " << (double(end - start) / CLOCKS_PER_SEC) << " s" << std::endl;
        }

        void ArrowFusion::Output(AvpeArrowFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            constexpr double minOuputLength = 0.5; // 50cm
            for (auto &trackArrowPtr : trackArrows_)
            {
                if (trackArrowPtr->IsOutputPub() && !trackArrowPtr->direction.empty())
                {
                    trackArrowPtr->TransferToBase(frame.poseOb);
                    frame.outputArrows_.push_back(trackArrowPtr);
                }
            }
        }

        Eigen::Vector3d ArrowFusion::GenRightEndPoint(const double width, const Eigen::Vector3d &left,
                                                      const Eigen::Vector3d &right, const Eigen::Vector3d &basePoint)
        {
            const double sinAnlge = (right.y() - left.y()) / (right - left).norm();
            const double cosAnlge = (right.x() - left.x()) / (right - left).norm();
            const double sinVangle = cosAnlge;
            const double cosVangle = sinAnlge;

            Eigen::Vector3d endPoint(0.0, 0.0, 0.0);
            endPoint.x() = basePoint.x() + cosVangle * width;
            endPoint.y() = basePoint.y() - sinVangle * width;
            return endPoint;
        }

        Eigen::Vector3d ArrowFusion::GenLeftEndPoint(const double width, const Eigen::Vector3d &left,
                                                     const Eigen::Vector3d &right, const Eigen::Vector3d &basePoint)
        {
            double sinAnlge = (right.y() - left.y()) / (right - left).norm();
            double cosAnlge = (right.x() - left.x()) / (right - left).norm();
            double sinVangle = cosAnlge;
            double cosVangle = sinAnlge;

            Eigen::Vector3d endPoint;
            endPoint[0] = basePoint[0] - cosVangle * width;
            endPoint[1] = basePoint[1] + sinVangle * width;
            return endPoint;
        }

        void ArrowFusion::GenComplexArrowDir(FusionArrow::Ptr &trackArrow)
        {
            std::cout << "This is a left or right turn arrow!" << std::endl;
            auto p0 = trackArrow->worldBoxPoints[0];
            auto p1 = trackArrow->worldBoxPoints[1];
            auto p2 = trackArrow->worldBoxPoints[2];
            auto p3 = trackArrow->worldBoxPoints[3];
            auto c01 = (p0 + p1) * 0.5;
            auto c12 = (p1 + p2) * 0.5;
            auto c23 = (p2 + p3) * 0.5;
            auto c30 = (p3 + p0) * 0.5;
            auto c00 = (p0 + p1 + p2 + p3) * 0.25;
            double len1 = (p0 - p1).norm();
            double len2 = (p1 - p2).norm();
            std::array<std::vector<Eigen::Vector3d>, 4U> boxList{};

            if (len1 > len2)
            {
                boxList[0] = {c30, p0, c01, c00};
                boxList[1] = {c00, c01, p1, c12};
                boxList[2] = {c23, c00, c12, p2};
                boxList[3] = {p3, c30, c00, c23};
            }
            else
            {
                boxList[0] = {c01, p1, c12, c00};
                boxList[1] = {c00, c12, p2, c23};
                boxList[2] = {c30, c00, c23, p3};
                boxList[3] = {p0, c01, c00, c30};
            }

            std::vector<Eigen::Vector3d> minBox{};
            double minArea = 1e10;
            for (uint8_t i = 0; i < 4U; ++i)
            {
                double area = Geometry::DifferenceArea(trackArrow->worldEdgePoints, boxList[i]);
                std::cout << "Corner: " << i << " area is " << area << std::endl;
                if (area < minArea)
                {
                    minArea = area;
                    minBox = boxList[i];
                }
            }

            trackArrow->tailInter = minBox;

            // 斜边
            double arrisEdgeLen = (minBox[0] - minBox[2]).norm();
            double maxLen = 0.;
            Eigen::Vector3d dirPoint;
            // 找到次大边
            for (uint8_t i = 0; i < 4U; ++i)
            {
                double curSegLen = (minBox[i] - c00).norm();
                if (curSegLen < (arrisEdgeLen - 1e-2) && curSegLen > maxLen)
                {
                    maxLen = curSegLen;
                    dirPoint = minBox[i];
                }
            }
            trackArrow->direction = {c00, dirPoint};
        }

        void ArrowFusion::GenSimpleArrowDir(FusionArrow::Ptr &trackArrow)
        {
            std::cout << "This is a normal arrow!" << std::endl;
            auto p0 = trackArrow->worldBoxPoints[0];
            auto p1 = trackArrow->worldBoxPoints[1];
            auto p2 = trackArrow->worldBoxPoints[2];
            auto p3 = trackArrow->worldBoxPoints[3];
            auto c01 = (p0 + p1) * 0.5;
            auto c12 = (p1 + p2) * 0.5;
            auto c23 = (p2 + p3) * 0.5;
            auto c30 = (p3 + p0) * 0.5;
            auto c00 = (p0 + p1 + p2 + p3) * 0.25;
            double len1 = (p0 - p1).norm();
            double len2 = (p1 - p2).norm();

            std::vector<Eigen::Vector3d> headBox{};
            std::vector<Eigen::Vector3d> tailBox{};
            if (len1 > len2)
            {
                headBox = {p0, c01, c23, p3};
                tailBox = {c01, p1, p2, c23};
            }
            else
            {
                headBox = {p1, c12, c30, p0};
                tailBox = {c12, p2, p3, c30};
            }

            trackArrow->headBox = headBox;
            trackArrow->tailBox = tailBox;

            double tailArea = Geometry::DifferenceArea(trackArrow->worldEdgePoints, tailBox);
            double headArea = Geometry::DifferenceArea(trackArrow->worldEdgePoints, headBox);

            std::cout << trackArrow->id << "Current arrow head area is " << headArea << " tail area is " << tailArea << std::endl;
            if (headArea > tailArea)
            {
                auto dirPoint = (headBox[0] + headBox[1] + headBox[2] + headBox[3]) * 0.25;
                trackArrow->direction = {c00, dirPoint};
            }
            else
            {
                auto dirPoint = (tailBox[0] + tailBox[1] + tailBox[2] + tailBox[3]) * 0.25;
                trackArrow->direction = {c00, dirPoint};
            }
        }

        void ArrowFusion::GenArrowDir(AvpeArrowFrame &frame)
        {
            for (auto &trackArrow : trackArrows_)
            {
                auto arrowType = trackArrow->GetArrowTypeByVote();
                bool isComplexArrow = trackArrow->GetWorldRectWidth() > 0.65;
                isComplexArrow &= (arrowType == StaticElement::GROUND_ARROW_STARGIHT_LEFT ||
                                   arrowType == StaticElement::GROUND_ARROW_STARGIHT_RIGHT);
                if (isComplexArrow)
                {
                    GenComplexArrowDir(trackArrow);
                }
                else
                {
                    GenSimpleArrowDir(trackArrow);
                }
                bool isLeftOrRightTurnArrow = (arrowType == StaticElement::GROUND_ARROW_LEFT ||
                                               arrowType == StaticElement::GROUND_ARROW_RIGHT);
                if (isLeftOrRightTurnArrow)
                {
                    CorrectArrowDir(frame, trackArrow);
                }
            }
        }

        void ArrowFusion::GenVirtualBox()
        {
            for (const auto &trackArrow : trackArrows_)
                if (trackArrow->direction.size() == 2U)
                {
                    auto dirHead = trackArrow->direction[0];
                    auto dirTail = trackArrow->direction[1];
                    Eigen::Vector3d p0 = GenRightEndPoint(0.01, dirHead, dirTail, dirTail);
                    Eigen::Vector3d p1 = GenLeftEndPoint(0.01, dirHead, dirTail, dirTail);
                    std::swap(dirHead, dirTail);
                    Eigen::Vector3d p2 = GenRightEndPoint(0.1, dirHead, dirTail, dirTail);
                    Eigen::Vector3d p3 = GenLeftEndPoint(0.1, dirHead, dirTail, dirTail);
                    trackArrow->worldOutputBoxPoints = {p0, p1, p2, p3};
                }
        }

        void ArrowFusion::CorrectArrowDir(AvpeArrowFrame &frame, FusionArrow::Ptr &trackArrow)
        {
            /*
                1.读取当前与当前行驶同向的车道线集合 relLaneLines
                2.找到箭头方向左转，右转，左右转集合 relArrows
                3.遍历relArrows，并找到与之夹角小于20°的 relLaneLines
                4.矫正箭头方向
            */

            std::cout << "correct arrow direction " << std::endl;
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> relLaneLines{};
            for (const auto &line : laneLines_)
            {
                if (line.classification != StaticElement::GROUND_LANE_LINE)
                {
                    continue;
                }

                if (line.number_of_vertices != 2)
                {
                    continue;
                }
                Eigen::Vector3d head = {line.vertices_list[0].x, line.vertices_list[0].y, 0.0};
                Eigen::Vector3d tail = {line.vertices_list[1].x, line.vertices_list[1].y, 0.0};
                Eigen::Vector3d laneLineHeading = tail - head;

                double yaw = Quaternion2Eular(frame.poseOb.orientation).z();
                double headX = frame.poseOb.translation.x();
                double headY = frame.poseOb.translation.y();
                double tailX = headX + cos(yaw);
                double tailY = headY + sin(yaw);
                Eigen::Vector3d vehicleHeading = {tailX - headX, tailY - headY, 0.0};
                bool isSameDir = vehicleHeading.dot(laneLineHeading) > 0.0;

                double cosTheta = vehicleHeading.dot(laneLineHeading) / (vehicleHeading.norm() * laneLineHeading.norm());
                if (abs(cosTheta) > 0.9)
                {
                    if (cosTheta < 0.0)
                    {
                        relLaneLines.push_back({tail, head});
                    }
                    else
                    {
                        relLaneLines.push_back({head, tail});
                    }
                }
            }
            std::cout << "Rel lane size: " << relLaneLines.size() << std::endl;
            double lMaxDist = 0.0;
            double rMaxDist = 0.0;
            std::pair<Eigen::Vector3d, Eigen::Vector3d> leftestLaneLine{};
            std::pair<Eigen::Vector3d, Eigen::Vector3d> rightestLaneLine{};
            for (const auto &laneLine : relLaneLines)
            {
                double dist = Geometry::PointToSegmentDist(laneLine.first, laneLine.second, frame.poseOb.translation);
                if (!Geometry::ToLeft(laneLine.first, laneLine.second, frame.poseOb.translation))
                {
                    if (dist < 6.0 && dist > lMaxDist)
                    {
                        lMaxDist = dist;
                        leftestLaneLine = laneLine;
                    }
                }
                else
                {
                    if (dist < 6.0 && dist > rMaxDist)
                    {
                        rMaxDist = dist;
                        rightestLaneLine = laneLine;
                    }
                }
            }
            if (lMaxDist > 0.0 && rMaxDist > 0.0)
            {
                std::cout << "Leftest lane " << leftestLaneLine.first.transpose() << std::endl;
                std::cout << "Rightest lane " << rightestLaneLine.first.transpose() << std::endl;
                auto arrowCenterPoint = trackArrow->GetWorldCenterPoint();
                std::vector<Eigen::Vector3d> box = {leftestLaneLine.first, leftestLaneLine.second, rightestLaneLine.second, rightestLaneLine.first};
                if (Geometry::PointInPolygon(arrowCenterPoint, box))
                {
                    auto laneLineDir = leftestLaneLine.first - leftestLaneLine.second;
                    auto arrowDir = trackArrow->direction.front() - trackArrow->direction.back();
                    bool isSameDir = arrowDir.dot(laneLineDir) / (arrowDir.norm() * laneLineDir.norm()) > 0.0;

                    double sinTheta = (leftestLaneLine.second.y() - leftestLaneLine.first.y()) / (leftestLaneLine.second - leftestLaneLine.first).norm();
                    double cosTheta = (leftestLaneLine.second.x() - leftestLaneLine.first.x()) / (leftestLaneLine.second - leftestLaneLine.first).norm();
                    double tanTheta = (leftestLaneLine.second.y() - leftestLaneLine.first.y()) / (leftestLaneLine.second.x() - leftestLaneLine.first.x());

                    if (isSameDir)
                    {
                        Eigen::Vector3d dirTail = {arrowCenterPoint.x() + cosTheta, arrowCenterPoint.y() + sinTheta, 0.0};
                        trackArrow->direction = {arrowCenterPoint, dirTail};
                    }
                    else
                    {
                        Eigen::Vector3d dirTail = {arrowCenterPoint.x() - cosTheta, arrowCenterPoint.y() - sinTheta, 0.0};
                        trackArrow->direction = {arrowCenterPoint, dirTail};
                    }
                }
            }
        }

        void ArrowFusion::SetCurFrameLanLine(std::vector<StaticElement> &elems)
        {
            laneLines_.clear();
            for (const auto &line : elems)
            {
                if (line.classification != StaticElement::GROUND_LANE_LINE)
                {
                    continue;
                }
                if (line.number_of_vertices != 2)
                {
                    continue;
                }
                laneLines_.push_back(line);
            }

            std::cout << "Getted " << laneLines_.size() << " lane line " << std::endl;
        }

        void ArrowFusion::Debug(AvpeArrowFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
#if DEBUG_MODE
            ShowPerceptionArrow(frame.detectArrow_);
            // ShowOdomArrow(frame.clusterArrows_);
            ShowOdomArrow({trackArrows_.begin(), trackArrows_.end()});
#endif
        }

#if DEBUG_MODE

        void ArrowFusion::ShowOdomArrow(const std::vector<FusionArrow::Ptr> &arrows)
        {
            visualization_msgs::msg::MarkerArray markers;

            if (!arrows.empty())
            {
                for (size_t i = 0U; i < arrows.size(); ++i)
                {
                    // show counter.
                    PushSingleOdomArrowMarker(arrows[i], i, markers);
                    // show box.
                    PushSingleOdomArrowBoxMarker(arrows[i], i + arrows.size(), markers);
                    // show poly
                    // PushSingleOdomArrowPolyMarker(arrows[i], i + 2 * arrows.size(), markers);
                }
            }
            fusionOdomArrowMarkerPub_->publish(markers);
        }

        void ArrowFusion::PushSingleOdomArrowPolyMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                                        visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "fusion";
            marker.id = id;
            marker.type = 4;
            marker.action = 0;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            for (uint32_t i = 0U; i < fusionArrow->worldEdgePoints.size(); ++i)
            {
                geometry_msgs::msg::Point point;
                point.x = fusionArrow->worldEdgePoints[i].x();
                point.y = fusionArrow->worldEdgePoints[i].y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }
            markers.markers.emplace_back(marker);
        }

        void ArrowFusion::PushSingleOdomArrowBoxMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                                       visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "fusion";
            marker.id = id;
            marker.type = 4;
            marker.action = 0;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            for (uint32_t i = 0U; i < fusionArrow->tailInter.size(); ++i)
            {
                geometry_msgs::msg::Point point;
                point.x = fusionArrow->tailInter[i].x();
                point.y = fusionArrow->tailInter[i].y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }
            markers.markers.emplace_back(marker);
        }

        void ArrowFusion::PushSingleOdomArrowMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                                    visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "fusion";
            marker.id = id;
            marker.type = 4;
            marker.action = 0;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 0;
            color.g = 0;
            color.b = 1;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            for (uint32_t i = 0U; i < fusionArrow->worldEdgePoints.size(); ++i)
            {
                geometry_msgs::msg::Point point;
                point.x = fusionArrow->worldEdgePoints[i].x();
                point.y = fusionArrow->worldEdgePoints[i].y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }

            markers.markers.emplace_back(marker);
        }

        void ArrowFusion::ShowPerceptionArrow(const FusionArrow::Ptr &arrows)
        {
            visualization_msgs::msg::MarkerArray markers;
            PushSinglePerceptionArrowMarker(arrows, 0, markers);
            fusionPerceptionArrowMarkerPub_->publish(markers);
        }

        void ArrowFusion::PushSinglePerceptionArrowMarker(const FusionArrow::Ptr &fusionArrow, const uint16_t id,
                                                          visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "fusion";
            marker.id = id;
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1;
            marker.color = color;
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            for (uint32_t i = 0U; i < fusionArrow->worldPoints.size(); ++i)
            {
                geometry_msgs::msg::Point point;
                point.x = fusionArrow->worldPoints[i].x();
                point.y = fusionArrow->worldPoints[i].y();
                point.z = 0.0;

                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }

            markers.markers.emplace_back(marker);
        }

#endif

    }
}

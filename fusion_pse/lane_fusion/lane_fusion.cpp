//
// Created by igs on 2022/12/5.
//

#include "lane_fusion.h"
#include "msg_buffer/odom_buffer.h"
#include "Astar/astar.h"

namespace Fusion
{
    namespace PSE
    {

        void LaneFusion::CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg)
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

        void LaneFusion::DetectParkingStatus(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg)
        {
            isParking_ = (msg->hmi_pnc_info.parkin_slot_id != 0) && (((msg->sm_pnc_info.apa_interior_status >= 9) && (msg->sm_pnc_info.apa_interior_status <= 12)) || ((msg->sm_pnc_info.rpastsin >= 5) && (msg->sm_pnc_info.rpastsin <= 12)));
            //    std::cout << "Parking status is :" << isParking_ << std::endl;
        }

        void LaneFusion::Publish(AvpeLaneFrame &frame)
        {
        }

        bool LaneFusion::MakeFrame(AvpeLaneFrame &frame)
        {
            return manager_.MakeFrame(frame);
        }

        void LaneFusion::Detect(AvpeLaneFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            //    auto startTime2 = chrono::system_clock::now();
            SaveOneFrame(frame);
            //    SaveMultiFrame(frame);
            //    chrono::duration<double> diff2 = chrono::system_clock::now() - startTime2;
            //    std::cout << "break line detect: " << diff2.count() << "s" << std::endl;
        }

        void LaneFusion::Track(AvpeLaneFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            //    auto startTime2 = chrono::system_clock::now();

            std::vector<Point::Ptr> noMatchPoints;
            Match(frame.lineFrame_->detectPointsWorld_, noMatchPoints);
            ///@rviz
            //    noMatchPoints_.clear();
            //    noMatchPoints_ = noMatchPoints;
            ///@rviz end
            FusionFilter();
            Clear();

            //    std::cout << "This frame has points: " << frame.lineFrame_->detectPointsWorld_.size() << std::endl;
            //    std::cout << "This frame has no match points: " << noMatchPoints.size() << std::endl;

            if (!noMatchPoints.empty())
            {
                std::vector<ClusterFrame::Ptr> allClusters;
                SetGridMap(noMatchPoints);
                Cluster(allClusters);
                frame.lineFrame_->noMatchPointsClusters_ = allClusters;
                FindStartAndEndPoint(frame);
                FindPath(frame);
                Generate(frame);
                Add();
            }
            //    chrono::duration<double> diff2 = chrono::system_clock::now() - startTime2;
            //    std::cout << "break line track: " << diff2.count() << "s" << std::endl;
        }

        void LaneFusion::Output(AvpeLaneFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            //    std::cout << "************OUTPUT************" << std::endl;
            if (trackLanes_.empty())
            {
                return;
            }
            ///@rviz
            //    matchPoints_.clear();
            ///@rviz end
            for (const auto &trackLane : trackLanes_)
            {
                ///@rviz
                //        if (!trackLane->relativePoints.empty())
                //            matchPoints_.insert(matchPoints_.end(), trackLane->relativePoints.begin(), trackLane->relativePoints.end());
                ///@rviz end
                if (trackLane->lifeCount >= 6U)
                {
                    //        if (trackLane->lifeCount >= 3U) {
                    frame.outputLanes_.emplace_back(trackLane);
                    ///@debug
                    std::cout << "Line id: " << trackLane->id << std::endl;
                    if (trackLane->param_.isInverse)
                    {
                        //                std::cout << "x = " << trackLane->param_.k << " y + " << trackLane->param_.b << std::endl;
                        std::cout << "start: " << trackLane->param_.GetY(trackLane->param_.start)
                                  << " " << trackLane->param_.start
                                  << " | end: " << trackLane->param_.GetY(trackLane->param_.end)
                                  << " " << trackLane->param_.end << std::endl;
                    }
                    else
                    {
                        //                std::cout << "y = " << trackLane->param_.k << " x + " << trackLane->param_.b << std::endl;
                        std::cout << "start: " << trackLane->param_.start
                                  << " " << trackLane->param_.GetY(trackLane->param_.start)
                                  << " | end: " << trackLane->param_.end
                                  << " " << trackLane->param_.GetY(trackLane->param_.end) << std::endl;
                    }
                    //            std::cout << "----------------" << std::endl;
                    ///@debug end
                }
            }
#if DEBUG_MODE
            std::cout << "Track Lane Object Size : " << trackLanes_.size() << std::endl;
            std::cout << "Publish Lane Object Size : " << frame.outputLanes_.size() << std::endl;
//    std::cout << "----------------" << std::endl;
#endif
        }

        void LaneFusion::Debug(AvpeLaneFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            ShowOdomLine(frame.outputLanes_);
            ShowdetectPoints(frame.lineFrame_);
            //    ShowNoMatchPoints(noMatchPoints_);
            //    ShowMatchPoints(matchPoints_);
            std::cout << "================================" << std::endl;
        }

        void LaneFusion::SaveOneFrame(AvpeLaneFrame &frame)
        {
            LineFrame::Ptr lineFrame = std::make_shared<LineFrame>();
            auto laneSeg = frame.lane_->segmentations[AvpeSegmentation::Type::LANE];
            lineFrame->SetPointsBase(laneSeg.region);
            lineFrame->TransToWorld(frame.poseOb);
            frame.lineFrame_ = lineFrame;
        }

        /*
        void LaneFusion::SaveMultiFrame(AvpeLaneFrame& frame)
        {
            const double minDist = 0.1;
            Odometry curOdom = frame.poseOb;
            const double dist = (curOdom.translation - preOdom_.translation).norm();
            if (!frame.lineFrame_->detectPointsBase_.empty() &&
                dist > minDist) {
                multiFrames_.Push(frame.lineFrame_);
                preOdom_ = curOdom;
            }

            if (multiFrames_.size() == 10) {
                multiFramesPoints_.clear();
                for (const auto& oneFrame : multiFrames_) {
                    for (const auto& point : oneFrame->detectPointsWorld_) {
                        multiFramesPoints_.emplace_back(point);
                    }
                }
            }
        }

        void LaneFusion::DownSampling(AvpeLaneFrame& frame)
        {
            if (multiFrames_.size() < 10) {
                return;
            }

            double minX = 1e7;
            double minY = 1e7;
            double maxX = -1e7;
            double maxY = -1e7;
            double precision = 0.2; //m

            for (const auto& point : multiFramesPoints_) {
                minX = std::min(minX, point->point.x());
                minY = std::min(minY, point->point.y());
                maxX = std::max(maxX, point->point.x());
                maxY = std::max(maxY, point->point.y());
            }
            const double deltaX = maxX - minX;
            const double deltaY = maxY - minY;
            if (deltaX > 0.0 && deltaY > 0.0) {
                gridMapDownSampling_ = std::make_shared<AvpeGridMap>(minX, minY, deltaX, deltaY, precision);
                for (const auto& point : multiFramesPoints_) {
                    gridMapDownSampling_->InsertElem(point->point);
                }
            }

            std::vector<ClusterFrame::Ptr> allClusters;
            if (gridMapDownSampling_ != nullptr) {
                const auto clusters = gridMapDownSampling_->Clustering(SearchMethod::FOUR_NEIGHBOUR);
                AvpeGrid::Ptr grid = std::make_shared<AvpeGrid>();
                for (const auto& cluster : clusters) {
                    ClusterFrame::Ptr oneCluster = std::make_shared<ClusterFrame>();
                    for (const auto& gridIndex : cluster) {
                        if (gridMapDownSampling_->GetElem(gridIndex, grid)) {
                            oneCluster->SetIndex(grid->GetCenterCoord(),
                                                 grid->GetGridIndex());
                        }
                    }
                    if (int(oneCluster->pointsWorld.size()) < oneClusterSizeThre_) {
                        continue;
                    }
                    allClusters.emplace_back(oneCluster);
                }
                frame.lineFrame_->detectClusters_ = allClusters;
            }
        }


        void LaneFusion::Match(const std::vector<ClusterFrame::Ptr>& clusters,
                               std::vector<Point::Ptr>& noMatchPoints)
        {
            std::cout << "*****match*****" << std::endl;
            if (clusters.empty()) {
                return;
            }

            for (const auto& track : trackLanes_) {
                track->relativePoints.clear();
            }

            for (const auto& cluster : clusters) {
                for (const auto& pt : cluster->pointsWorld) {

                    bool isFindRelative = false;
                    for (const auto& track : trackLanes_) {
                        const bool isRelative =
                                (!track->param_.isInverse &&
                                 (pt->point.x() > (track->param_.start - 1)) &&
                                 (pt->point.x() < (track->param_.end + 1)) &&
                                 track->GetDistance(pt) < 0.5) ||
                                ((track->param_.isInverse &&
                                  (pt->point.y() > (track->param_.start - 1)) &&
                                  (pt->point.y() < (track->param_.end + 1)) &&
                                  track->GetDistance(pt) < 0.5));
                        if (isRelative) {
                            track->relativePoints.emplace_back(pt);
                            isFindRelative = true;
                            break;
                        }
                    }

                    if (!isFindRelative) {
        //                std::cout << "This point not find match line" << std::endl;
                        noMatchPoints.emplace_back(pt);
                    }
                }
            }

            ///@debug
            for (const auto& track : trackLanes_) {
                if (!track->relativePoints.empty()) {
                    std::cout << "This track line has relative points: " << track->relativePoints.size() << std::endl;
                } else {
                    std::cout << "This track line has empty relative points" << std::endl;
                }
            }
        }*/

        void LaneFusion::Match(const std::vector<Point::Ptr> &detectPoints,
                               std::vector<Point::Ptr> &noMatchPoints)
        {
            //    std::cout << "*****match*****" << std::endl;
            if (detectPoints.empty())
            {
                return;
            }
            for (const auto &track : trackLanes_)
            {
                track->relativePoints.clear();
            }
            for (const auto &pt : detectPoints)
            {
                bool isFindRelative = false;
                for (const auto &track : trackLanes_)
                {
                    ///@brief consider start > end situation
                    bool isRelative;
                    if (track->param_.start > track->param_.end)
                    {
                        if (!track->param_.isInverse)
                        {
                            isRelative = (pt->point.x() > (track->param_.end - 2.0)) &&
                                         (pt->point.x() < (track->param_.start + 1.5)) &&
                                         track->GetDistance(pt) < 0.8;
                        }
                        else
                        {
                            isRelative = (pt->point.y() > (track->param_.end - 2.0)) &&
                                         (pt->point.y() < (track->param_.start + 1.5)) &&
                                         track->GetDistance(pt) < 0.8;
                        }
                    }
                    else
                    {
                        if (!track->param_.isInverse)
                        {
                            isRelative = (pt->point.x() > (track->param_.start - 2.0)) &&
                                         (pt->point.x() < (track->param_.end + 1.5)) &&
                                         track->GetDistance(pt) < 0.8;
                        }
                        else
                        {
                            isRelative = (pt->point.y() > (track->param_.start - 2.0)) &&
                                         (pt->point.y() < (track->param_.end + 1.5)) &&
                                         track->GetDistance(pt) < 0.8;
                        }
                    }
                    if (isRelative)
                    {
                        track->relativePoints.emplace_back(pt);
                        isFindRelative = true;
                        break;
                    }
                }

                if (!isFindRelative)
                {
                    noMatchPoints.emplace_back(pt);
                }
            }
        }

        void LaneFusion::FusionFilter()
        {
            //    std::cout << "*****filter*****" << std::endl;
            if (trackLanes_.empty())
            {
                return;
            }

            for (auto &track : trackLanes_)
            {
                ///@brief generate new start and end
                ParamPredict(track);
                //        std::cout << "Line: " << track->id << " has relative size: " << track->relativePoints.size() << std::endl;
                if (track->relativePoints.size() < 300)
                {
                    //        if (track->relativePoints.size() < 30) {
                    //        if (track->relativePoints.size() < 3U) {
                    --track->lifeCount;
                }
                else
                {
                    ++track->lifeCount;
                    track->lifeCount = std::min(track->lifeCount, 50);
                    LineParam lineParam;
                    ///@brief use new relative points to generate new k, b, isInverse
                    std::vector<Eigen::Vector3d> points;
                    for (const auto &pt : track->relativePoints)
                    {
                        points.emplace_back(pt->point);
                    }
                    DouglasPeucker::Polyfit(points, track->param_.isInverse, lineParam);
                    ///@brief generate new all params
                    ParamUpdate(lineParam, track);
                }
            }
        }

        void LaneFusion::ParamPredict(FusionLane::Ptr &track)
        {
            const double shrinkScale = 0.97;
            const double startTmp = track->param_.start;
            const double endTmp = track->param_.end;
            track->param_.start =
                (startTmp + endTmp) / 2.0 -
                shrinkScale * (endTmp - startTmp) / 2.0;
            track->param_.end =
                (startTmp + endTmp) / 2.0 +
                shrinkScale * (endTmp - startTmp) / 2.0;
        }

        void LaneFusion::ParamUpdate(LineParam &newParam,
                                     FusionLane::Ptr &track)
        {
            newParam.start = track->param_.start;
            newParam.end = track->param_.end;

            for (const auto &pt : track->relativePoints)
            {
                if (newParam.isInverse)
                {
                    newParam.start = std::min(newParam.start, pt->point.y());
                    newParam.end = std::max(newParam.end, pt->point.y());
                }
                else
                {
                    newParam.start = std::min(newParam.start, pt->point.x());
                    newParam.end = std::max(newParam.end, pt->point.x());
                }
            }

            const double startAndEndRatio = 0.5;
            track->param_.start = (1 - startAndEndRatio) * track->param_.start + startAndEndRatio * newParam.start;
            track->param_.end = (1 - startAndEndRatio) * track->param_.end + startAndEndRatio * newParam.end;

            ///@brief Initial tracking line is more of a belief in newParam(detect line)
            /// if newParam's k is mutated, lower value k
            double kRatio;
            if (track->lifeCount < 10)
            {
                kRatio = 0.1;
            }
            else
            {
                kRatio = 0.05;
            }
            ///@debug
            //    std::cout << "diff k : " << std::abs(newParam.k - track->param_.k) << std::endl;
            ///@debug end
            track->param_.k = (1 - kRatio) * track->param_.k + kRatio * newParam.k;

            double bRatio;
            if (track->lifeCount < 10)
            {
                bRatio = 0.1;
            }
            else
            {
                bRatio = 0.05;
            }
            ///@debug
            //    std::cout << "diff b : " << std::abs(newParam.b - track->param_.b) << std::endl;
            ///@debug end
            track->param_.b = (1 - bRatio) * track->param_.b + bRatio * newParam.b;
        }

        void LaneFusion::Clear()
        {
            if (trackLanes_.empty())
            {
                return;
            }
            for (auto iter = trackLanes_.begin(); iter != trackLanes_.end();)
            {
                if ((*iter)->lifeCount == 0)
                {
                    std::cout << "clear line id: " << (*iter)->id << std::endl;
                    //            idPool_[(* iter)->id] = false;
                    iter = trackLanes_.erase(iter);
                }
                else
                {
                    ++iter;
                }
            }
        }

        void LaneFusion::SetGridMap(const std::vector<Point::Ptr> &dataIn)
        {
            double minX = 1e7;
            double minY = 1e7;
            double maxX = -1e7;
            double maxY = -1e7;
            double precision = 0.1; // m

            for (const auto &point : dataIn)
            {
                minX = std::min(minX, point->point.x());
                minY = std::min(minY, point->point.y());
                maxX = std::max(maxX, point->point.x());
                maxY = std::max(maxY, point->point.y());
            }
            const double deltaX = maxX - minX;
            const double deltaY = maxY - minY;
            if (deltaX > 0.0 && deltaY > 0.0)
            {
                gridMapCluster_ = std::make_shared<AvpeGridMap>(minX, minY, deltaX, deltaY, precision);
                for (const auto &point : dataIn)
                {
                    gridMapCluster_->InsertElem(point->point);
                }
            }
        }

        void LaneFusion::Cluster(std::vector<ClusterFrame::Ptr> &allClusters)
        {
            //    std::cout << "*****cluster*****" << std::endl;
            if (gridMapCluster_ != nullptr)
            {
                const auto clusters = gridMapCluster_->Clustering(SearchMethod::FOUR_NEIGHBOUR);
                if (clusters.empty())
                {
                    return;
                }
                AvpeGrid::Ptr grid = std::make_shared<AvpeGrid>();
                for (const auto &cluster : clusters)
                {
                    ClusterFrame::Ptr oneCluster = std::make_shared<ClusterFrame>();
                    for (const auto &gridIndex : cluster)
                    {
                        if (gridMapCluster_->GetElem(gridIndex, grid))
                        {
                            oneCluster->SetIndex(grid->GetCenterCoord(),
                                                 grid->GetGridIndex());
                        }
                    }
                    //            std::cout << "one cluster has: " << oneCluster->pointsWorld.size() << std::endl;
                    if (int(oneCluster->pointsWorld.size()) < 70)
                    {
                        continue;
                    }
                    allClusters.emplace_back(oneCluster);
                    //            std::cout << "Cluster has: " << oneCluster->pointsWorld.size() << " points" << std::endl;
                }
                //        std::cout << "No match cluster size: " << allClusters.size() << std::endl;
            }
        }

        void LaneFusion::FindStartAndEndPoint(AvpeLaneFrame &frame)
        {
            for (const auto &cluster : frame.lineFrame_->noMatchPointsClusters_)
            {
                cluster->TransToBase(frame.poseOb);
                double maxDist = 1e-7;
                int iMin = -1, iMax = -1;
                for (int i = 0; i < int(cluster->pointsBase.size()); ++i)
                {
                    for (int j = i + 1; j < int(cluster->pointsBase.size()); ++j)
                    {
                        const double dist = (cluster->pointsBase[i]->point -
                                             cluster->pointsBase[j]->point)
                                                .norm();
                        if (dist < maxDist)
                        {
                            continue;
                        }
                        maxDist = dist;
                        iMin = i;
                        iMax = j;
                    }
                }
                if (cluster->pointsBase[iMin]->point.x() < cluster->pointsBase[iMax]->point.x())
                {
                    cluster->sEPointsBase[0] = cluster->pointsBase[iMin];
                    cluster->sEPointsBase[1] = cluster->pointsBase[iMax];
                }
                else
                {
                    cluster->sEPointsBase[1] = cluster->pointsBase[iMin];
                    cluster->sEPointsBase[0] = cluster->pointsBase[iMax];
                }
                cluster->TransToWorld(frame.poseOb);
            }
        }

        void LaneFusion::FindPath(AvpeLaneFrame &frame)
        {
            //    std::cout << "*****astar*****" << std::endl;
            if (frame.lineFrame_->noMatchPointsClusters_.empty())
            {
                return;
            }

            for (auto &cluster : frame.lineFrame_->noMatchPointsClusters_)
            {
                int rowMax = -1e7, rowMin = 1e7, colMax = -1e7, colMin = 1e7;
                for (const auto &point : cluster->pointsWorld)
                {
                    rowMax = std::max(int(point->index.row), rowMax);
                    rowMin = std::min(int(point->index.row), rowMin);
                    colMax = std::max(int(point->index.col), colMax);
                    colMin = std::min(int(point->index.col), colMin);
                }
                const int rows = rowMax - rowMin;
                const int cols = colMax - colMin;
                if (rows < 0 || cols < 0)
                {
                    //            std::cout << "Error, can not generate grid!" << std::endl;
                    return;
                }

                Astar astar;
                astar.ReSetGrid(cluster->pointsWorld,
                                cluster->sEPointsWorld,
                                rowMin, colMin);

                astar.InitAstar(rows, cols,
                                cluster->pointsWorld);

                Point::Ptr startPoint = cluster->sEPointsWorld[0];
                Point::Ptr endPoint = cluster->sEPointsWorld[1];

                astar.Search(startPoint, endPoint);
                //        printf("%s", "\nprint map: \n");
                //        astar.PrintMap();
                if (!astar.GetPath().empty())
                {
                    cluster->SetPath(astar.GetPath());
                }
            }
        }

        void LaneFusion::GenerateNewId(FusionLane::Ptr &line)
        {
            for (size_t i = 5000; i < idPool_.size(); ++i)
            {
                if (idPool_[i] == false)
                {
                    line->id = i;
                    idPool_[i] = true;
                    return;
                }
            }
        }

        void LaneFusion::Generate(AvpeLaneFrame &frame)
        {
            //    std::cout << "*****dp*****" << std::endl;
            newLines_.clear();
            if (frame.lineFrame_->noMatchPointsClusters_.empty())
            {
                return;
            }

            for (auto &cluster : frame.lineFrame_->noMatchPointsClusters_)
            {
                DouglasPeucker DP;
                DP.Init(cluster->pathWorld);
                DP.GenerateSegPoints(0, int(cluster->pathWorld.size()) - 1, distThre_);
                DP.Output();
                cluster->SetLines(DP.GetParams());
                //        std::cout << "This cluster has: " << cluster->lines.size() << " poly lines" << std::endl;
                newLines_.insert(newLines_.end(), cluster->lines.begin(), cluster->lines.end());
            }
            //    std::cout << "This frame has: " << newLines_.size() << " lines" << std::endl;
            for (auto &detectLine : newLines_)
            {
                GenerateNewId(detectLine);
            }
        }

        void LaneFusion::Add()
        {
            //    std::cout << "*****add*****" << std::endl;
            if (!newLines_.empty())
            {
                for (const auto &detectLine : newLines_)
                {
                    trackLanes_.emplace_back(detectLine);
                }
                ///@debug
                std::cout << "Add: " << newLines_.size() << " new lines" << std::endl;
                for (const auto &line : newLines_)
                {
                    std::cout << "new line id is: " << line->id << std::endl;
                }
                ///@debug end
            }
        }

        ///--------------------DEBUG INFORMATION------------------------
        void LaneFusion::ShowOdomLine(const std::vector<FusionLane::Ptr> &outputs)
        {
            visualization_msgs::msg::MarkerArray markersFusionLane;
            if (!outputs.empty())
            {
                for (const auto &output : outputs)
                {
                    PubFusionLane(output, markersFusionLane);
                    PubTest(output, markersFusionLane);
                }
            }
            fusionLaneMarkerPub_->publish(markersFusionLane);
        }

        void LaneFusion::ShowNoMatchPoints(const std::vector<Point::Ptr> &noMatchPoints)
        {
            if (noMatchPoints.empty())
            {
                return;
            }

            visualization_msgs::msg::MarkerArray markers;
            visualization_msgs::msg::Marker marker, markerText;
            marker.header.frame_id = "odom";
            marker.ns = "no_match_points";
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;

            std_msgs::msg::ColorRGBA color;
            /// red
            color.r = 1;
            color.a = 1;
            marker.color = color;
            marker.lifetime = rclcpp::Duration::from_seconds(1);
            for (const auto &pt : noMatchPoints)
            {
                geometry_msgs::msg::Point point;
                point.x = pt->point.x();
                point.y = pt->point.y();
                point.z = 0.0;
                marker.points.emplace_back(point);
            }
            markers.markers.emplace_back(marker);

            markerText.header.frame_id = "odom";
            markerText.header.stamp = nh_.get_clock()->now();
            markerText.lifetime = rclcpp::Duration::from_seconds(1);
            markerText.ns = "size";
            markerText.type = 9;
            markerText.action = 0;

            markerText.scale.x = 0.5;
            markerText.scale.y = 0.5;
            markerText.scale.z = 0.5;

            /// yellow
            markerText.color.r = 1;
            markerText.color.g = 1;
            markerText.color.a = 1;

            markerText.pose.orientation.w = -1.0;
            markerText.pose.orientation.x = 0.0;
            markerText.pose.orientation.y = 0.0;
            markerText.pose.orientation.z = 0.0;

            double centerX, centerY;

            for (const auto &point : noMatchPoints)
            {
                centerX += point->point.x();
                centerY += point->point.y();
            }
            centerX = centerX / noMatchPoints.size();
            centerY = centerY / noMatchPoints.size();
            markerText.pose.position.x = centerX;
            markerText.pose.position.y = centerY;
            markerText.pose.position.z = 0.0f;

            const std::string output = std::to_string(noMatchPoints.size());
            markerText.text = output;
            markers.markers.emplace_back(markerText);
            noMatchPointsMarkerPub_->publish(markers);
        }

        void LaneFusion::ShowMatchPoints(const std::vector<Point::Ptr> &matchPoints)
        {
            if (matchPoints.empty())
            {
                return;
            }

            visualization_msgs::msg::MarkerArray markers;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.ns = "match_points";
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;

            std_msgs::msg::ColorRGBA color;
            /// blue
            color.b = 1;
            color.a = 1;
            marker.color = color;
            marker.lifetime = rclcpp::Duration::from_seconds(1);
            for (const auto &pt : matchPoints)
            {
                geometry_msgs::msg::Point point;
                point.x = pt->point.x();
                point.y = pt->point.y();
                point.z = 0.0;
                marker.points.emplace_back(point);
            }
            markers.markers.emplace_back(marker);
            noMatchPointsMarkerPub_->publish(markers);
        }

        void LaneFusion::ShowdetectPoints(const LineFrame::Ptr &lineFrame)
        {
            if (lineFrame->detectPointsWorld_.empty())
            {
                return;
            }

            visualization_msgs::msg::MarkerArray markers;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.ns = "points";
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;

            std_msgs::msg::ColorRGBA color;
            /// white
            color.r = 1;
            color.g = 1;
            color.b = 1;
            color.a = 1;
            marker.color = color;
            marker.lifetime = rclcpp::Duration::from_seconds(1);
            for (const auto &pt : lineFrame->detectPointsWorld_)
            {
                geometry_msgs::msg::Point point;
                point.x = pt->point.x();
                point.y = pt->point.y();
                point.z = 0.0;
                marker.points.emplace_back(point);
            }
            markers.markers.emplace_back(marker);
            detectPointsMarkerPub_->publish(markers);
        }

        void LaneFusion::PubFusionLane(const FusionLane::Ptr &fusionLane,
                                       visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker, markerP;
            marker.header.frame_id = "odom";
            marker.ns = "fusion_lines";
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = 0;
            marker.scale.x = 0.1;
            marker.lifetime = rclcpp::Duration::from_seconds(1);
            std_msgs::msg::ColorRGBA color;
            // green
            color.g = 1;
            color.a = 1;
            marker.color = color;
            marker.id = fusionLane->id;
            //------------------------------------------
            markerP.header.frame_id = "odom";
            markerP.ns = "se_point";
            markerP.type = 8;
            markerP.action = 0;
            markerP.id = fusionLane->id;
            markerP.scale.x = 0.2;
            markerP.scale.y = 0.2;
            markerP.scale.z = 0.2;

            std_msgs::msg::ColorRGBA colorP;
            /// white
            colorP.r = 1;
            colorP.g = 1;
            colorP.b = 1;
            colorP.a = 1;
            markerP.color = colorP;
            markerP.lifetime = rclcpp::Duration::from_seconds(1);
            //------------------------------------------

            if (!fusionLane->param_.isInverse)
            {
                geometry_msgs::msg::Point startPoint;
                startPoint.x = fusionLane->param_.start;
                startPoint.y = fusionLane->param_.GetY(fusionLane->param_.start);
                startPoint.z = 0.0f;
                marker.points.emplace_back(startPoint);
                markerP.points.emplace_back(startPoint);
                geometry_msgs::msg::Point endPoint;
                endPoint.x = fusionLane->param_.end;
                endPoint.y = fusionLane->param_.GetY(fusionLane->param_.end);
                endPoint.z = 0.0f;
                marker.points.emplace_back(endPoint);
                markerP.points.emplace_back(endPoint);
            }
            else
            {
                geometry_msgs::msg::Point startPoint;
                startPoint.x = fusionLane->param_.GetY(fusionLane->param_.start);
                startPoint.y = fusionLane->param_.start;
                startPoint.z = 0.0f;
                marker.points.emplace_back(startPoint);
                markerP.points.emplace_back(startPoint);
                geometry_msgs::msg::Point endPoint;
                endPoint.x = fusionLane->param_.GetY(fusionLane->param_.end);
                endPoint.y = fusionLane->param_.end;
                endPoint.z = 0.0f;
                marker.points.emplace_back(endPoint);
                markerP.points.emplace_back(endPoint);
            }
            markers.markers.emplace_back(marker);
            markers.markers.emplace_back(markerP);
        }

        void LaneFusion::PubTest(const FusionLane::Ptr &fusionLane,
                                 visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker markerS, markerE, marker, markerSize;
            markerS.header.frame_id = "odom";
            markerS.header.stamp = nh_.get_clock()->now();
            markerS.lifetime = rclcpp::Duration::from_seconds(1);
            markerS.ns = "start";
            markerS.id = fusionLane->id;
            markerS.type = 9;
            markerS.action = 0;

            markerS.scale.x = 0.5;
            markerS.scale.y = 0.5;
            markerS.scale.z = 0.5;

            /// yellow
            markerS.color.r = 1;
            markerS.color.g = 1;
            markerS.color.a = 1;

            markerS.pose.orientation.w = -1.0;
            markerS.pose.orientation.x = 0.0;
            markerS.pose.orientation.y = 0.0;
            markerS.pose.orientation.z = 0.0;

            markerS.pose.position.x = fusionLane->param_.isInverse ? fusionLane->param_.GetY(fusionLane->param_.start)
                                                                   : fusionLane->param_.start;
            markerS.pose.position.y = fusionLane->param_.isInverse ? fusionLane->param_.start
                                                                   : fusionLane->param_.GetY(fusionLane->param_.start);
            markerS.pose.position.z = 0.0f;

            const std::string outputS = std::to_string(markerS.pose.position.x) + " " + std::to_string(markerS.pose.position.y);
            markerS.text = outputS;
            //----------------------------------------------------------------------
            markerE.header.frame_id = "odom";
            markerE.header.stamp = nh_.get_clock()->now();
            markerE.lifetime = rclcpp::Duration::from_seconds(1);
            markerE.ns = "end";
            markerE.id = fusionLane->id;
            markerE.type = 9;
            markerE.action = 0;

            markerE.scale.x = 0.5;
            markerE.scale.y = 0.5;
            markerE.scale.z = 0.5;

            /// yellow
            markerE.color.r = 1;
            markerE.color.g = 1;
            markerE.color.a = 1;

            markerE.pose.orientation.w = -1.0;
            markerE.pose.orientation.x = 0.0;
            markerE.pose.orientation.y = 0.0;
            markerE.pose.orientation.z = 0.0;

            markerE.pose.position.x = fusionLane->param_.isInverse ? fusionLane->param_.GetY(fusionLane->param_.end)
                                                                   : fusionLane->param_.end;
            markerE.pose.position.y = fusionLane->param_.isInverse ? fusionLane->param_.end
                                                                   : fusionLane->param_.GetY(fusionLane->param_.end);
            markerE.pose.position.z = 0.0f;

            const std::string outputE = std::to_string(markerE.pose.position.x) + " " + std::to_string(markerE.pose.position.y);
            markerE.text = outputE;
            //----------------------------------------------------------------------
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.lifetime = rclcpp::Duration::from_seconds(1);
            marker.ns = "KB";
            marker.id = fusionLane->id;
            marker.type = 9;
            marker.action = 0;

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            /// yellow
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.a = 1;

            marker.pose.orientation.w = -1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;

            const double centerX = fusionLane->param_.isInverse ? fusionLane->param_.GetY((fusionLane->param_.start + fusionLane->param_.end) / 2) : (fusionLane->param_.start + fusionLane->param_.end) / 2;

            const double centerY = fusionLane->param_.isInverse ? (fusionLane->param_.start + fusionLane->param_.end) / 2 : fusionLane->param_.GetY((fusionLane->param_.start + fusionLane->param_.end) / 2);

            marker.pose.position.x = centerX;
            marker.pose.position.y = centerY;
            marker.pose.position.z = 0.0f;

            const std::string output = fusionLane->param_.isInverse ? "x= " + std::to_string(fusionLane->param_.k) + " y+ " +
                                                                          std::to_string(fusionLane->param_.b)
                                                                    : "y= " + std::to_string(fusionLane->param_.k) + " x+ " +
                                                                          std::to_string(fusionLane->param_.b);
            marker.text = output;
            //------------------------------------------
            markerSize.header.frame_id = "odom";
            markerSize.header.stamp = nh_.get_clock()->now();
            markerSize.lifetime = rclcpp::Duration::from_seconds(1);
            markerSize.ns = "relative size";
            markerSize.id = fusionLane->id;
            markerSize.type = 9;
            markerSize.action = 0;

            markerSize.scale.x = 0.5;
            markerSize.scale.y = 0.5;
            markerSize.scale.z = 0.5;

            /// yellow
            markerSize.color.r = 1;
            markerSize.color.g = 1;
            markerSize.color.a = 1;

            markerSize.pose.orientation.w = -1.0;
            markerSize.pose.orientation.x = 0.0;
            markerSize.pose.orientation.y = 0.0;
            markerSize.pose.orientation.z = 0.0;

            markerSize.pose.position.x = centerX;
            markerSize.pose.position.y = centerY;
            markerSize.pose.position.z = 0.0f;

            const std::string outputSize =
                std::to_string(fusionLane->id) + " " + std::to_string(fusionLane->relativePoints.size()) + " " +
                std::to_string(fusionLane->lifeCount);

            markerSize.text = outputSize;
            //-----------------------------------------
            markers.markers.emplace_back(marker);
            markers.markers.emplace_back(markerS);
            markers.markers.emplace_back(markerE);
            markers.markers.emplace_back(markerSize);
        }
    }
}

#include "zebra_fusion/zebra_fusion.h"
#include "utils/min_rect_utils.h"
#include "chrono"
#include "algorithm"

namespace Fusion
{
    namespace PSE
    {

        void ZebraFusion::CANCallBack(const vehicle_msgs::msg::CanCopyCp2Ap50msStruct::SharedPtr msg)
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

        void ZebraFusion::DetectParkingStatus(const mpc_prk_msgs::msg::MsgSmHmi2Pnc::SharedPtr msg)
        {
            isParking_ = (msg->hmi_pnc_info.parkin_slot_id != 0) && (((msg->sm_pnc_info.apa_interior_status >= 9) && (msg->sm_pnc_info.apa_interior_status <= 12)) || ((msg->sm_pnc_info.rpastsin >= 5) && (msg->sm_pnc_info.rpastsin <= 12)));
            //    std::cout << "Parking status is :" << isParking_ << std::endl;
        }

        void ZebraFusion::Publish(AvpeZebraFrame &frame)
        {
        }

        bool ZebraFusion::MakeFrame(AvpeZebraFrame &frame)
        {
            return manager_.MakeFrame(frame);
        }

        void ZebraFusion::Detect(AvpeZebraFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }

            StoreMultiFrameZebra(frame);
            //    auto startTime2 = chrono::system_clock::now();
            Cluster(frame);
            //    chrono::duration<double> diff2 = chrono::system_clock::now() - startTime2;
            //    std::cout << "break zebra detect: " << diff2.count() << "s" << std::endl;
        }

        void ZebraFusion::Track(AvpeZebraFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            auto startTime2 = chrono::system_clock::now();
            MatchZebras(frame);
            UpdateTrackZebras();
            AddNewFoundZebras(frame);
            ClearUntrackObject(frame);
            FusionTrackedObjects();
            chrono::duration<double> diff2 = chrono::system_clock::now() - startTime2;
            std::cout << "break zebra track: " << diff2.count() << "s" << std::endl;
        }

        void ZebraFusion::Output(AvpeZebraFrame &frame)
        {
            if (isParking_)
            {
                return;
            }
            if (isDoorStatusOk_ == false)
            {
                return;
            }
            for (auto &trackZebraPtr : trackZebras_)
            {
                if (trackZebraPtr->IsOutputPub())
                {
                    trackZebraPtr->TransferToBase(frame.poseOb);
                    frame.outputZebras_.push_back(trackZebraPtr);
                }
            }
        }

        void ZebraFusion::Debug(AvpeZebraFrame &frame)
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
            //    ShowTextureBox(frame.textureZebras_);
            //    ShowOdomZebra(frame.clusterZebras_);
            std::cout << "Track zebra size: " << trackZebras_.size() << std::endl;
            std::cout << "Output zebra size: " << frame.outputZebras_.size() << std::endl;
            ShowOdomZebra(frame.outputZebras_);
            ShowPoint(frame);
#endif
        }

        void ZebraFusion::ShowTextureBox(const std::vector<FusionZebra::Ptr> &zebras)
        {
            visualization_msgs::msg::MarkerArray markers;
            if (!zebras.empty())
            {
                for (size_t i = 0U; i < zebras.size(); ++i)
                {
                    Pub(zebras[i], i + zebras.size(), markers);
                }
            }
            textureMarkerPub_->publish(markers);
        }

        void ZebraFusion::StoreMultiFrameZebra(AvpeZebraFrame &frame)
        {
            constexpr double minGapDist = 0.0;
            Odometry curOdom = frame.poseOb;
            double gapDist = (curOdom.translation - preOdom_.translation).norm();
            if (!frame.detectZebra_->points.empty() && gapDist >= minGapDist)
            {
                multiFrameZebras_.Push(frame.detectZebra_);
                preOdom_ = frame.poseOb;
            }
        }

        void ZebraFusion::RandomDownSampling()
        {

            std::vector<Eigen::Vector3d> allPoints;
            for (const auto &frame : multiFrameZebras_)
            {
                for (const auto &point : frame->worldPoints)
                {
                    allPoints.emplace_back(point);
                }
            }
            afterDownSampling_.clear();
            std::cout << "before down-sampling all points: " << allPoints.size() << std::endl;

            int sum = static_cast<int>(allPoints.size());
            int pick = sum * 0.2;
            if (pick < 2500)
            {
                pick = 2500;
            }

            srand((unsigned int)time(0));
            for (int i = 0; i < sum; ++i)
            {
                if (rand() % (sum - i) < pick)
                {
                    afterDownSampling_.emplace_back(allPoints[i]);
                    --pick;
                }
            }
            std::cout << "after down-sampling all points: " << afterDownSampling_.size() << std::endl;
        }

        /*void ZebraFusion::PointToMat()
        {
            double maxIndexX = -1e10;
            double maxIndexY = -1e10;
            double minIndexX = +1e10;
            double minIndexY = +1e10;
            std::vector<Eigen::Vector3d> allPoints;
            for (const auto& fusionZebraPtr : multiFrameZebras_) {
                for (const auto& point : fusionZebraPtr->worldPoints) {
                    maxIndexX = std::max(maxIndexX, point.x());
                    maxIndexY = std::max(maxIndexY, point.y());
                    minIndexX = std::min(minIndexX, point.x());
                    minIndexY = std::min(minIndexY, point.y());
                    allPoints.emplace_back(point);
                }
            }
            double deltaX = abs(maxIndexX - minIndexX);
            double deltaY = abs(maxIndexY - minIndexY);
            cv::Mat mat(deltaY * 100, deltaX * 100, CV_8UC1, 255);
            for (const auto& pt : allPoints) {

                int x = (pt.x() - minIndexX) * 100;
                int y = (pt.y() - minIndexY) * 100;

                if (x > 0 && x < deltaX * 100 && y > 0 && y < deltaY * 100) {
        //            mat.at<cv::Vec3b>(y, x)[0] = 1;
        //            mat.at<cv::Vec3b>(y, x)[1] = 1;
        //            mat.at<cv::Vec3b>(y, x)[2] = 1;
                    mat.at<int>(y, x) = 1;
                }
            }
            std::cout << "channel: "<<mat.channels() << std::endl;
            std::cout << mat << std::endl;
            cv::imshow("image", mat);
            cv::waitKey(0);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            std::cout << "contours size: " << contours.size() << std::endl;
            for (const auto& contour : contours) {
                std::cout << "contour size: " << contour.size() << std::endl;
            }
        }*/

        bool ZebraFusion::CreateGridMap(AvpeGridMap::Ptr &gMapPtr)
        {
            constexpr double percesion = 0.05; // m
            double maxIndexX = -1e10;
            double maxIndexY = -1e10;
            double minIndexX = +1e10;
            double minIndexY = +1e10;
            uint32_t pointCnt = 0;
            for (const auto &fusionZebraPtr : multiFrameZebras_)
            {
                pointCnt += fusionZebraPtr->worldPoints.size();
                for (const auto &point : fusionZebraPtr->worldPoints)
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

        void ZebraFusion::Cluster(AvpeZebraFrame &frame)
        {
            constexpr size_t minFrameCount = 5;
            if (multiFrameZebras_.size() < minFrameCount || frame.detectZebra_->points.empty())
            {
                return;
            }
            // 纹理实体聚类
            //    auto startTime2 = chrono::system_clock::now();
            ZebraTextureCluster(frame);
            //    chrono::duration<double> diff2 = chrono::system_clock::now() - startTime2;
            //    std::cout << "break2.1: " << diff2.count() << "s" << std::endl;

            // 斑马线实体聚类
            //    auto startTime3 = chrono::system_clock::now();
            ZebraCluster(frame);
            //    chrono::duration<double> diff3 = chrono::system_clock::now() - startTime3;
            //    std::cout << "break2.2: " << diff3.count() << "s" << std::endl;
        }

        void ZebraFusion::InitLocalMap(AvpeZebraFrame &frame)
        {
            double vehicleX = frame.poseOb.translation.x();
            double vehicleY = frame.poseOb.translation.y();
            double minX = vehicleX - this->mapRange_ * 0.5;
            double minY = vehicleY - this->mapRange_ * 0.5;
            this->gMapPtr_->SetLowerBound(minX, minY);
        }

        void ZebraFusion::ZebraTextureCluster(AvpeZebraFrame &frame)
        {
            // 0.1m ^ 2
            constexpr double minZebraTextureArea = 0.1;
            constexpr double minZebraTextureLength = 0.1;
            // down sampling
            RandomDownSampling();
            // create map
            InitLocalMap(frame);
            size_t pointNum = 0;
            for (const auto &point : afterDownSampling_)
            {
                gMapPtr_->InsertElem(point);
                ++pointNum;
            }

            /*for (const auto& fusionZebraPtr : multiFrameZebras_) {
                for (const auto& point : fusionZebraPtr->worldPoints) {
                    gMapPtr_->InsertElem(point);
                }
                pointNum += fusionZebraPtr->worldPoints.size();
            }*/

            //    std::cout << "Current Process Zebra Point Count is " << pointNum << std::endl;

            std::vector<std::vector<Eigen::Vector3d>> counters;
            gMapPtr_->FindContour(counters);
            for (const auto &cnt : counters)
            {
                FusionZebra::Ptr fusionZebraPtr = std::make_shared<FusionZebra>();
                fusionZebraPtr->worldEdgePoints = cnt;
                fusionZebraPtr->SetMinAreaRect();
                if (fusionZebraPtr->GetWorldRectArea() < minZebraTextureArea)
                {
                    continue;
                }
                if (fusionZebraPtr->GetWorldRectLength() < minZebraTextureLength)
                {
                    continue;
                }
                frame.clusterZebras_.push_back(fusionZebraPtr);
                ////@debug
                //        frame.textureZebras_.emplace_back(fusionZebraPtr);
                ////@debug end
            }
            gMapPtr_->Reset();
        }

        void ZebraFusion::InsertZebraTexutres(const AvpeZebraFrame &frame)
        {
            size_t count = frame.clusterZebras_.size();
            for (uint32_t i = 0U; i < count; ++i)
            {
                auto zebraPtr = frame.clusterZebras_[i];
                rtreeIndexPtr->Insert(Geometry::MakeBoostPolygon({zebraPtr->worldBoxPoints.begin(),
                                                                  zebraPtr->worldBoxPoints.end()}),
                                      i);
            }
        }

        void ZebraFusion::ZebraCluster(AvpeZebraFrame &frame)
        {
            constexpr uint8_t minTextureNum = 3;
            std::vector<FusionZebra::Ptr> zebraObjects;
            std::set<uint32_t> closeIds{}; // the set of all vistied
            std::set<uint32_t> openIds{};  // the set of id in queue;
            size_t count = frame.clusterZebras_.size();
            InsertZebraTexutres(frame);
            for (size_t curId = 0U; curId < count; ++curId)
            {
                if (closeIds.count(curId) == 1)
                {
                    continue;
                }
                // 对每个纹理找一遍相关纹理并把id存入openIds
                ZebraClusterSingle(frame.clusterZebras_, curId, closeIds, openIds);
                // fill zebra object info.
                if (openIds.size() <= minTextureNum)
                {
                    openIds.clear();
                    continue;
                }
                FusionZebra::Ptr fusionZebraPtr = std::make_shared<FusionZebra>();
                std::vector<Eigen::Vector3d> zebraTexturePoints{};
                for (const auto &textureId : openIds)
                {
                    auto zebraTexturePtr = frame.clusterZebras_[textureId];
                    if (zebraTexturePtr->IsNormTexture())
                    {
                        fusionZebraPtr->relativeTextures.emplace_back(zebraTexturePtr);
                    }

                    zebraTexturePoints.insert(zebraTexturePoints.end(),
                                              zebraTexturePtr->worldEdgePoints.begin(),
                                              zebraTexturePtr->worldEdgePoints.end());
                }
                openIds.clear();
                fusionZebraPtr->worldEdgePoints = zebraTexturePoints;
                fusionZebraPtr->SetMinAreaRect();

                /// maxAreaZebraPtr == fusionZebraPtr == 斑马线ptr
                fusionZebraPtr->maxAreaZebraPtr = std::make_shared<FusionZebra>();
                fusionZebraPtr->maxAreaZebraPtr->worldBoxPoints = fusionZebraPtr->worldBoxPoints;
                fusionZebraPtr->maxAreaZebraPtr->worldEdgePoints = fusionZebraPtr->worldEdgePoints;
                fusionZebraPtr->maxAreaZebraPtr->worldPoints = fusionZebraPtr->worldPoints;
                ///@debug
                zebraObjects.push_back(fusionZebraPtr);
            }

            // clear old zebra texture objects.
            frame.clusterZebras_.clear();
            rtreeIndexPtr->clear();

            // fill zebra objects.
            frame.clusterZebras_ = zebraObjects;
        }

        double
        ZebraFusion::GetZebraTextureDist(const FusionZebra::Ptr &fusionZebraPtr1, const FusionZebra::Ptr &fusionZebraPtr2)
        {
            double minDist = 1e10;
            for (size_t i = 1U; i < 5U; ++i)
            {
                for (size_t j = 1U; j < 5U; ++j)
                {
                    auto point1 = (fusionZebraPtr1->worldBoxPoints[i - 1] + fusionZebraPtr1->worldBoxPoints[i % 4U]) * 0.5;
                    auto point2 = (fusionZebraPtr2->worldBoxPoints[j - 1] + fusionZebraPtr2->worldBoxPoints[j % 4U]) * 0.5;
                    double dist = (point1 - point2).norm();
                    minDist = std::min(minDist, dist);
                }
            }
            return minDist;
        }

        double ZebraFusion::GetZebraTextureDirInnerProduct(const FusionZebra::Ptr &fusionZebraPtr1,
                                                           const FusionZebra::Ptr &fusionZebraPtr2)
        {
            uint32_t l1Index = fusionZebraPtr1->GetWorldRectLengthEdgeIndex();
            Eigen::Vector3d v1 = (fusionZebraPtr1->worldBoxPoints[l1Index + 1] - fusionZebraPtr1->worldBoxPoints[l1Index]);
            uint32_t l2Index = fusionZebraPtr2->GetWorldRectLengthEdgeIndex();
            Eigen::Vector3d v2 = (fusionZebraPtr2->worldBoxPoints[l2Index + 1] - fusionZebraPtr2->worldBoxPoints[l2Index]);
            return (v1.dot(v2)) / (v1.norm() * v2.norm());
        }

        void ZebraFusion::ZebraClusterSingle(const std::vector<FusionZebra::Ptr> &zebras,
                                             const uint32_t curId,
                                             std::set<uint32_t> &closeIds,
                                             std::set<uint32_t> &openIds)
        {
            const double minClusterDist = 1.5; // m 车体宽度 1.4 ~ 1.8
            const double error = 0.01;
            std::deque<uint32_t> queue{};
            queue.push_back(curId);
            openIds.insert(curId);
            while (!queue.empty())
            {
                auto curZebraId = queue.front();
                if (curZebraId >= zebras.size())
                {
                    continue;
                }
                auto curZebraPtr = zebras[curZebraId];
                if (closeIds.count(curZebraId) == 1)
                {
                    continue;
                }
                closeIds.insert(curZebraId);
                queue.pop_front();
                auto relZebraIds = Geometry::GetNearbyElemByRTree(rtreeIndexPtr,
                                                                  curZebraPtr->GetWorldRectCenter(),
                                                                  minClusterDist);
                for (const auto &relZebraId : relZebraIds)
                {
                    if (relZebraId == curZebraId ||
                        closeIds.count(relZebraId) == 1 ||
                        openIds.count(relZebraId) == 1)
                    {
                        continue;
                    }
                    auto relZebraPtr = zebras[relZebraId];
                    double dist = GetZebraTextureDist(curZebraPtr, relZebraPtr);
                    double innerProuduct = GetZebraTextureDirInnerProduct(curZebraPtr, relZebraPtr);
                    bool innerProductMatch = (1.0 - abs(innerProuduct)) < error;
                    bool distMatch = dist < minClusterDist;
                    if (innerProductMatch && distMatch)
                    {
                        openIds.insert(relZebraId);
                        queue.push_back(relZebraId);
                    }
                }
            }
        }

        bool ZebraFusion::IsMatched(const FusionZebra::Ptr &trackZebraPtr, const FusionZebra::Ptr &detectZebraPtr)
        {
            constexpr double error = 0.6;
            if (detectZebraPtr->isVisited)
            {
                return false;
            }
            bool isIntersect = Geometry::IsRectangleIntersect(trackZebraPtr->maxAreaZebraPtr->worldBoxPoints,
                                                              detectZebraPtr->worldBoxPoints);
            double innerProdct = GetZebraTextureDirInnerProduct(trackZebraPtr, detectZebraPtr);
            bool isSameDir = (1.0 - abs(innerProdct)) < error;
            return isIntersect && isSameDir;
        }

        void ZebraFusion::MatchZebras(AvpeZebraFrame &frame)
        {
            if (frame.clusterZebras_.empty())
            {
                return;
            }

            if (trackZebras_.empty())
            {
                trackZebras_.assign(frame.clusterZebras_.begin(), frame.clusterZebras_.end());
                for (const auto &fusionZebraPtr : trackZebras_)
                {
                    fusionZebraPtr->id = (++this->zebraId_);
                    fusionZebraPtr->isVisited = true;
                }
            }
            else
            {
                for (const auto &trackZebraPtr : trackZebras_)
                {
                    trackZebraPtr->isActive = false;
                    for (const auto &detectZebraPtr : frame.clusterZebras_)
                    {
                        bool matchResult = IsMatched(trackZebraPtr, detectZebraPtr);
                        if (matchResult)
                        {
                            if (!trackZebraPtr->IsConfirm())
                            {
                                trackZebraPtr->PushRelativeZebra(detectZebraPtr);
                            }
                            trackZebraPtr->isActive = true;
                            detectZebraPtr->isVisited = true;
                        }
                    }
                }
            }
        }

        void ZebraFusion::UpdateTrackZebras()
        {
            for (const auto &trackZebraPtr : trackZebras_)
            {
                if (!trackZebraPtr->isActive)
                {
                    --trackZebraPtr->avaliableCount;
                }
            }
        }

        void ZebraFusion::AddNewFoundZebras(AvpeZebraFrame &frame)
        {
            for (const auto &fusionZebraPtr : frame.clusterZebras_)
            {
                if (!fusionZebraPtr->isVisited)
                {
                    fusionZebraPtr->id = (++this->zebraId_);
                    trackZebras_.push_back(fusionZebraPtr);
                }
            }
        }

        void ZebraFusion::CalcOneDimNormProbParam(const std::vector<double> &xs, double &mean, double &stdVar)
        {
            uint32_t count = xs.size();
            double meanSum = 0.0;
            for (size_t i = 0U; i < count; ++i)
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

        void ZebraFusion::FusionByNormProbDistibute(const FusionZebra::Ptr &fusionZebraPtr,
                                                    std::vector<FusionZebra::Ptr> &fusionZebras)
        {
            std::vector<double> angles{};
            for (const auto &relZebraPtr : fusionZebraPtr->relativeZebras)
            {
                angles.push_back(relZebraPtr->GetWorldZebraDirection());
            }
            double angleMean = 0.0;
            double angleStdVar = 0.0;
            CalcOneDimNormProbParam(angles, angleMean, angleStdVar);
            // 1-sigma
            double angleLowerBound = angleMean - 1. * angleStdVar;
            double angleUpperBound = angleMean + 1. * angleStdVar;

            bool angleStdVarIsZero = (angleStdVar < 1e-20);
            for (const auto &relZebraPtr : fusionZebraPtr->relativeZebras)
            {
                double angle = relZebraPtr->GetWorldZebraDirection();
                bool isNormAngle = (angleLowerBound < angle && angle < angleUpperBound);
                if ((!angleStdVarIsZero) && isNormAngle)
                {
                    fusionZebras.push_back(relZebraPtr);
                }
            }
        }

        void ZebraFusion::SortByZebraWidthError(std::vector<FusionZebra::Ptr> &fusionZebras, double zebraMeanWidth)
        {
            struct
            {
                bool operator()(FusionZebra::Ptr fusionZebraPtr1, FusionZebra::Ptr fusionZebraPtr2) const
                {
                    return fusionZebraPtr1->zebraWidthError < fusionZebraPtr2->zebraWidthError;
                }
            } widthErrorSorter;

            for (const auto &relZebraPtr : fusionZebras)
            {
                relZebraPtr->zebraWidthError = abs(relZebraPtr->GetWorldRectWidth() - zebraMeanWidth);
            }
            if (!fusionZebras.empty())
            {
                std::sort(fusionZebras.begin(), fusionZebras.end(), widthErrorSorter);
            }
        }

        void ZebraFusion::SortByZebraLength(std::vector<FusionZebra::Ptr> &fusionZebras, double topNRadio)
        {
            struct
            {
                bool operator()(FusionZebra::Ptr fusionZebraPtr1, FusionZebra::Ptr fusionZebraPtr2) const
                {
                    return fusionZebraPtr1->GetWorldRectLength() > fusionZebraPtr2->GetWorldRectLength();
                }
            } lengthSorter;

            size_t topN = static_cast<size_t>(fusionZebras.size() * topNRadio);
            if (!fusionZebras.empty())
            {
                std::sort(fusionZebras.begin(), fusionZebras.begin() + topN, lengthSorter);
            }
        }

        double ZebraFusion::GetTextureMeanLength(const FusionZebra::Ptr &fusionZebraPtr)
        {
            double meanLength = 0.;
            size_t textureCnt = 0;
            double maxValue = 0.;
            double minValue = 1e10;
            for (const auto &relZebraPtr : fusionZebraPtr->relativeZebras)
            {
                textureCnt += relZebraPtr->relativeTextures.size();
                for (const auto &texturePtr : relZebraPtr->relativeTextures)
                {
                    double textureLength = texturePtr->GetWorldRectLength();
                    maxValue = std::max(maxValue, textureLength);
                    minValue = std::min(minValue, textureLength);
                    meanLength += textureLength;
                }
            }
            // delete max and min value, delete noise
            if (textureCnt > 3U)
            {
                meanLength -= maxValue;
                meanLength -= minValue;
                textureCnt -= 2U;
            }
            return meanLength / static_cast<double>(textureCnt);
        }

        void ZebraFusion::GetFusionedZebra(const FusionZebra::Ptr &trackZebraPtr, FusionZebra::Ptr &fusionedZebraPtr)
        {
            constexpr double topNRadio = 0.5;
            double zebraMeanWidth = GetTextureMeanLength(trackZebraPtr);
            std::vector<FusionZebra::Ptr> fusionZebraPtrs{};
            FusionByNormProbDistibute(trackZebraPtr, fusionZebraPtrs);
            if (fusionZebraPtrs.size() > 2U)
            {
                SortByZebraWidthError(fusionZebraPtrs, zebraMeanWidth);
                SortByZebraLength(fusionZebraPtrs, topNRadio);
                if (!fusionZebraPtrs.empty())
                {
                    fusionedZebraPtr = fusionZebraPtrs.front();
                }
            }
            else
            {
                SortByZebraWidthError(trackZebraPtr->relativeZebras, zebraMeanWidth);
                SortByZebraLength(trackZebraPtr->relativeZebras, topNRadio);
                if (!trackZebraPtr->relativeZebras.empty())
                {
                    fusionedZebraPtr = trackZebraPtr->relativeZebras.front();
                }
            }
        }

        void ZebraFusion::UpdateTrackZebraPoints(FusionZebra::Ptr &trackZebraPtr,
                                                 const FusionZebra::Ptr &fusionedZebraPtr)
        {
            trackZebraPtr->worldEdgePoints.assign(fusionedZebraPtr->worldEdgePoints.begin(),
                                                  fusionedZebraPtr->worldEdgePoints.end());
            for (size_t i = 0U; i < 4U; ++i)
            {
                trackZebraPtr->worldBoxPoints[i] = fusionedZebraPtr->worldBoxPoints[i];
            }
        }

        void ZebraFusion::FusionMultiObjectZebra(FusionZebra::Ptr &trackZebraPtr)
        {
            FusionZebra::Ptr fusionedZebraPtr = nullptr;
            if (trackZebraPtr->IsConfirm())
            {
                GetFusionedZebra(trackZebraPtr, fusionedZebraPtr);
            }
            else
            {
                fusionedZebraPtr = trackZebraPtr->maxAreaZebraPtr;
            }
            if (fusionedZebraPtr == nullptr)
            {
                return;
            }

            /// 0 < fusionedZebraPtr长边 - trackZebraPtr长边 < 0.5
            constexpr double minWidthDelta = 0.5;
            if (!trackZebraPtr->IsConfirm())
            {
                if (fusionedZebraPtr->GetWorldRectLength() < trackZebraPtr->GetWorldRectLength())
                {
                    return;
                }
                if (abs(trackZebraPtr->GetWorldRectWidth() - fusionedZebraPtr->GetWorldRectWidth()) >
                    minWidthDelta)
                {
                    return;
                }
            }
            // update
            /// fusionedZebraPtr作为trackZebraPtr进行下一次track和fusion
            UpdateTrackZebraPoints(trackZebraPtr, fusionedZebraPtr);
        }

        void ZebraFusion::ClearUntrackObject(AvpeZebraFrame &frame)
        {
            constexpr double maxDist = 15.;
            const Eigen::Vector3d trans2d(frame.poseOb.translation.x(), frame.poseOb.translation.y(), 0.0);
            for (auto trackIter = trackZebras_.begin(); trackIter != trackZebras_.end();)
            {
                //        Eigen::Vector3d zebraCoord = ((* trackIter)->worldBoxPoints[0U] + (* trackIter)->worldBoxPoints[1U]) * 0.5;
                Eigen::Vector3d zebraCoord = ((*trackIter)->worldBoxPoints[1U] + (*trackIter)->worldBoxPoints[2U]) * 0.5;
                double curFrameToZebraDist = (trans2d - zebraCoord).norm();
                if ((*trackIter)->avaliableCount == 0U || curFrameToZebraDist > maxDist)
                {
                    std::cout << "clear zebra" << std::endl;
                    (*trackIter)->clear();
                    trackIter = trackZebras_.erase(trackIter);
                }
                else
                {
                    ++trackIter;
                }
            }
        }

        void ZebraFusion::FusionTrackedObjects()
        {
            for (auto &trackZebraPtr : trackZebras_)
            {
                if (trackZebraPtr->IsOutputPub())
                {
                    FusionMultiObjectZebra(trackZebraPtr);
                }
            }
        }

#if DEBUG_MODE

        void ZebraFusion::ShowOdomZebra(const std::vector<FusionZebra::Ptr> &zebras)
        {
            visualization_msgs::msg::MarkerArray markers;

            if (!zebras.empty())
            {
                /* for (size_t i = 0U ; i < zebras.size() ; ++i) {
         //            Pub(zebras[i],i,markers);
                     PushSingleOdomZebraMarker(zebras[i], i, markers);
                     // show box
                     PushSingleOdomZebraBoxMarker(zebras[i], i + zebras.size(), markers);
                 }*/
                for (const auto &zebra : zebras)
                {
                    PushSingleOdomZebraMarker(zebra, markers);
                    PushSingleOdomZebraBoxMarker(zebra, markers);
                }
            }
            fusionOdomZebraMarkerPub_->publish(markers);
        }

        void ZebraFusion::ShowPoint(AvpeZebraFrame &frame)
        {
            /*if (frame.detectZebra_->worldPoints.empty()) {
                return;
            }
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "origin_points";
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            ///blue
            color.b = 1;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(1);

            for (auto& worldPoint : frame.detectZebra_->worldPoints) {
                geometry_msgs::msg::Point point;
                point.x = worldPoint.x();
                point.y = worldPoint.y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
            }
            zebraPointsMarkerPub_->publish(marker);*/
            ////------------------------------------------
            if (multiFrameZebras_.size() < 5)
            {
                return;
            }
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "multi_points";
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            /// blue
            color.b = 1;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(1);
            /*for (const auto& test : multiFrameZebras_) {
                for (const auto& worldPoint : test->worldPoints) {
                    geometry_msgs::msg::Point point;
                    point.x = worldPoint.x();
                    point.y = worldPoint.y();
                    point.z = 0.0;
                    marker.points.push_back(std::move(point));
                }
            }*/

            for (auto &worldPoint : afterDownSampling_)
            {
                geometry_msgs::msg::Point point;
                point.x = worldPoint.x();
                point.y = worldPoint.y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
            }

            zebraPointsMarkerPub_->publish(marker);
        }

        void ZebraFusion::PushSingleOdomZebraBoxMarker(const FusionZebra::Ptr &fusionZebra,
                                                       //                                               const uint16_t id,
                                                       visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "box";
            marker.id = fusionZebra->id;
            marker.type = 4;
            marker.action = 0;

            marker.scale.x = 0.08;
            marker.scale.y = 0.08;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(1);

            for (uint32_t i = 0U; i < fusionZebra->worldBoxPoints.size() + 1; ++i)
            {
                geometry_msgs::msg::Point point;
                uint32_t index = i % 4U;
                point.x = fusionZebra->worldBoxPoints[index].x();
                point.y = fusionZebra->worldBoxPoints[index].y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }
            markers.markers.emplace_back(marker);
            ///-----------------------------------------
            visualization_msgs::msg::Marker markerId;
            markerId.header.frame_id = "odom";
            markerId.header.stamp = nh_.get_clock()->now();
            markerId.ns = "id";
            markerId.id = fusionZebra->id;
            markerId.type = 9;
            markerId.action = 0;

            markerId.scale.x = 0.3;
            markerId.scale.y = 0.3;
            markerId.scale.z = 0.3;

            std_msgs::msg::ColorRGBA colorId;
            colorId.g = 1;
            colorId.r = 1;
            colorId.a = 1;
            markerId.color = colorId;

            markerId.lifetime = rclcpp::Duration::from_seconds(1);
            markerId.pose.orientation.w = -1.0;
            markerId.pose.orientation.x = 0.0;
            markerId.pose.orientation.y = 0.0;
            markerId.pose.orientation.z = 0.0;

            double centerX = 0, centerY = 0;
            for (const auto &corner : fusionZebra->worldBoxPoints)
            {
                centerX += corner.x();
                centerY += corner.y();
            }
            centerX /= 4;
            centerY /= 4;

            markerId.pose.position.x = centerX;
            markerId.pose.position.y = centerY;
            markerId.pose.position.z = 0.0f;

            const std::string output = std::to_string(fusionZebra->id);
            markerId.text = output;
            markers.markers.emplace_back(markerId);
        }

        void ZebraFusion::PushSingleOdomZebraMarker(const FusionZebra::Ptr &fusionZebra,
                                                    //                                            const uint16_t id,
                                                    visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "edge_points";
            marker.id = fusionZebra->id;
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.0;

            /// green
            std_msgs::msg::ColorRGBA color;
            color.g = 1;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(1);

            for (uint32_t i = 0U; i < fusionZebra->worldEdgePoints.size(); ++i)
            {
                geometry_msgs::msg::Point point;
                point.x = fusionZebra->worldEdgePoints[i].x();
                point.y = fusionZebra->worldEdgePoints[i].y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }
            markers.markers.emplace_back(marker);
        }

        void ZebraFusion::ShowPerceptionZebra(const FusionZebra::Ptr &zebras)
        {
            visualization_msgs::msg::MarkerArray markers;
            PushSinglePerceptionZebraMarker(zebras, 0, markers);
            //    fusionPerceptionZebraMarkerPub_->publish(markers);
        }

        void ZebraFusion::PushSinglePerceptionZebraMarker(const FusionZebra::Ptr &fusionZebra, const uint16_t id,
                                                          visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "ego";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "fusion";
            marker.id = id;
            marker.type = 8;
            marker.action = 0;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1;
            marker.color = color;
            marker.lifetime = rclcpp::Duration::from_seconds(0);

            for (uint32_t i = 0U; i < fusionZebra->points.size(); ++i)
            {
                geometry_msgs::msg::Point point;
                point.x = fusionZebra->points[i].x();
                point.y = fusionZebra->points[i].y();
                point.z = 0.0;

                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }

            markers.markers.emplace_back(marker);
        }

        void ZebraFusion::Pub(const FusionZebra::Ptr &fusionZebra,
                              const uint16_t id,
                              visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = nh_.get_clock()->now();
            marker.ns = "texture_box";
            marker.id = id;
            marker.type = 4;
            marker.action = 0;

            marker.scale.x = 0.08;
            marker.scale.y = 0.08;
            marker.scale.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 1;
            color.g = 1;
            color.b = 0;
            color.a = 1;
            marker.color = color;

            marker.lifetime = rclcpp::Duration::from_seconds(1);

            for (uint32_t i = 0U; i < fusionZebra->worldBoxPoints.size() + 1; ++i)
            {
                geometry_msgs::msg::Point point;
                uint32_t index = i % 4U;
                point.x = fusionZebra->worldBoxPoints[index].x();
                point.y = fusionZebra->worldBoxPoints[index].y();
                point.z = 0.0;
                marker.points.push_back(std::move(point));
                marker.colors.push_back(std::move(color));
            }
            markers.markers.emplace_back(marker);
            ///-------------------------------------------
            visualization_msgs::msg::Marker markerFirstPoint;
            markerFirstPoint.header.frame_id = "odom";
            markerFirstPoint.header.stamp = nh_.get_clock()->now();
            markerFirstPoint.lifetime = rclcpp::Duration::from_seconds(1);
            markerFirstPoint.ns = "corner0";
            markerFirstPoint.id = id;
            markerFirstPoint.type = 9;
            markerFirstPoint.action = 0;

            markerFirstPoint.scale.x = 0.3;
            markerFirstPoint.scale.y = 0.3;
            markerFirstPoint.scale.z = 0.3;

            /// yellow
            markerFirstPoint.color.g = 1;
            markerFirstPoint.color.a = 1;

            markerFirstPoint.pose.orientation.w = -1.0;
            markerFirstPoint.pose.orientation.x = 0.0;
            markerFirstPoint.pose.orientation.y = 0.0;
            markerFirstPoint.pose.orientation.z = 0.0;

            markerFirstPoint.pose.position.x = fusionZebra->worldBoxPoints[0].x();
            markerFirstPoint.pose.position.y = fusionZebra->worldBoxPoints[0].y();
            markerFirstPoint.pose.position.z = 0.0f;

            const std::string outputSize = "0" /*+ std::to_string(fusionZebra->worldBoxPoints[0].x()) + " " +
                                            std::to_string(fusionZebra->worldBoxPoints[0].y())*/
                ;
            markerFirstPoint.text = outputSize;
            markers.markers.emplace_back(markerFirstPoint);
            ///-------------------------------------------
            visualization_msgs::msg::Marker markerSecondPoint;
            markerSecondPoint.header.frame_id = "odom";
            markerSecondPoint.header.stamp = nh_.get_clock()->now();
            markerSecondPoint.lifetime = rclcpp::Duration::from_seconds(1);
            markerSecondPoint.ns = "corner1";
            markerSecondPoint.id = id;
            markerSecondPoint.type = 9;
            markerSecondPoint.action = 0;

            markerSecondPoint.scale.x = 0.3;
            markerSecondPoint.scale.y = 0.3;
            markerSecondPoint.scale.z = 0.3;

            /// yellow
            markerSecondPoint.color.g = 1;
            markerSecondPoint.color.a = 1;

            markerSecondPoint.pose.orientation.w = -1.0;
            markerSecondPoint.pose.orientation.x = 0.0;
            markerSecondPoint.pose.orientation.y = 0.0;
            markerSecondPoint.pose.orientation.z = 0.0;

            markerSecondPoint.pose.position.x = fusionZebra->worldBoxPoints[1].x();
            markerSecondPoint.pose.position.y = fusionZebra->worldBoxPoints[1].y();
            markerSecondPoint.pose.position.z = 0.0f;
            const std::string outputSize1 = "1 " /*+ std::to_string(fusionZebra->worldBoxPoints[1].x()) + " " +
                                            std::to_string(fusionZebra->worldBoxPoints[1].y())*/
                ;
            markerSecondPoint.text = outputSize1;
            markers.markers.emplace_back(markerSecondPoint);

            ///-------------------------------------------
            visualization_msgs::msg::Marker markerPoint3;
            markerPoint3.header.frame_id = "odom";
            markerPoint3.header.stamp = nh_.get_clock()->now();
            markerPoint3.lifetime = rclcpp::Duration::from_seconds(1);
            markerPoint3.ns = "corner2";
            markerPoint3.id = id;
            markerPoint3.type = 9;
            markerPoint3.action = 0;

            markerPoint3.scale.x = 0.3;
            markerPoint3.scale.y = 0.3;
            markerPoint3.scale.z = 0.3;

            /// yellow
            markerPoint3.color.g = 1;
            markerPoint3.color.a = 1;

            markerPoint3.pose.orientation.w = -1.0;
            markerPoint3.pose.orientation.x = 0.0;
            markerPoint3.pose.orientation.y = 0.0;
            markerPoint3.pose.orientation.z = 0.0;

            markerPoint3.pose.position.x = fusionZebra->worldBoxPoints[2].x();
            markerPoint3.pose.position.y = fusionZebra->worldBoxPoints[2].y();
            markerPoint3.pose.position.z = 0.0f;

            const std::string outputSize2 = "2" /*+ std::to_string(fusionZebra->worldBoxPoints[2].x()) + " " +
                                            std::to_string(fusionZebra->worldBoxPoints[2].y())*/
                ;
            markerPoint3.text = outputSize2;
            markers.markers.emplace_back(markerPoint3);
        }

#endif

    }
}

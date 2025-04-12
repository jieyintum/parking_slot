#ifndef ARROW_FRAME
#define ARROW_FRAME

#include <array>
#include <memory>
#include <Eigen/Eigen>

#include "frame_base.h"
#include "utils/min_rect_utils.h"
#include "utils/pld_utils.h"
#include "inner_msgs/avpe.h"
#include "inner_msgs/static_elem.h"

namespace Fusion
{
    namespace PSE
    {

#define ARROW_TYPE_CNT 14U

        struct FusionArrow
        {

            using Ptr = std::shared_ptr<FusionArrow>;

            void TransferToWorld(const Odometry &odom)
            {
                this->worldPoints.resize(this->points.size());
                this->worldEdgePoints.resize(this->edgePoints.size());

                for (size_t i = 0U; i < points.size(); ++i)
                {
                    worldPoints[i] = odom.orientation * points[i] + odom.translation;
                }

                for (size_t i = 0U; i < edgePoints.size(); ++i)
                {
                    worldEdgePoints[i] = odom.orientation * edgePoints[i] + odom.translation;
                }

                for (size_t i = 0U; i < boxPoints.size(); ++i)
                {
                    worldBoxPoints[i] = odom.orientation * boxPoints[i] + odom.translation;
                }
            }

            void TransferToBase(const Odometry &odom)
            {

                this->points.resize(this->worldPoints.size());
                this->edgePoints.resize(this->worldEdgePoints.size());

                for (size_t i = 0U; i < worldPoints.size(); ++i)
                {
                    points[i] = (odom.orientation.inverse() * (worldPoints[i] - odom.translation));
                }

                for (size_t i = 0U; i < worldEdgePoints.size(); ++i)
                {
                    edgePoints[i] = (odom.orientation.inverse() * (worldEdgePoints[i] - odom.translation));
                }

                for (size_t i = 0U; i < worldBoxPoints.size(); ++i)
                {
                    boxPoints[i] = (odom.orientation.inverse() * (worldBoxPoints[i] - odom.translation));
                }
            }

            inline double GetWorldRectArea() const
            {
                double w = (worldBoxPoints[0U] - worldBoxPoints[1U]).norm();
                double h = (worldBoxPoints[1U] - worldBoxPoints[2U]).norm();

                return w * h;
            }

            inline double GetWorldRectWidth() const
            {
                double l1 = (worldBoxPoints[0U] - worldBoxPoints[1U]).norm();
                double l2 = (worldBoxPoints[1U] - worldBoxPoints[2U]).norm();

                return std::min(l1, l2);
            }

            inline double GetWorldRectLength() const
            {
                double l1 = (worldBoxPoints[0U] - worldBoxPoints[1U]).norm();
                double l2 = (worldBoxPoints[1U] - worldBoxPoints[2U]).norm();

                return std::max(l1, l2);
            }

            /*
                brief: cos direction
            */
            double GetWorldArrowDirection() const
            {
                double l1 = (worldBoxPoints[0U] - worldBoxPoints[1U]).norm();
                double l2 = (worldBoxPoints[1U] - worldBoxPoints[2U]).norm();
                double dir = 0.0;
                if (l1 > l2)
                {
                    dir = (worldBoxPoints[1U].y() - worldBoxPoints[0U].y()) / l1;
                }
                else
                {
                    dir = (worldBoxPoints[2U].y() - worldBoxPoints[1U].y()) / l2;
                }

                return dir;
            }

            std::pair<Eigen::Vector3d, Eigen::Vector3d> GetWorldArrowDirVector() const
            {
                double l1 = (worldBoxPoints[0U] - worldBoxPoints[1U]).norm();
                double l2 = (worldBoxPoints[1U] - worldBoxPoints[2U]).norm();
                double dir = 0.0;
                if (l1 > l2)
                {
                    return {worldBoxPoints[0U], worldBoxPoints[1U]};
                }
                else
                {
                    return {worldBoxPoints[1U], worldBoxPoints[2U]};
                }
            }

            StaticElement::Classification GetArrowTypeByVote()
            {
                int maxType = 1;
                int maxCnt = 0;
                for (int i = 0; i < ARROW_TYPE_CNT; ++i)
                {
                    if (arrowTypes[i] > maxCnt)
                    {
                        maxType = i;
                        maxCnt = arrowTypes[i];
                    }
                    std::cout << i << ":" << arrowTypes[i] << " | ";
                }
                std::cout << std::endl;
                std::cout << this->id << " Vote type is: " << maxType << std::endl;
                int baseType = static_cast<int>(StaticElement::GROUND_ARROW_STARGIHT) - 1;
                auto arrowType = static_cast<StaticElement::Classification>(maxType + baseType);
                return arrowType;
            }

            inline bool IsOutputPub()
            {
                auto arrowType = this->GetArrowTypeByVote();
                bool isOutputType = (outputTypes.find(arrowType) != outputTypes.end());
                if (!isOutputType)
                {
                    std::cout << "Current ArrowType is " << isOutputType << " and cannot output!" << std::endl;
                }
                return (this->relativeArrows.size() >= 3U) && isOutputType;
            }

            inline bool IsConfirm() const
            {
                return this->relativeArrows.size() >= 10U;
            }

            void SetMinAreaRect()
            {
                // get min area rect.
                MinAreaRectCV rectExtractor(this->worldEdgePoints);
                std::array<Eigen::Vector3d, 4U> rect;
                rectExtractor.Solve(rect);
                this->worldBoxPoints = rect;
            }

            double CalcBoxArea() const
            {
                if (!worldBoxPoints.empty())
                {
                    return (worldBoxPoints[0] - worldBoxPoints[1]).norm() * (worldBoxPoints[1] - worldBoxPoints[2]).norm();
                }
                return 0.0;
            }

            StaticElement ToMsg()
            {
                std::vector<VertexStruct> vertexList{};
                for (size_t i = 0; i < 4; ++i)
                {
                    VertexStruct v;
                    v.x = worldOutputBoxPoints[i % 4].x();
                    v.y = worldOutputBoxPoints[i % 4].y();
                    vertexList.push_back(v);
                }

                int voteSubType = this->GetArrowTypeByVote();
                auto subClass = static_cast<StaticElement::Classification>(voteSubType);
                StaticElement staticElem(vertexList, subClass);
                staticElem.id = this->id;
                return staticElem;
            }

            void SetWorldEdgePoints(std::vector<Eigen::Vector3d> pWorldEdgePoints)
            {
                worldEdgePoints = pWorldEdgePoints;
                Eigen::Vector3d edgeCenterPoint = {0.0, 0.0, 0.0};
                for (const auto &point : pWorldEdgePoints)
                {
                    edgeCenterPoint += point;
                }
                edgeCenterPoint /= pWorldEdgePoints.size();
                SetWorldCenterPoint(edgeCenterPoint);
            }

            void SetWorldCenterPoint(const Eigen::Vector3d &pWorldCenterPoint)
            {
                worldCenterPoint = pWorldCenterPoint;
            }

            Eigen::Vector3d GetWorldCenterPoint() const
            {
                return worldCenterPoint;
            }

            void AddArrowType(int type)
            {
                arrowTypes[type] += 1;
            }

        public:
            int id = 4001;

            bool isVisited = false; // if visited for match
            bool isActive = false;  // if checkout new object

            // life time.
            uint32_t avaliableCount = 30;

            // relation arrow cache
            FixedSizeDeque<FusionArrow::Ptr> relativeArrows{20};

            // 类型推导顺序 1->2->3
            // 1.每个像素的类别编号，范围为 1-13;
            std::vector<int> pixleTypes{};
            // 2.像素推导出的当前箭头类型
            int subType = 1;
            // 3.跟踪对象的类别统计表，投票综合得出最终类别
            std::array<uint32_t, ARROW_TYPE_CNT> arrowTypes{};

            // 原始点云
            std::vector<Eigen::Vector3d> points{};
            std::vector<Eigen::Vector3d> worldPoints{};

            // 拟合后的边界点
            std::vector<Eigen::Vector3d> edgePoints{};
            std::vector<Eigen::Vector3d> worldEdgePoints{};

            // 箭头中心点
            Eigen::Vector3d centerPoint;
            Eigen::Vector3d worldCenterPoint;

            // 最小外接矩形包围盒
            std::array<Eigen::Vector3d, 4U> boxPoints{};
            std::array<Eigen::Vector3d, 4U> worldBoxPoints{};

            // 虚拟矩形包围盒
            std::array<Eigen::Vector3d, 4U> outputBoxPoints{};
            std::array<Eigen::Vector3d, 4U> worldOutputBoxPoints{};

            // 矢量方向
            std::vector<Eigen::Vector3d> direction{};

            // 可发布集合
            std::set<StaticElement::Classification> outputTypes = {
                StaticElement::GROUND_ARROW_LEFT,
                StaticElement::GROUND_ARROW_RIGHT,
                StaticElement::GROUND_ARROW_STARGIHT,
                StaticElement::GROUND_ARROW_STARGIHT_LEFT,
                StaticElement::GROUND_ARROW_STARGIHT_RIGHT,
                StaticElement::GROUND_ARROW_LEFT_RIGHT,
            };

            // debug
            std::vector<Eigen::Vector3d> tailInter{};

            std::vector<Eigen::Vector3d> headBox{};
            std::vector<Eigen::Vector3d> tailBox{};
        };

        struct AvpeArrowFrame : public FrameBase
        {

            FusionArrow::Ptr detectArrow_;
            std::vector<FusionArrow::Ptr> clusterArrows_;
            std::vector<FusionArrow::Ptr> outputArrows_;
        };

    }
}

#endif // ARROW_FRAME

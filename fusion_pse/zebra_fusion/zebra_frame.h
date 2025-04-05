#ifndef __ZEBRA_FRAME_H__
#define __ZEBRA_FRAME_H__

#include <set>
#include <array>
#include <memory>
#include <Eigen/Eigen>

#include "utils/pld_utils.h"
#include "frame_base.h"
#include "inner_msgs/avpe.h"
#include "opencv4/opencv2/opencv.hpp"

namespace Fusion {
namespace PSE {
struct FusionZebra {

    using Ptr = std::shared_ptr<FusionZebra>;

    FusionZebra() = default;

    ~FusionZebra() = default;

    void TransferToWorld(const Odometry& odom)
    {
        this->worldPoints.resize(this->points.size());
        this->worldEdgePoints.resize(this->edgePoints.size());

        for (size_t i = 0U ; i < points.size() ; ++i) {
            worldPoints[i] = odom.orientation * points[i] + odom.translation;
        }

        for (size_t i = 0U ; i < edgePoints.size() ; ++i) {
            worldEdgePoints[i] = odom.orientation * edgePoints[i] + odom.translation;
        }

        for (size_t i = 0U ; i < 4U ; ++i) {
            worldBoxPoints[i] = odom.orientation * boxPoints[i] + odom.translation;
        }
    }

    void TransferToBase(const Odometry& odom)
    {
        this->points.resize(this->worldPoints.size());
        this->edgePoints.resize(this->worldEdgePoints.size());

        for (size_t i = 0U ; i < worldPoints.size() ; ++i) {
            points[i] = (odom.orientation.inverse() * (worldPoints[i] - odom.translation));
        }

        for (size_t i = 0U ; i < worldEdgePoints.size() ; ++i) {
            edgePoints[i] = (odom.orientation.inverse() * (worldEdgePoints[i] - odom.translation));
        }

        for (size_t i = 0U ; i < 4U ; ++i) {
            boxPoints[i] = (odom.orientation.inverse() * (worldBoxPoints[i] - odom.translation));
        }
    }

    inline Eigen::Vector3d GetWorldRectCenter() const
    {
        return (worldBoxPoints[0U] + worldBoxPoints[1U] + worldBoxPoints[2U] + worldBoxPoints[3U]) / 4.0;
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

    inline uint32_t GetWorldRectLengthEdgeIndex() const
    {
        double l1 = (worldBoxPoints[0U] - worldBoxPoints[1U]).norm();
        double l2 = (worldBoxPoints[1U] - worldBoxPoints[2U]).norm();
        return l1 > l2 ? 0U : 1U;
    }

    inline double GetWorldRectWidthLengthRadio() const
    {
        double width = GetWorldRectWidth();
        double length = GetWorldRectLength();
        return width / length;
    }

    /*
        brief: cos direction
    */
    double GetWorldZebraDirection() const
    {
        double l1 = (worldBoxPoints[0U] - worldBoxPoints[1U]).norm();
        double l2 = (worldBoxPoints[1U] - worldBoxPoints[2U]).norm();
        double dir = 0.0;
        if (l1 > l2) {
            dir = (worldBoxPoints[1U].y() - worldBoxPoints[0U].y()) / l1;
        } else {
            dir = (worldBoxPoints[2U].y() - worldBoxPoints[1U].y()) / l2;
        }
        return dir;
    }


    void SetMinAreaRect()
    {
        // get min area rect.
        /*MinAreaRect rectExtractor(this->worldEdgePoints);
        std::array<Eigen::Vector3d, 4U> rect;
        rectExtractor.Solve(rect);
        this->worldBoxPoints = rect;*/

        vector<cv::Point2f> pointsIn;
        for (const auto& pt : this->worldEdgePoints) {
            pointsIn.emplace_back(pt.x(), pt.y());
        }
        cv::RotatedRect box = cv::minAreaRect(pointsIn);
        cv::Point2f boxCornerPoints[4];
        box.points(boxCornerPoints);
        for (int i = 0 ; i < 4 ; ++i) {
            this->worldBoxPoints[i].x() = boxCornerPoints[i].x;
            this->worldBoxPoints[i].y() = boxCornerPoints[i].y;
            this->worldBoxPoints[i].z() = 0.0;
        }

    }

    bool IsNormTexture()
    {
        constexpr double maxWHRadio = 0.7; // m
        constexpr double minWidth = 0.1;   // m
        constexpr double maxWidth = 0.35;  // m
        constexpr double minLength = 1.0;  // m
        double radio = this->GetWorldRectWidthLengthRadio();
        if (radio > maxWHRadio) {
            return false;
        }
        double width = this->GetWorldRectWidth();
        bool isNormTextureWidth = width > minWidth && width < maxWidth;
        double length = this->GetWorldRectLength();
        bool isNormTextureLength = length > minLength;
        return isNormTextureWidth && isNormTextureLength;
    }

    inline bool IsOutputPub() const
    {
        return this->relativeZebras.size() >= 3U;
    }

    inline bool IsConfirm() const
    {
        return this->relativeZebras.size() >= 20U;
    }

    uint32_t GetGeomId()
    {
        if (geomId > 1) {
            return geomId;
        }
        double x, y;
        if (!this->worldEdgePoints.empty()) {
            x = this->worldEdgePoints.front().x();
            y = this->worldEdgePoints.front().y();
        }
        uint32_t xId = (uint32_t) (x * 100) % (4096);
        uint32_t yId = (uint32_t) (y * 100) % (4096);
        geomId += xId;
        geomId <<= 12;
        geomId += yId;
        return geomId;
    }

    StaticElement ToMsg()
    {
        std::vector<VertexStruct> vertexList{};
        for (size_t i = 0U ; i < 4U ; ++i) {
            VertexStruct v;
            v.x = worldBoxPoints[i].x();
            v.y = worldBoxPoints[i].y();
            vertexList.push_back(v);
        }

        StaticElement staticElem(vertexList, StaticElement::Classification::GROUND_ZEBRA);
        staticElem.id = this->id;
        return staticElem;
    }

    void clear()
    {
        relativeZebras.clear();
        relativeTextures.clear();

        points.clear();
        worldPoints.clear();

        edgePoints.clear();
        worldEdgePoints.clear();

    }

    void PushRelativeZebra(const FusionZebra::Ptr& fusionZebraPtr)
    {
        this->relativeZebras.push_back(fusionZebraPtr);
        ///取detect斑马线和track斑马线的最大边长斑马线为maxAreaZebraPtr
        double curLength = fusionZebraPtr->GetWorldRectLength();
        double maxLength = 0.0;
        if (maxAreaZebraPtr == nullptr) {
            maxLength = GetWorldRectLength();
        } else {
            maxLength = maxAreaZebraPtr->GetWorldRectLength();
        }
        if (curLength > maxLength) {
//            maxAreaZebraPtr = fusionZebraPtr;
            fusionZebraPtr->maxAreaZebraPtr = std::make_shared<FusionZebra>();
            maxAreaZebraPtr->worldBoxPoints = fusionZebraPtr->worldBoxPoints;
            maxAreaZebraPtr->worldEdgePoints = fusionZebraPtr->worldEdgePoints;
            maxAreaZebraPtr->worldPoints = fusionZebraPtr->worldPoints;
        }
    }

#if DEBUG_MODE

    void Print()
    {
        std::cout << " ---------------------- " << std::endl;
        std::cout << "GeomId :" << GetGeomId() << std::endl;
        std::cout << "isActive :" << isActive << std::endl;
        std::cout << "isConfirm :" << IsConfirm() << std::endl;
        std::cout << "avaliableCount :" << avaliableCount << std::endl;
        std::cout << "relativeZebras :" << relativeZebras.size() << std::endl;
        std::cout << "relativeTextures :" << relativeTextures.size() << std::endl;
//        std::cout << "maxAreaZebraPtr :" << maxAreaZebraPtr->GetWorldRectLength() << std::endl;
    }

#endif

public:
    int id = 4501;

    uint32_t geomId = 0;

    bool isVisited = false;  // if visited for match
    bool isActive = false;   // if checkout new object

    double zebraWidthError = 0.0;
    uint32_t avaliableCount = 30;

    FusionZebra::Ptr maxAreaZebraPtr = nullptr;

    std::vector<FusionZebra::Ptr> relativeZebras{};
    std::vector<FusionZebra::Ptr> relativeTextures{};

    std::vector<Eigen::Vector3d> points{};
    std::vector<Eigen::Vector3d> worldPoints{};

    std::vector<Eigen::Vector3d> edgePoints{};
    std::vector<Eigen::Vector3d> worldEdgePoints{};

    std::array<Eigen::Vector3d, 4U> boxPoints{};
    std::array<Eigen::Vector3d, 4U> worldBoxPoints{};
};

struct AvpeZebraFrame : public FrameBase {

    FusionZebra::Ptr detectZebra_;
    std::vector<FusionZebra::Ptr> clusterZebras_;
    std::vector<FusionZebra::Ptr> outputZebras_;
//    std::vector<FusionZebra::Ptr> textureZebras_;
};


}
}

#endif // __ZEBRA_FRAME_H__
#ifndef __SPEEDBUMP_FRAME_H__
#define __SPEEDBUMP_FRAME_H__

#include <array>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "frame_base.h"
#include "utils/min_rect_utils.h"
#include "utils/overlap_checker.h"
#include "inner_msgs/avpe.h"
#include "inner_msgs/static_elem.h"

namespace Fusion {
namespace PSE {


struct SpeedBumpTrack {

    using Ptr = std::shared_ptr<SpeedBumpTrack>;

    void TransferToWorld(const Odometry& odom)
    {
        this->vertices_world.resize(this->vertices.size());

        for (size_t i = 0U; i < vertices.size(); ++i) {
            vertices_world[i] = odom.orientation * vertices[i] + odom.translation;
        }

        for (size_t i = 0U; i < center_points.size(); ++i) {
            center_points_world[i] = odom.orientation * center_points[i] + odom.translation;
        }
    }

    void TransferToBase(const Odometry& odom)
    {
        this->vertices.resize(this->vertices_world.size());

        for (size_t i = 0U; i < vertices_world.size(); ++i) {
            vertices[i] = (odom.orientation.inverse() * (vertices_world[i] - odom.translation));
        }

        for (size_t i = 0U; i < center_points_world.size(); ++i) {
            center_points[i] = (odom.orientation.inverse() * (center_points_world[i] - odom.translation));
        }
    }

    inline double GetWorldRectArea() const 
    {
        double w = (vertices_world[0U] - vertices_world[1U]).norm();
        double h = (vertices_world[1U] - vertices_world[2U]).norm();

        return w * h;
    }

    inline double GetWorldRectWidth() const 
    {
        double l1 = (vertices_world[0U] - vertices_world[1U]).norm();
        double l2 = (vertices_world[1U] - vertices_world[2U]).norm();

        return std::min(l1, l2);
    }

    inline double GetWorldRectLength() const
    {
        double l1 = (vertices_world[0U] - vertices_world[1U]).norm();
        double l2 = (vertices_world[1U] - vertices_world[2U]).norm();

        return std::max(l1, l2);
    }

    void GetCenterPoints()
    {
        if (this->vertices.empty()) return;

        auto l1 = (vertices[0U] - vertices[1U]).norm();
        auto l2 = (vertices[1U] - vertices[2U]).norm();

        if (l1 > l2) {
            center_points[0U] = (vertices[1U] + vertices[2U]) * 0.5;
            center_points[1U] = (vertices[0U] + vertices[3U]) * 0.5;
        }
        else {
            center_points[0U] = (vertices[0U] + vertices[1U]) * 0.5;
            center_points[1U] = (vertices[2U] + vertices[3U]) * 0.5;
        }
    }

    double GetWorldDirectionAssociation(const std::vector<Eigen::Vector3d>& rect){
        if (vertices_world.empty()) return 0.0;
        double l_ , l ;
        Eigen::Vector3d longerEdge , longerEdge_01 , longerEdge_10;
        double l1 = (vertices_world[0U] - vertices_world[1U]).norm();
        double l2 = (vertices_world[1U] - vertices_world[2U]).norm(); 
        if(l1 > l2){
           longerEdge = vertices_world[0U] - vertices_world[1U];       
        }else{
           longerEdge = vertices_world[1U] - vertices_world[2U]; 
        }
        double l1_ = (rect[0U] - rect[1U]).norm();
        double l2_ = (rect[1U] - rect[2U]).norm();    
        if(l1_ > l2_){
            longerEdge_01 = rect[0U] - rect[1U];
            longerEdge_10 = rect[1U] - rect[0U];
        }else{
            longerEdge_01 = rect[1U] - rect[2U];
            longerEdge_10 = rect[2U] - rect[1U];
        }
        double angle_01 = longerEdge.dot(longerEdge_01) / (longerEdge_01.norm() * longerEdge.norm());
        double angle_10 = longerEdge.dot(longerEdge_10) / (longerEdge_10.norm() * longerEdge.norm());
        double angle = (angle_01 > 0) ? angle_01 : angle_10;
        return angle;
    }
    /*
        brief: cos direction
    */
    double GetWorldDirection() const 
    {
        if (vertices_world.empty()) return 0.0;

        double l1 = (vertices_world[0U] - vertices_world[1U]).norm();
        double l2 = (vertices_world[1U] - vertices_world[2U]).norm();
        double dir = 0.0;
        if(l1 > l2) {
            dir = (vertices_world[1U].y() - vertices_world[0U].y()) / l1;
        } else {
            dir = (vertices_world[2U].y() - vertices_world[1U].y()) / l2;
        }
        
        return dir;
    }

    double GetWorldDistance2Rect(const std::vector<Eigen::Vector3d> rect) const 
    {
        double min_dist = std::numeric_limits<double>::max();
        if (rect.empty()) return min_dist;

        for (size_t i = 0; i < vertices_world.size(); ++i) {
            for (size_t j = 0; j < rect.size(); ++j) {
                auto dist = (vertices_world[i] - rect[j]).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }

        return min_dist;
    }
    double GetMinDistRectAngle(const std::vector<Eigen::Vector3d> rect) const {
        double min_dist = std::numeric_limits<double>::max();
        if (rect.empty()) return min_dist;
        int min_indexi = 0;
        int min_indexj = 0;
        for (size_t i = 0; i < vertices_world.size(); ++i) {
            for (size_t j = 0; j < rect.size(); ++j) {
                auto dist = (vertices_world[i] - rect[j]).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    min_indexi = i;
                    min_indexj = j;
                }
            }
        }
        double l01,l02;
        double l01j,l02j;
        Eigen::Vector3d directi,directj;
        Eigen::Vector3d directi01 = vertices_world[min_indexi] - vertices_world[(min_indexi + 1) % vertices_world.size()];
        l01 = directi01.norm();
        Eigen::Vector3d directi02 = vertices_world[min_indexi] - vertices_world[(min_indexi - 1) % vertices_world.size()];
        l02 = directi02.norm();
        if(l01 > l02){
            directi = directi01;
        }else{
            directi = directi02;
        }
        Eigen::Vector3d directj01 = rect[min_indexj] - rect[(min_indexj + 1) % rect.size()];
        l01j = directj01.norm();
        Eigen::Vector3d directj02 = rect[min_indexj] - rect[(min_indexj - 1) % rect.size()];
        l02j = directj02.norm();
        if(l01j > l02j){
            directj = directj01;                 
        }else{
            directj = directj02;        
        }
        double angle = directi.dot(directj);
        return angle;   
    }  
    bool ReconstructRect(const std::array<Eigen::Vector3d, 2U>& centerPts, std::vector<Eigen::Vector3d>& edgePts, const double width)
    {
        if (centerPts.empty()) return false;

        auto length = (centerPts[0] - centerPts[1]).norm(); 
        if (length < 1e-4) return false;

        auto ratio = width / length;
        auto dx = centerPts[1].x() - centerPts[0].x();
        auto dy = centerPts[1].y() - centerPts[0].y();
        center_points = centerPts;

        edgePts.resize(4);
        edgePts[0].x() = centerPts[0].x() - 0.5 * dy * ratio;
        edgePts[0].y() = centerPts[0].y() + 0.5 * dx * ratio;
        edgePts[1].x() = centerPts[0].x() + 0.5 * dy * ratio;
        edgePts[1].y() = centerPts[0].y() - 0.5 * dx * ratio;
        edgePts[2].x() = centerPts[1].x() + 0.5 * dy * ratio;
        edgePts[2].y() = centerPts[1].y() - 0.5 * dx * ratio;
        edgePts[3].x() = centerPts[1].x() - 0.5 * dy * ratio;
        edgePts[3].y() = centerPts[1].y() + 0.5 * dx * ratio;
        GetWorldDirection();

        return true;
    }

    bool IsCrossWithRect(const std::vector<Eigen::Vector3d>& rect_pts)
    {
        if (rect_pts.empty()) return false;

        auto px = std::abs(vertices_world[0].x() + vertices_world[2].x() - rect_pts[0].x() - rect_pts[2].x());
        auto py = std::abs(vertices_world[0].y() + vertices_world[2].y() - rect_pts[0].y() - rect_pts[2].y());
        auto l1 = (vertices_world[0U] - vertices_world[1U]).norm();
        auto w1 = (vertices_world[1U] - vertices_world[2U]).norm();
        auto l2 = (rect_pts[0U] - rect_pts[1U]).norm();
        auto w2 = (rect_pts[1U] - rect_pts[2U]).norm();
        if (px <= (l1 + l2) && py <= (w1 + w2)) {
            return true;
        }

        return false;
    }

    StaticElement ToMsg() 
    {
        std::vector<VertexStruct> vertexList{};
        for (size_t i = 0; i < 4U; ++i) {
            VertexStruct v;
            v.x = vertices_world[i].x();
            v.y = vertices_world[i].y();
            vertexList.push_back(v);
        }

        StaticElement staticElem(vertexList, StaticClassification::STATIC_SPEED_BUMP);
        staticElem.id = track_id;
        return staticElem;
    }


public:

    bool isVisited = false;  // if visited for match
    bool isActive = false;   // if checkout new object

    int     detect_id;
    int     track_id;
    int     measure_status;
    int     track_status;
    int     contributing_sensor;
    int     classification;
    int     number_of_vertices;
    float   life_time;
    float   exist_probability;
    float   direction;

    int track_count = 0;
    int miss_count = 0;
   
    std::vector<Eigen::Vector3d> vertices{};
    std::vector<Eigen::Vector3d> vertices_world{};

    std::array<Eigen::Vector3d, 2U> center_points{};
    std::array<Eigen::Vector3d, 2U> center_points_world{};
};

struct AvpeSpeedBumpFrame : public FrameBase {
    
    bool is_ego_static_ = false;

    std::vector<SpeedBumpTrack::Ptr> detect_speed_bumps_;
    std::vector<SpeedBumpTrack::Ptr> merged_speed_bumps_;
    std::vector<SpeedBumpTrack::Ptr> output_speed_bumps_;

};

}
}

#endif // __SPEEDBUMP_FRAME_H__
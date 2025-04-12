#include "speed_bump_fusion/speed_bump_fusion.h"

namespace Fusion
{
    namespace PSE
    {

        void SpeedBumpFusion::SetParkingMode(const std::atomic<bool> &mode)
        {
            is_Parking_Mode_ = mode;
        }

        bool SpeedBumpFusion::MakeFrame(AvpeSpeedBumpFrame &frame)
        {
            return manager_.MakeFrame(frame);
        }

        void SpeedBumpFusion::Detect(AvpeSpeedBumpFrame &frame)
        {
#if DEBUG_MODE
            // std::cout << "\n=== input speed bump: " << frame.detect_speed_bumps_.size() << std::endl;
#endif

            for (auto &ptr : frame.detect_speed_bumps_)
            {
                ptr->GetWorldDirection();
                // std::cout << "id: " << ptr->detect_id << ", direction: " << ptr->GetWorldDirection() << std::endl;
            }

            for (auto iter = frame.detect_speed_bumps_.begin(); iter != frame.detect_speed_bumps_.end(); iter++)
            {
                for (auto next_iter = iter + 1; next_iter != frame.detect_speed_bumps_.end();)
                {
                    auto min_dist = (*iter)->GetWorldDistance2Rect((*next_iter)->vertices_world);
                    auto d_angle = std::abs((*iter)->GetWorldDirection() - (*next_iter)->GetWorldDirection());

                    // add merge use track info
                    bool closest_to_track = false;
                    for (const auto &it : frame.output_speed_bumps_)
                    {
                        auto dist_iter = it->GetWorldDistance2Rect((*iter)->vertices_world);
                        auto dist_next = it->GetWorldDistance2Rect((*next_iter)->vertices_world);
                        if (dist_iter > 1.0 || dist_next > 1.0 || d_angle > 0.17)
                            continue;

                        closest_to_track = true;
                        // std::cout << " speed bumps close to track: " << it->detect_id << std::endl;
                    }

                    if ((min_dist < 1.0 && d_angle < 0.17) || closest_to_track)
                    {
                        MergeSpeedBumps(*iter, *next_iter);
#if DEBUG_MODE
                        // std::cout << " merge speed bumps: " << (*iter)->detect_id << " & " << (*next_iter)->detect_id
                        //           << ", length: " << (*iter)->GetWorldRectLength() << ", width: " << (*iter)->GetWorldRectWidth() << std::endl;
#endif
                        next_iter = frame.detect_speed_bumps_.erase(next_iter);
                    }
                    else
                    {
                        next_iter++;
                    }
                }
            }

            for (auto iter = frame.detect_speed_bumps_.begin(); iter != frame.detect_speed_bumps_.end(); iter++)
            {
                if ((*iter)->GetWorldRectWidth() > 1.5)
                {
                    iter = frame.detect_speed_bumps_.erase(iter);
                    iter--;
                }
            }
        }

        void SpeedBumpFusion::Track(AvpeSpeedBumpFrame &frame)
        {
            // std::cout<<"sb id: ";
            //  for (const auto& ptr : trackObjs_) {
            //      std::cout<<ptr->track_id<<" ";
            //  }
            //  std::cout<<"\n";

            if (!is_Parking_Mode_)
            {
                Reset();
                return;
            }

            is_ego_static_ = frame.is_ego_static_;
#if DEBUG_MODE
            // std::cout << "= measure size: " << frame.detect_speed_bumps_.size() << ", track size: " << trackObjs_.size()
            //           << ", is_ego_static: " << is_ego_static_ << "\n";
#endif

            if (trackObjs_.empty())
            {
                for (auto &ptr : frame.detect_speed_bumps_)
                {
                    GenerateNewId(ptr);
                    trackObjs_.emplace_back(ptr);
                }
                return;
            }

            if (frame.detect_speed_bumps_.empty())
            {
                for (auto &ptr : trackObjs_)
                {
                    ptr->detect_id = -1;
                    if (!is_ego_static_)
                    {
                        ptr->miss_count++;
                        NumLimit(ptr->miss_count);
                    }
                }
                return;
            }

            std::vector<int> trackObsFlag(trackObjs_.size(), -1);
            std::vector<int> measureObsFlag(frame.detect_speed_bumps_.size(), -1);
            AssociationRect(frame.detect_speed_bumps_, trackObsFlag, measureObsFlag);

            for (size_t i = 0; i < trackObjs_.size(); i++)
            {
                if (trackObsFlag[i] != -1)
                {
                    if (!is_ego_static_)
                    {
                        UpdateRect(trackObjs_[i], frame.detect_speed_bumps_[trackObsFlag[i]]);
                    }

                    UpdateAttribute(trackObjs_[i], frame.detect_speed_bumps_[trackObsFlag[i]]);
                }
                else
                {
                    trackObjs_[i]->detect_id = -1;
                    if (!is_ego_static_)
                    {
                        trackObjs_[i]->miss_count++;
                        NumLimit(trackObjs_[i]->miss_count);
                    }
                }
            }

            for (size_t i = 0; i < frame.detect_speed_bumps_.size(); i++)
            {
                if (measureObsFlag[i] == -1 && trackObjs_.size() < max_track_size_)
                {
                    GenerateNewId(frame.detect_speed_bumps_[i]);
                    trackObjs_.emplace_back(frame.detect_speed_bumps_[i]);
                }
            }

#if DEBUG_MODE
            for (const auto &ptr : trackObjs_)
            {
                // ", detect_id: " << ptr->detect_id
                //         << ", track_cnt: " << ptr->track_count << ", miss_cnt: " << ptr->miss_count
                //         << ", center pt1: " << ptr->center_points[0].x() << " " << ptr->center_points[0].y()
                //         << ", pt2: " << ptr->center_points[1].x() << " " << ptr->center_points[1].y() << "\n";
            }

            // std::cout << "= tracked size: " << trackObjs_.size() << std::endl;
#endif
        }

        void SpeedBumpFusion::Output(AvpeSpeedBumpFrame &frame)
        {
            frame.output_speed_bumps_.clear();
            DeleteUselessTrack(frame.poseOb);

            for (const auto &ptr : trackObjs_)
            {
                if ((ptr->track_count >= 5 && ptr->miss_count < 5) || ptr->track_count > 20)
                {
                    if (ptr->GetWorldRectLength() < 2.0 || ptr->GetWorldRectWidth() > 1.5)
                        continue;

                    frame.output_speed_bumps_.emplace_back(ptr);
                }
            }

#if DEBUG_MODE
            // std::cout << "= output size: " << frame.output_speed_bumps_.size() << std::endl;
#endif
        }

        void SpeedBumpFusion::Debug(AvpeSpeedBumpFrame &frame)
        {
            visualization_msgs::msg::MarkerArray markers;
            visualization_msgs::msg::Marker clean;
            clean.action = visualization_msgs::msg::Marker::DELETEALL;
            markers.markers.emplace_back(clean);

            ShowDetectObjs(frame.detect_speed_bumps_, markers);
            ShowTrackedObjs(frame.output_speed_bumps_, markers);

            SpeedBumpMarkerPub_->publish(markers);
        }

        void SpeedBumpFusion::Publish(AvpeSpeedBumpFrame &frame)
        {
        }

        void SpeedBumpFusion::MergeSpeedBumps(const SpeedBumpTrack::Ptr &cur, const SpeedBumpTrack::Ptr &next)
        {
            Eigen::Vector3d merge_pt1, merge_pt2;
            if ((cur->center_points_world[0] - next->center_points_world[0]).norm() >
                (cur->center_points_world[1] - next->center_points_world[0]).norm())
            {
                merge_pt1 = cur->center_points_world[0];
            }
            else
            {
                merge_pt1 = cur->center_points_world[1];
            }

            if ((merge_pt1 - next->center_points_world[0]).norm() > (merge_pt1 - next->center_points_world[0]).norm())
            {
                merge_pt2 = next->center_points_world[0];
            }
            else
            {
                merge_pt2 = next->center_points_world[1];
            }

            double merge_length, merge_width;
            merge_length = (merge_pt1 - merge_pt2).norm();
            if (cur->GetWorldRectLength() > next->GetWorldRectLength())
            {
                merge_width = cur->GetWorldRectLength();
            }
            else
            {
                merge_width = next->GetWorldRectLength();
            }

            cur->center_points_world[0] = merge_pt1;
            cur->center_points_world[1] = merge_pt2;
            cur->ReconstructRect(cur->center_points_world, cur->vertices_world, merge_width);
        }

        void SpeedBumpFusion::AssociationRect(std::vector<SpeedBumpTrack::Ptr> &measure,
                                              std::vector<int> &trackFlag,
                                              std::vector<int> &measureFlag)
        {
            if (measure.empty() || trackObjs_.empty())
                return;

            float max_dist = 10;
            float threshold = 2.0;
            Eigen::MatrixXf cost_matrix;
            cost_matrix.setZero(trackObjs_.size(), measure.size());

            for (size_t i = 0; i < trackObjs_.size(); i++)
            {
                Eigen::Vector4i indx(-1, -1, -1, -1);
                for (size_t j = 0; j < measure.size(); j++)
                {
                    auto dist = trackObjs_[i]->GetWorldDistance2Rect(measure[j]->vertices_world);
                    auto d_angle = trackObjs_[i]->GetWorldDirectionAssociation(measure[j]->vertices_world);
                    double directionAngle = trackObjs_[i]->GetMinDistRectAngle(measure[j]->vertices_world);
                    // std::cout<<" association "<<trackObjs_[i]->track_id<<" dis "<<dist <<"angle"<<directionAngle<<"\n";
                    // auto d_angle = std::abs(trackObjs_[i]->GetWorldDirection() - measure[j]->GetWorldDirection());
                    if (dist < threshold && d_angle > 0.98481 && directionAngle > 0)
                    {
                        cost_matrix(i, j) = dist;
                    }
                    else
                    {
                        cost_matrix(i, j) = max_dist;
                    }
                }
            }
            Eigen::MatrixXd::Index minRow = -1, minCol = -1;
            for (size_t i = 0; i < trackObjs_.size(); ++i)
            {
                double min_t_cost;
                min_t_cost = cost_matrix.minCoeff(&minRow, &minCol);

                if (min_t_cost <= threshold)
                {
                    for (int j = 0; j < measure.size(); ++j)
                    {
                        cost_matrix(minRow, j) = max_dist;
                    }
                    if (measureFlag[minCol] == -1)
                    {
                        measureFlag[minCol] = (int)minRow;
                    }
                    if (trackFlag[minRow] == -1 && (measureFlag[minCol] == minRow))
                    {
                        trackFlag[minRow] = (int)minCol;
                    }
                }
                else
                {
                    break;
                }
            }

            std::cout << " cost_matrix: \n"
                      << cost_matrix << "\n";
        }

        void SpeedBumpFusion::UpdateRect(const SpeedBumpTrack::Ptr &obj, const SpeedBumpTrack::Ptr &measure)
        {
            int track_idx, measure_idx;
            double update_distance = 2.0;
            double dist = GetClosestPts(measure, obj, track_idx, measure_idx);
            if (track_idx == -1 || measure_idx == -1 || dist > update_distance)
                return;

            double dist_obj = (obj->center_points_world[0] - obj->center_points_world[1]).norm();
            double dist_measure = (measure->center_points_world[0] - measure->center_points_world[1]).norm();
            double ratio = 0.5;
            if (dist_obj > dist_measure)
            {
                if (dist_measure > 3.0)
                {
                    ratio = 0.9;
                }
                else
                {
                    ratio = 1.0;
                }
            }

            for (int i = 0; i < 2; ++i)
            {
                for (int j = 0; j < 2; ++j)
                {
                    if ((track_idx == i && measure_idx == j) || (track_idx != i && measure_idx != j))
                    {
                        // UpdateMeasureRect(obj,measure);
                        if (dist_obj - dist_measure > 0.4 && (obj->center_points_world[i], measure->center_points_world[j]).norm() > update_distance)
                            continue;

                        obj->center_points_world[i] = obj->center_points_world[i] * ratio + measure->center_points_world[j] * (1 - ratio);
                    }
                }
            }

            auto width = obj->GetWorldRectWidth() * ratio + measure->GetWorldRectWidth() * (1 - ratio);
            obj->ReconstructRect(obj->center_points_world, obj->vertices_world, width);
        }

        void SpeedBumpFusion::UpdateMeasureRect(const SpeedBumpTrack::Ptr &obj, const SpeedBumpTrack::Ptr &measure)
        {

            const Eigen::Vector2d direction = (measure->center_points_world[1].block<2, 1>(0, 0) - measure->center_points_world[0].block<2, 1>(0, 0)).normalized();
            const double startXTrack = (obj->center_points_world[0].block<2, 1>(0, 0) - measure->center_points_world[0].block<2, 1>(0, 0)).dot(direction);
            const double endXTrack = (obj->center_points_world[1].block<2, 1>(0, 0) - measure->center_points_world[0].block<2, 1>(0, 0)).dot(direction);
            const double startXMeasure = 0;
            const double endXMeasure = (measure->center_points_world[1].block<2, 1>(0, 0) - measure->center_points_world[0].block<2, 1>(0, 0)).dot(direction);
            const double minX = std::min(std::min(startXTrack, endXTrack), std::min(startXMeasure, endXMeasure));
            const double maxX = std::max(std::max(startXTrack, endXTrack), std::max(startXMeasure, endXMeasure));
            // std::cout << "dist Start End " << startXTrack << " " << endXTrack << std::endl;
            if (endXMeasure > 0)
            {
                measure->center_points_world[0].block<2, 1>(0, 0) = measure->center_points_world[0].block<2, 1>(0, 0) + direction * minX;
                measure->center_points_world[1].block<2, 1>(0, 0) = measure->center_points_world[0].block<2, 1>(0, 0) + direction * maxX;
            }
            else
            {
                measure->center_points_world[0].block<2, 1>(0, 0) = measure->center_points_world[0].block<2, 1>(0, 0) + direction * maxX;
                measure->center_points_world[1].block<2, 1>(0, 0) = measure->center_points_world[0].block<2, 1>(0, 0) + direction * minX;
            }

            // std::cout<<"dist Start End track"<<track.pointStart.x()<<" "<<track.pointStart.y()<<" "<<track.pointEnd.x()<<" "<<track.pointEnd.y()<<std::endl;
            // std::cout<<"dist Start End Old"<<measure.pointStart.x()<<" "<<measure.pointStart.y()<<" "<<measure.pointEnd.x()<<" "<<measure.pointEnd.y()<<std::endl;
            // std::cout<<"dist Start End New"<<correctedMeasure.pointStart.x()<<" "<<correctedMeasure.pointStart.y()<<" "<<correctedMeasure.pointEnd.x()<<" "<<correctedMeasure.pointEnd.y()<<std::endl;
        }
        void SpeedBumpFusion::UpdateAttribute(const SpeedBumpTrack::Ptr &obj, const SpeedBumpTrack::Ptr &measure)
        {
            float alpha = 0.2f;
            obj->detect_id = measure->detect_id;
            obj->track_count++;
            obj->miss_count = 0;
            NumLimit(obj->track_count);
            obj->exist_probability = (1 - alpha) * obj->track_count / 20 + alpha * measure->exist_probability;
            obj->exist_probability = std::min(obj->exist_probability, 1.0f);
        }

        double SpeedBumpFusion::GetClosestPts(const SpeedBumpTrack::Ptr &a, const SpeedBumpTrack::Ptr &b, int &idx_a, int &idx_b)
        {
            double min_dist = 100.0;

            idx_a = -1;
            idx_b = -1;
            for (size_t i = 0; i < 2; ++i)
            {
                for (size_t j = 0; j < 2; ++j)
                {
                    auto dist = (a->center_points_world[i] - b->center_points_world[j]).norm();
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        idx_b = i;
                        idx_a = j;
                    }
                }
            }

            return min_dist;
        }

        void SpeedBumpFusion::DeleteUselessTrack(const Odometry &odom)
        {
            Eigen::Vector3d ego_center(1.5, 0.0, 0.0); // ego center point
            for (auto iter = trackObjs_.begin(); iter != trackObjs_.end(); iter++)
            {
                (*iter)->TransferToBase(odom);
                auto center = ((*iter)->center_points[0] + (*iter)->center_points[1]) * 0.5;
                auto dist = (center - ego_center).norm();
                if ((*iter)->miss_count > 20 && ((*iter)->track_count < 20 || dist > 5.0))
                {
                    idPool_[(*iter)->track_id] = false;
                    iter = trackObjs_.erase(iter);
                    iter--;
                }
            }

            for (auto iter = trackObjs_.begin(); iter != trackObjs_.end(); iter++)
            {
                for (auto next_iter = iter + 1; next_iter != trackObjs_.end();)
                {
                    std::vector<Eigen::Vector2d> trackerObj;
                    std::vector<Eigen::Vector2d> trackerObjNext;
                    for (int i = 0; i < (*iter)->vertices_world.size(); ++i)
                    {
                        Eigen::Vector2d tmp1((*iter)->vertices_world[i].x(), (*iter)->vertices_world[i].y());
                        Eigen::Vector2d tmp2((*next_iter)->vertices_world[i].x(), (*next_iter)->vertices_world[i].y());
                        trackerObj.emplace_back(tmp1);
                        trackerObjNext.emplace_back(tmp2);
                    }
                    bool flag = StaticObjectMerge::OverlapChecker::SATOverlap<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>, Eigen::Vector2d>(trackerObj, trackerObjNext);
                    // auto flag = (*iter)->IsCrossWithRect((*next_iter)->vertices_world);
                    auto dist = (*iter)->GetWorldDistance2Rect((*next_iter)->vertices_world);
                    double dist_iter = ((*iter)->center_points_world[0] - (*iter)->center_points_world[1]).norm();
                    double dist_next = ((*next_iter)->center_points_world[0] - (*next_iter)->center_points_world[1]).norm();
                    // std::cout<<"delete track "<< flag <<"\n";
                    if (flag)
                    {
                        if (dist_iter > dist_next)
                        {
                            // std::cout << " erase track id: " << (*next_iter)->track_id << "\n";
                            idPool_[(*next_iter)->track_id] = false;
                            next_iter = trackObjs_.erase(next_iter);
                        }
                        else
                        {
                            // std::cout << " erase track id: " << (*iter)->track_id << "\n";
                            idPool_[(*iter)->track_id] = false;
                            *iter = *next_iter;
                            next_iter = trackObjs_.erase(next_iter);
                        }
                    }
                    else
                    {
                        next_iter++;
                    }
                }
            }
        }

        void SpeedBumpFusion::Reset()
        {
            if (!trackObjs_.empty())
            {
                for (auto &iter : trackObjs_)
                {
                    idPool_[iter->track_id] = false;
                }
                trackObjs_.clear();
                // std::cout << "reset sb tracker! \n";
            }
        }

        void SpeedBumpFusion::GenerateNewId(const SpeedBumpTrack::Ptr &obj)
        {
            for (size_t i = 3000U; i < idPool_.size(); ++i)
            {
                if (idPool_[i] == false)
                {
                    obj->track_id = i;
                    idPool_[i] = true;
                    return;
                }
            }
        }

        void SpeedBumpFusion::NumLimit(int &count)
        {
            count = count < 0 ? 0 : count;
            count = count >= 100 ? 100 : count;
        }

#if DEBUG_MODE
        void SpeedBumpFusion::ShowDetectObjs(const std::vector<SpeedBumpTrack::Ptr> &speedBumps, visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker objs;

            for (size_t i = 0; i < speedBumps.size(); ++i)
            {
                if (speedBumps[i]->vertices.empty())
                    continue;

                // objs.header.stamp = header.stamp;
                objs.header.frame_id = "base_link";
                objs.ns = "camera_speed_bumps";
                objs.id = i;
                // objs.lifetime = rclcpp::Duration(0,100000000);
                objs.type = visualization_msgs::msg::Marker::LINE_STRIP;
                objs.action = visualization_msgs::msg::Marker::ADD;
                objs.pose.orientation.w = 1.0;
                objs.color.a = 1.0;
                objs.color.r = 0.5;
                objs.color.g = 0.5;
                objs.color.b = 0.0;
                objs.scale.x = 0.1;
                objs.scale.y = 0.1;
                for (size_t j = 0; j < speedBumps[i]->vertices.size(); ++j)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = speedBumps[i]->vertices[j].x();
                    pt.y = speedBumps[i]->vertices[j].y();
                    objs.points.emplace_back(pt);
                }
                geometry_msgs::msg::Point pt_end;
                pt_end.x = speedBumps[i]->vertices[0].x();
                pt_end.y = speedBumps[i]->vertices[0].y();
                objs.points.emplace_back(pt_end);

                markers.markers.emplace_back(objs);
                objs.points.clear();
            }
        }

        void SpeedBumpFusion::ShowTrackedObjs(const std::vector<SpeedBumpTrack::Ptr> &speedBumps, visualization_msgs::msg::MarkerArray &markers)
        {
            visualization_msgs::msg::Marker objs;

            for (size_t i = 0; i < speedBumps.size(); ++i)
            {
                if (speedBumps[i]->vertices.empty())
                    continue;

                // objs.header.stamp = header.stamp;
                objs.header.frame_id = "base_link";
                objs.ns = "tracked_speed_bumps";
                objs.id = i;
                // objs.lifetime = rclcpp::Duration(0,100000000);
                objs.type = visualization_msgs::msg::Marker::LINE_STRIP;
                objs.action = visualization_msgs::msg::Marker::ADD;
                objs.pose.orientation.w = 1.0;
                objs.color.a = 1.0;
                objs.color.r = 1.0;
                objs.color.g = 0.0;
                objs.color.b = 0.0;
                objs.scale.x = 0.1;
                objs.scale.y = 0.1;
                for (size_t j = 0; j < speedBumps[i]->vertices.size(); ++j)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = speedBumps[i]->vertices[j].x();
                    pt.y = speedBumps[i]->vertices[j].y();
                    objs.points.emplace_back(pt);
                }
                geometry_msgs::msg::Point pt_end;
                pt_end.x = speedBumps[i]->vertices[0].x();
                pt_end.y = speedBumps[i]->vertices[0].y();
                objs.points.emplace_back(pt_end);

                markers.markers.emplace_back(objs);
                objs.points.clear();
            }
        }
#endif

    }
}
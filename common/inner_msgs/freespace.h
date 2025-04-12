#ifndef INNER_MSGS_FREE_SPACE_H_
#define INNER_MSGS_FREE_SPACE_H_

#include "inner_msgs/msg_base.h"
#include "cem_interfaces/msg/freespace_struct.hpp"

namespace Fusion
{

    using FSMsg = cem_interfaces::msg::FreespaceStruct;
    using FSMsgPtr = FSMsg::SharedPtr;

    struct FSStruct : MsgBase
    {
        using Ptr = std::shared_ptr<FSStruct>;

        struct FSPoint
        {
            Eigen::Vector3d worldCorner;
            bool isOccupied;
        };
        typedef std::vector<FSPoint> FSObject;

        FSStruct() = default;

        ~FSStruct() = default;

        FSStruct(const FSMsgPtr &msgPtr) : MsgBase(msgPtr->header.stamp)
        {
            outerObjects_.clear();
            innerObjects_.clear();

            for (size_t i = 0; i < msgPtr->outer_elements_list.size(); i++)
            {
                FSObject outerObject;
                outerObject.clear();
                for (size_t j = 0; j < msgPtr->outer_elements_list[i].vertices_list.size(); j++)
                {
                    FSPoint newFSPoint;
                    newFSPoint.worldCorner << msgPtr->outer_elements_list[i].vertices_list[j].x,
                        msgPtr->outer_elements_list[i].vertices_list[j].y,
                        0.0;
                    newFSPoint.isOccupied = (msgPtr->outer_elements_list[i].freespace_property == 0) ? true : false;
                    outerObject.push_back(newFSPoint);
                }
                outerObjects_.push_back(outerObject);
            }

            for (size_t i = 0; i < msgPtr->inner_elements_list.size(); i++)
            {
                FSObject innerObject;
                innerObject.clear();
                for (size_t j = 0; j < msgPtr->inner_elements_list[i].vertices_list.size(); j++)
                {
                    FSPoint newFSPoint;
                    newFSPoint.worldCorner << msgPtr->inner_elements_list[i].vertices_list[j].x,
                        msgPtr->inner_elements_list[i].vertices_list[j].y,
                        0.0;
                    newFSPoint.isOccupied = (msgPtr->inner_elements_list[i].freespace_property == 0) ? true : false;
                    innerObject.push_back(newFSPoint);
                }
                innerObjects_.push_back(innerObject);
            }
        }

    public:
        std::vector<FSObject> outerObjects_;
        std::vector<FSObject> innerObjects_;
    };

}

#endif /* INNER_MSGS_FREE_SPACE_H_ */

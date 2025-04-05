#ifndef __STATIC_ELEM_H__
#define __STATIC_ELEM_H__

#include "msg_base.h"
#include "cem_interfaces/msg/static_elements_struct.hpp"

namespace Fusion {
namespace PSE {

using StaiticElementsStruct = cem_interfaces::msg::StaticElementsStruct;

using StaiticElementStruct = cem_interfaces::msg::StaticElementStruct;


using StaiticElementsMsg = cem_interfaces::msg::StaticElementsStruct;
using StaiticElementsMsgPtr = StaiticElementsMsg::SharedPtr;

using StaiticElementMsg = cem_interfaces::msg::StaticElementStruct;

using VertexStruct = cem_interfaces::msg::VertexStruct;

enum StaticClassification { STATIC_UNKNOEN = 0,
                            STATIC_FLAT,
                            STATIC_CURB,   
                            STATIC_ELEVATED,
                            STATIC_CONE_POLE,
                            STATIC_PARKED,
                            STATIC_SOFT,
                            STATIC_BAFFLE,
                            STATIC_BAFFLE_ONE_POINT = 11,
                            STATIC_SPEED_BUMP = 21,
                            STATIC_GROUND_ARROW = 22,
                            STATIC_GROUND_ZEBRA = 23,
                            STATIC_GROUND_LANE_LINE = 24,
                            STATIC_OTHER
                            };

struct StaticElement {
public:
enum Classification { 
    UNKNOEN = 0,
    FLAT,
    CURB,   
    ELEVATED,
    CONE_POLE,
    PARKED,
    SOFT,
    BAFFLE,
    BAFFLE_ONE_POINT = 11,

    SPEED_BUMP = 21,
    GROUND_PILLAR = 22,
    GROUND_WALL = 23,
    GROUND_LANE_LINE = 24,
    GROUND_ZEBRA = 25,

    GROUND_ARROW_STARGIHT = 26,
    GROUND_ARROW_LEFT,
    GROUND_ARROW_RIGHT,
    GROUND_ARROW_STARGIHT_LEFT ,
    GROUND_ARROW_STARGIHT_RIGHT,
    GROUND_ARROW_UTURN,
    GROUND_ARROW_LEFT_UTURN ,
    GROUND_ARROW_STRAIGHT_UTURN,
    GROUND_ARROW_LEFT_RIGHT,
    GROUND_ARROW_LEFT_MERGE ,
    GROUND_ARROW_RIGHT_MERGE ,
    GROUND_ARROW_STARGIHT_LEFT_RIGHT,
    GROUND_ARROW_OTHER,

    OTHER
};

public:
    StaticElement(const std::vector<VertexStruct>& vertices, const uint8_t classification): 
                    vertices_list(vertices), number_of_vertices(vertices.size()), classification(static_cast<Classification>(classification)) {};

    StaticElement(const int& id, const std::vector<VertexStruct>& vertices, const uint8_t classification): 
                    id(id), vertices_list(vertices), number_of_vertices(vertices.size()), classification(static_cast<Classification>(classification)) {};


    StaiticElementMsg ToMsg()
    {
        StaiticElementMsg singleElement;
        singleElement.vertices_list = this->vertices_list;
        singleElement.number_of_vertices = this->number_of_vertices;
        singleElement.classification = static_cast<uint8_t>(this->classification);

        return singleElement;
    }

    uint64_t last_measured_time_stamp;
    std::vector<VertexStruct> vertices_list;
    uint8_t number_of_vertices;

    Classification classification;
    int id;
};


struct StaticElements: MsgBase {
public:
    using Ptr = std::shared_ptr<StaticElements>;

    StaticElements(TimeStamp timestamp): MsgBase(timestamp) {}

    StaticElements(const StaiticElementsMsgPtr& msg) : MsgBase(msg->header.stamp)
    {
        for (const auto& element : msg->elements_list) {
            this->elems.emplace_back(element.vertices_list, element.classification);
        }
        this->elemNum = msg->elements_number;
    }

    StaticElements(const StaiticElementsMsgPtr& msg, const bool forWallPillar) : MsgBase(msg->header.stamp)
    {

        // GROUND_PILLAR = 22,
        // GROUND_WALL = 23,
        this->elemNum = 0;
        for (const auto& element : msg->elements_list) {
            if( (element.classification == 22) || (element.classification == 23)) {
                this->elems.emplace_back(element.id, element.vertices_list, element.classification);
                this->elemNum++;
            }
        }
    }



    void Append(const StaticElements& elements)
    {
        for (const auto& ele : elements.elems) {
            this->elems.push_back(ele);
            ++elemNum;
        }
    }
    
    void Append(StaticElement elem) 
    {
        elems.push_back(elem);
        ++elemNum;
    }

    StaiticElementsMsg ToMsg() {
        StaiticElementsMsg staticElemStructList;
        staticElemStructList.elements_number = elemNum;
        //std::cout<<"num of output ele: "<<elemNum<<"\n";
        staticElemStructList.header.stamp = timestamp.ToMsg();
        staticElemStructList.cem_header.time_stamp = timestamp.NSec();
        //std::cout<<"output id: ";
        for (size_t i = 0; i < elemNum; ++i) {
            StaiticElementMsg staticElemStruct;
            staticElemStruct.classification = elems[i].classification;
            staticElemStruct.number_of_vertices = elems[i].number_of_vertices;
            for (size_t j = 0; j < elems[i].number_of_vertices; ++j) {
                staticElemStruct.vertices_list.push_back(elems[i].vertices_list[j]);
            }
            staticElemStruct.id = elems[i].id;
            staticElemStructList.elements_list.push_back(staticElemStruct);

        //    std::cout<<elems[i].id<<" ";
        }

        //std::cout<<"\n";
        return staticElemStructList;
    }

public:
    std::vector<StaticElement> elems{};
    uint16_t elemNum = 0;
};

}
}



#endif // __STATIC_ELEM_H__
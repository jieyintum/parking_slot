#ifndef FUSION_LINE_POSITION_TYPE_H_
#define FUSION_LINE_POSITION_TYPE_H_

namespace Fusion {
enum class LinePositionType : uint8_t {
    UNKNOW               =0,
    FIRST_ON_THE_LEFT    =1,
    SECOND_ON_THE_LEFT   =2,
    THIRD_ON_THE_LEFT    =3,
    FORTH_ON_THE_LEFT    =4,
    FIFTH_ON_THE_LEFT    =5,
    SIXTH_ON_THE_LEFT    =6,
    SEVENTH_ON_THE_LEFT  =7,
    FIRST_ON_THE_RIGHT   =8,
    SECOND_ON_THE_RIGHT  =9,
    THIRD_ON_THE_RIGHT   =10,
    FORTH_ON_THE_RIGHT   =11,
    FIFTH_ON_THE_RIGHT   =12,
    SIXTH_ON_THE_RIGHT   =13,
    SEVENTH_ON_THE_RIGHT =14,
    OTHER                =15
}; 
}

#endif 
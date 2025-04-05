#ifndef LINE_COMMON_FUNCTION
#define LINE_COMMON_FUNCTION

#include "line_model.hpp"
#include "trackLine.h"
#include "line_math_function.hpp"
namespace Fusion
{   
    inline bool isPointInsideSlot(
        const std::array<Fusion::CTrackLine::Ptr, 4U> lines,
        const Eigen::Vector3d point)
    {
        for(int8_t i = 0; i < 4U; i++) {
            if(lines[i]->isPointOnRightSideOfUnitVector(point)) {
                return false;
            }
        }

        return true;
    }

    // //---------------------------------------------------------------------
    // //! @brief calculates cosine between two unit vectors
    // //! @param[in] f_unitVecA_vf32 : first unit vector
    // //! @param[in] f_unitVecB_vf32 : second unit vector
    // //! @return cosine-value in [-1,1]
    // //! @author  yij7szh
    // //---------------------------------------------------------------------
    inline float calcCosAngleBetweenTwoUnitVectors(const Eigen::Vector3d& f_unitVecA_vf32, const Eigen::Vector3d& f_unitVecB_vf32)
    {
        // u1,u2 are two unit vectors, therefore:
        // dot(u1,u2) = |u1| * |u2| cos(angle(u1,u2)) = cos(angle(u1,u1))
        float dotProduct_f32(0.0f);
        calcDotProd(f_unitVecA_vf32, f_unitVecB_vf32, dotProduct_f32);

        // treating floating point inaccuracy
        const float floatingPointTolerance = 0.00001f;
        if (isEqual(dotProduct_f32, 1.0f, floatingPointTolerance) && (1.0f < dotProduct_f32))
        {
            dotProduct_f32 = 1.0f;
        }
        else if (isEqual(dotProduct_f32, -1.0f, floatingPointTolerance) && (-1.0f > dotProduct_f32))
        {
            dotProduct_f32 = -1.0f;
        }
        else
        {
            // coding rule
            // do nothing
        }

        return dotProduct_f32;
    }


    //from -pi to pi in Radian
    //from UnitVectorA to UnitVectorB
    //positive is in clockwise
    inline float calAngleBetweenTwoUnitVector(
        const Eigen::Vector3d f_unitVectorA_vr,
        const Eigen::Vector3d f_unitVectorB_vr)
    {
        float l_angle_f = //from 0 to pi
            std::acos(calcCosAngleBetweenTwoUnitVectors(f_unitVectorA_vr, f_unitVectorB_vr));

        Eigen::Vector3d l_unitVectorATurn90InClockwise_v(-f_unitVectorA_vr[1], f_unitVectorA_vr[0], 0.0f);
        if(isPositive(
            calcCosAngleBetweenTwoUnitVectors(l_unitVectorATurn90InClockwise_v, f_unitVectorB_vr) )
        )
        {
            //do nothing   
        }
        else
        {
            l_angle_f = -l_angle_f;
        }

        return l_angle_f;
    }

    inline float calcAngleBetweenTwoUnitVectorInDegree(
        const Eigen::Vector3d f_unitVectorA_vr,
        const Eigen::Vector3d f_unitVectorB_vr)
    {
        float angleInRadian = calAngleBetweenTwoUnitVector(f_unitVectorA_vr, f_unitVectorB_vr);
        return (float)(angleInRadian*180.0f)/M_PI;
    }


    inline Eigen::Vector3d turnUnitVectorFor90DegreeInClockwiseInVehicleCoordinate(const Eigen::Vector3d f_unitVector_v)
    {
        Eigen::Vector3d l_returnUV(f_unitVector_v[1], -f_unitVector_v[0], 0.0f);
        
        return l_returnUV;
    }


    inline Eigen::Vector3d turnUnitVectorFor90DegreeInAntiClockwiseInVehicleCoordinate(const Eigen::Vector3d f_unitVector_v)
    {
        Eigen::Vector3d l_returnUV(-f_unitVector_v[1], f_unitVector_v[0], 0.0f);
        
        return l_returnUV;
    }


}


#endif /* LINE_COMMON_FUNCTION */

#ifndef _RSD_MATH_FUNCTION_HEADER_
#define _RSD_MATH_FUNCTION_HEADER_

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

#include <math.h>
#include <stdio.h>

#include "line_types.hpp"

using namespace std;

namespace Fusion
{
    template <class VectorType>
    inline VectorType power(const VectorType f_value_r)
    {
        return f_value_r * f_value_r;
    }

    template <class valueType>
    inline void sort(valueType &f_valueA_r,
                     valueType &f_valueB_r)
    {
        valueType l_tempValue;
        if (f_valueA_r > f_valueB_r)
        {
            l_tempValue = f_valueA_r;
            f_valueA_r = f_valueB_r;
            f_valueB_r = l_tempValue;
        }
    }

    template <class valueType1, class valueType2, class valueType3>
    inline bool isEqual(const valueType1 f_valueA_r,
                        const valueType2 f_valueB_r,
                        const valueType3 f_epsilonValue_r)
    {
        return (fabs(f_valueA_r - f_valueB_r) < f_epsilonValue_r) ? true : false;
    }

    template <class valueType1, class valueType2>
    inline bool isEqual(const valueType1 f_valueA_r,
                        const valueType2 f_valueB_r)
    {
        return (fabs(f_valueA_r - f_valueB_r) < 0.000001) ? true : false;
    }

    template <class Type>
    inline Type max(const Type &f_value1_v, const Type &f_value2_v)
    {
        return (f_value1_v > f_value2_v) ? f_value1_v : f_value2_v;
    }

    template <class Type>
    inline Type min(const Type &f_value1_v, const Type &f_value2_v)
    {
        return (f_value1_v < f_value2_v) ? f_value1_v : f_value2_v;
    }

    template <class Type>
    inline bool isValuesInOrder(
        const Type f_a_r,
        const Type f_b_r,
        const Type f_c_r)
    {
        bool l_returnValue_b(false);
        if (((f_a_r >= f_b_r) && (f_b_r >= f_c_r)) || ((f_a_r <= f_b_r) && (f_b_r <= f_c_r)))
        {
            l_returnValue_b = true;
        }

        return l_returnValue_b;
    }

    //---------------------------------------------------------------------
    //! @brief calculates dot product of to vectors on road surface (x,y) with height z=0
    //! @param[in] f_vec1_v : vector1
    //! @param[in] f_vec2_v : vector2
    //! @param[out] f_outDotProduct_r: dot product of vector1 and vector2
    //! @author  yinjie
    //---------------------------------------------------------------------
    template <class Vector1Type, class Vector2Type, typename RetType>
    inline void calcDotProd(const Vector1Type &f_vec1_v, const Vector2Type &f_vec2_v, RetType &f_outDotProduct_r)
    {

        // return value type depends on value type from input vectors,
        // wrong return value type will not compile

        // V1 dot V2 := V1[0] x V2[0] + V1[1] x V2[1] = |V1| x |V2| x cos{angle(V1,V2)}
        f_outDotProduct_r = (f_vec1_v[0] * f_vec2_v[0]) + (f_vec1_v[1] * f_vec2_v[1]);
    }

    //---------------------------------------------------------------------
    //! @brief calculates cross product of to vectors on road surface (x,y) with height z=0
    //! @param[in] f_vec1_v : vector1 of valid vector type
    //! @param[in] f_vec2_v : vector2 of valid vector type
    //! @param[out] f_outCrossProduct_r: cross product of vector1 and vector2
    //! @author  yinjie
    //---------------------------------------------------------------------
    template <class Vector1Type, class Vector2Type>
    inline float calcCrossProd(const Vector1Type &f_vec1_v, const Vector2Type &f_vec2_v)
    {
        // return value type depends on value type from input vectors,
        // wrong return value type will not compile

        // V1 cross V2 := V1[0] x V2[1] - V1[1] x V2[0] = |V1| x |V2| x sin{angle(V1,V2)}
        float f_outCrossProduct_r = (f_vec1_v[0] * f_vec2_v[1]) - (f_vec1_v[1] * f_vec2_v[0]);
        return f_outCrossProduct_r;
    }

    template <class VectorType>
    inline float calcLength(const VectorType &f_vec_v)
    {
        return std::fabs(std::sqrt(std::pow(f_vec_v[0], 2) + std::pow(f_vec_v[1], 2)));
    }

    inline float calcDistBetweenPoints(const Eigen::Vector3d f_pointA_r,
                                       const Eigen::Vector3d f_pointB_r)
    {
        return calcLength(Eigen::Vector3d(f_pointA_r[0] - f_pointB_r[0], f_pointA_r[1] - f_pointB_r[1], 0.0f));
    }

    template <class VectorType>
    inline VectorType getUnitVector(const VectorType &f_vec_v)
    {
        float l_length = calcLength(f_vec_v);
        VectorType l_returnVector(3);
        l_returnVector[0] = (float)f_vec_v[0] / l_length;
        l_returnVector[1] = (float)f_vec_v[1] / l_length;

        return l_returnVector;
    }

    template <class Type>
    inline bool isZero(Type f_value)
    {
        return (std::fabs(f_value) < 0.000001f) ? true : false;
    }

    template <class Type>
    inline bool isNegative(Type f_value)
    {
        return (f_value < 0.0f) ? true : false;
    }

    template <class Type>
    inline bool isPositive(Type f_value)
    {
        return (f_value > 0.0f) ? true : false;
    }

    template <class VectorType>
    inline VectorType calcRotatedVector(const VectorType &f_vec_v, const float f_angle_f32)
    {

        const float cos_f32 = std::cos(f_angle_f32);
        const float sin_f32 = std::sin(f_angle_f32);

        return VectorType((f_vec_v[0] * cos_f32) + (f_vec_v[1] * sin_f32),
                          (-f_vec_v[0] * sin_f32) + (f_vec_v[1] * cos_f32),
                          0.0f);
    }

    inline float calculateTriangleAreawithThreePoints(
        const Eigen::Vector3d &f_PointA_r, // not matched point(outside point)
        const Eigen::Vector3d &f_PointB_r, // default start point of psl,
        const Eigen::Vector3d &f_PointC_r) // default end point of psl
    {
        // f_PointB_r is the common point of two vectors
        Eigen::Vector3d l_vectorA(
            f_PointC_r[0] - f_PointB_r[0], // from PointB to PointC
            f_PointC_r[1] - f_PointB_r[1],
            0.f);

        Eigen::Vector3d l_vectorB(
            f_PointB_r[0] - f_PointA_r[0], // from PointA to PointB
            f_PointB_r[1] - f_PointA_r[1],
            0.f);

        // the angle from VectorA to VectorB is related to the negatvie or positive of area
        float crossProduct_f32 = calcCrossProd(l_vectorA, l_vectorB);

        return (float)(crossProduct_f32 / 2.0f);
    }

    //!----------------------------------------------------------------------------------------------------------------------------
    //! @brief calculates relative position of the intersection of two input lines:
    //! @return true when an intersection is found, otherwise false when two lines are colinear or parralel
    //! @param[in] f_supportVector1_vf32 a point on line 1
    //! @param[in] f_directionVector1_v direction vector of line 1
    //! @param[in] f_supportVector2_vf32 a point on line 2
    //! @param[in] f_directionVector2_v direction vector of line 2
    //! @param[in] f_computeRelPosInLine1_b : flag to compte relative pos. ratio with direction vector length w.r.t. line 1.
    //! @param[in] f_computeRelPosInLine2_b : flag to compte relative pos. ratio with direction vector length w.r.t. line 2.
    //! @param[out] f_intersectionRelativePosInLine1Out_rf32 : relative pos/ direction vec length w.r.t line 1
    //! @param[out] f_intersectionRelativePosInLine2Out_rf32 : relative pos/ direction vec length w.r.t line 2
    //! @note when compute relative position flag is set,
    //! the ratio of the intersection's relative pos. from the line start and its direction vector length is returned (by reference)
    //! @author  yij7szh
    //---------------------------------------------------------------------
    template <class Vector1Type, class Vector2Type>
    inline bool calcLinesIntersectionRelativePos(
        const Eigen::Vector3d &f_supportVector1_vf32,
        const Vector1Type &f_directionVector1_v,
        const Eigen::Vector3d &f_supportVector2_vf32,
        const Vector2Type &f_directionVector2_v,
        const bool f_computeRelPosInLine1_b,
        const bool f_computeRelPosInLine2_b,
        float &f_intersectionRelativePosInLine1Out_rf32,
        float &f_intersectionRelativePosInLine2Out_rf32)
    {
        const Eigen::Vector3d pMinusQ_vf32(
            (f_supportVector1_vf32[0] - f_supportVector2_vf32[0]),
            (f_supportVector1_vf32[1] - f_supportVector2_vf32[1]),
            0.f);

        const Eigen::Vector3d u_vf32(
            f_directionVector1_v[0], f_directionVector1_v[1], 0.0);

        const Eigen::Vector3d v_vf32(
            f_directionVector2_v[0], f_directionVector2_v[1], 0.f);

        float crossProdDenominator_f32 =
            calcCrossProd(v_vf32, u_vf32); //(v x u)

        bool validIntersectionFound_b = false;

        if (isZero(crossProdDenominator_f32))
        {
            // 2 lines are parallel or colinear
            validIntersectionFound_b = false;
            return validIntersectionFound_b;
        }
        else
        {
            validIntersectionFound_b = true;
        }

        // save b in case of computing relative position of intersection point in line2 flag is set
        if (true == f_computeRelPosInLine2_b)
        {
            float crossProdNumerator_f32 =
                calcCrossProd(pMinusQ_vf32, u_vf32); // (P-Q) x u

            f_intersectionRelativePosInLine2Out_rf32 = (float)(crossProdNumerator_f32 / crossProdDenominator_f32);
        }

        // save a in case of computing relative position of intersection point in line1 flag is set
        if (true == f_computeRelPosInLine1_b)
        {
            float crossProdNumerator_f32 =
                calcCrossProd(pMinusQ_vf32, v_vf32); // (P-Q) x v

            f_intersectionRelativePosInLine1Out_rf32 = (float)(crossProdNumerator_f32 / crossProdDenominator_f32);
        }

        return validIntersectionFound_b;
    }

    //!----------------------------------------------------------------------------------------------------------------------------
    //! @brief calculate/check for existence of an intersection of 2 lines with their expansion taken into account:
    //! @return true in these following cases:
    //!         +)  both lines are of infinite expansion and an intersection point exists.
    //!         ++) either or both lines are of finite expansion and
    //!             an intersection point exists and lies within the finite expansion line(s).
    //! ;false otherwise
    //! @param[in] f_supportVector1_vf32 : a point on line 1
    //! @param[in] f_directionVector1_v : direction vector of line 1
    //! @param[in] f_supportVector2_vf32 :a point on line 2
    //! @param[in] f_directionVector2_v : direction vector of line 2
    //! @param[out] f_outIntersect_rv : output intersection 3D point (pass by reference)
    //! @note The according flag to check if intersection point lies inside a line's expansion  must be set properly
    //! @author yij7szh
    //---------------------------------------------------------------------
    template <class Vector1Type, class Vector2Type>
    inline bool calcLinesIntersectionPoint(
        const Eigen::Vector3d &f_supportVector1_vf32,
        const Vector1Type &f_directionVector1_v,
        const Eigen::Vector3d &f_supportVector2_vf32,
        const Vector2Type &f_directionVector2_v,
        Eigen::Vector3d &f_outIntersect_rv)
    {
        const bool computeRelPosIntersectionInLine1_b = false;
        const bool computeRelPosIntersectionInLine2_b = true;

        float relPosIntersectionInLine1_f32(0.f);
        float relPosIntersectionInLine2_f32(0.f);

        const bool intersectionExist_b = calcLinesIntersectionRelativePos(
            f_supportVector1_vf32,
            f_directionVector1_v,
            f_supportVector2_vf32,
            f_directionVector2_v,
            computeRelPosIntersectionInLine1_b,
            computeRelPosIntersectionInLine2_b,
            relPosIntersectionInLine1_f32,
            relPosIntersectionInLine2_f32);

        bool validIntersectionFound_b(false);
        if (false == intersectionExist_b)
        {
            return false;
        }
        else
        {
            validIntersectionFound_b = true;
            // will still need to check if the intersection point is inside the expansion of the line(s)
            // upon the value of the input flags
        }

        f_outIntersect_rv[0] =
            f_supportVector2_vf32[0] + relPosIntersectionInLine2_f32 * f_directionVector2_v[0];
        f_outIntersect_rv[1] =
            f_supportVector2_vf32[1] + relPosIntersectionInLine2_f32 * f_directionVector2_v[1];

        return validIntersectionFound_b;
    }
}
#endif
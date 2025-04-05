#ifndef RSD_LINEMODEL_V1_HPP_INCLUDED
#define RSD_LINEMODEL_V1_HPP_INCLUDED

#include "line_types.hpp"

#include "line_math_function.hpp"

using namespace std;

namespace Fusion
{

    class CLineModel
    {

    using Ptr = std::shared_ptr<CLineModel>;
    public:
        CLineModel()
           : m_id_i8(0),
             m_isMatched_b(false),
             m_matchedID_i8(0),
             m_startPoint_f32(0,0,0),
             m_endPoint_f32(0,0,0),
             m_phi_f32(0.0f),
             m_sinPhi_f32(0.0f),
             m_cosPhi_f32(0.0f),
             m_unitVec_vf32(0,0,0),
             m_perpPointOfOrigin_vf32(0,0,0),
             m_tauMin_f32(0),
             m_tauMinSet_b(false),
             m_tauMax_f32(0),
             m_tauMaxSet_b(false),
             m_isUnitVectorInversed_b(false)
            {
                m_startPoint_f32 = Eigen::Vector3d(0,0,0);
            }

        CLineModel(const Eigen::Vector3d startPoint, const Eigen::Vector3d endPoint)
           : m_id_i8(0),
             m_isMatched_b(false),
             m_matchedID_i8(0),
             m_startPoint_f32(0,0,0),
             m_endPoint_f32(0,0,0),
             m_phi_f32(0.0f),
             m_sinPhi_f32(0.0f),
             m_cosPhi_f32(0.0f),
             m_unitVec_vf32(0,0,0),
             m_perpPointOfOrigin_vf32(0,0,0),
             m_tauMin_f32(0),
             m_tauMinSet_b(false),
             m_tauMax_f32(0),
             m_tauMaxSet_b(false),
             m_isUnitVectorInversed_b(false)
        {
            initWithTwoWorldPoints(startPoint,endPoint);
        }




        //signedDist_f32
        //            y    snegative
        //            |   \
        //            |  / \
        //  2nd       | /   \
        //__\_________|/_____\______x
        //   \       /|       \  
        //3rd \     / |        \  4.th
        //     \/\ /  |
        //      \ /   
        //  postive 

   
    //f_worldPointB_vf32 is the default start point
    //f_worldPointA_vf32 is the default end point
    inline void initWithTwoWorldPoints(
        const Eigen::Vector3d& f_worldPointB_vf32,
        const Eigen::Vector3d& f_worldPointA_vf32) 
        //std::int32_t& f_NumofInverseDuringProcess_i32)
    {
        int f_NumofInverseDuringProcess_i32(0);

        setStartPoint(f_worldPointB_vf32);
        setEndPoint(f_worldPointA_vf32);

        Eigen::Vector3d tempUnitVec_vf32(
            (f_worldPointA_vf32[0] - f_worldPointB_vf32[0]),
            (f_worldPointA_vf32[1] - f_worldPointB_vf32[1]),
            0.0f);

        tempUnitVec_vf32.normalize();

        if (isNegative(tempUnitVec_vf32[0]))
        {
            tempUnitVec_vf32 = -tempUnitVec_vf32;
            f_NumofInverseDuringProcess_i32++;
        }  //angle of tempUnitVec_vf32 is within (-90 degree, 90 degree)

        const float signedDist_f32(calcCrossProd(f_worldPointA_vf32, tempUnitVec_vf32));

        // calculate perpendicular point of origin through cross product
        m_perpPointOfOrigin_vf32[0] = (signedDist_f32 * tempUnitVec_vf32[1]);  //tempUnitVec_vf32.y() =  sin(UnitVector_angle)
        m_perpPointOfOrigin_vf32[1] = (-signedDist_f32 * tempUnitVec_vf32[0]); //tempUnitVec_vf32.x() =  cos(UnitVector_angle)
        m_perpPointOfOrigin_vf32[2] = 0.0f;

        const Eigen::Vector3d normVector_vf32(-tempUnitVec_vf32[1], tempUnitVec_vf32[0], 0.f);
        //angle(normVector) - angle(tempUnitVec_vf32) = 90, normVector is 90 forward tempUnitVec
        // calc angle phi between normal vector and x-axis
        //            y
        //            |   normVector_vf32(x=cos(phi), y=sin(phi))
        //            |  /
        //  2nd       | / phi=?  1.st
        //____________|/___________x
        //            |
        //  3rd       |          4.th
        //            |
        //

        // normVector_vf32 is on the upper half of cos,sin circle
        // PLD_ASSERT(vfc::notNegative(normVector_vf32.y())); //cause tempUnitVec_vf32.x() > 0

        // when arcos slope is steeper than arcsin -> use arcsin to have better angle and vice versa
        // for e.g, when x is near -1 or 1 (corr. angle pi, 0), arccos slope is almost vertical, a small error in x -> big error in arccos(x)
        // similarly when y is -1 or 1 (corr. angle -pi/2, pi/2), arcsin slope is almost vertical, a small error in y -> big error in arcsin(y)
        float phi_f32(0.f);
        if(std::fabs(normVector_vf32[0]) > std::fabs(normVector_vf32[1]))
        {
            phi_f32 = std::asin(normVector_vf32[1]);
            // if normVector_vf32 is in 2nd quarter of the cos,sin circle
            // return the reflex angle of arcsin
            if (isNegative(normVector_vf32[0]))  //under this case, the got phi value after the uppon process is not the correct one
            {                                         //M_PI -phi is not used to inverse the
                phi_f32 = (M_PI - phi_f32);        //reason: under this case, the true phi = M_PI - phi
            }
        }
        else
        {
            phi_f32 = std::acos(normVector_vf32[0]);
            //if x() is negative, now phi is already an obtuse angle
        }//now, 0< phi < 180

        // the computed angle must be in the upper half of the cos,sin circle
        //PLD_ASSERT(vfc::notNegative(phi_f32) && (phi_f32 <= G_M_PI_F32));

        // decide on which hyperplane the origin is with unitVectorFromPhi_vf32 as unit vector
        // if f_worldPointA_vf32 (x) unitVectorFromPhi_vf32 > 0, origin is on the left hyperplane, and vice versa
        
        Eigen::Vector3d unitVectorFromPhi_vf32(-std::sin(phi_f32), std::cos(phi_f32), 0.0f);
        f_NumofInverseDuringProcess_i32++; //why?because angle(unitVectorFromPhi_vf32) - angle(normVector_vf32) = angle(normVector_vf32) - angle(tempUnitVec_vf32) = 90
        const float signedDistFromPhi_f32(calcCrossProd(f_worldPointA_vf32, unitVectorFromPhi_vf32));
        if (isNegative(signedDistFromPhi_f32))
        {
            // if origin is on the right hyperplane of the cirected-line(unit vector) , we need to inverse unit vector by adding pi to phi
            phi_f32 += M_PI;
            unitVectorFromPhi_vf32 = -unitVectorFromPhi_vf32;
            f_NumofInverseDuringProcess_i32++;
        }

        if(1 == f_NumofInverseDuringProcess_i32%2)
        {
            setIsUnitVectorInversed(true);
        }
        else
        {
            setIsUnitVectorInversed(false);
        }
        

        // phi can get 2Pi. Shift phi to valid area
        shiftPhiToValidRange(phi_f32);

        // set phi, dist, unit vector
        m_phi_f32  = phi_f32;
        m_dist_f32 = std::fabs(signedDist_f32);
        float dotProd_f32(0.f);
        calcDotProd(unitVectorFromPhi_vf32, tempUnitVec_vf32, dotProd_f32);
        //PLD_ASSERT(vfc::isEqual(1.f, vfc::abs(dotProd_f32), 0.000001f));
        m_unitVec_vf32 = (isPositive(dotProd_f32) ? tempUnitVec_vf32 : Eigen::Vector3d(-tempUnitVec_vf32)); //m_unitVec_vf32 =unitVectorFromPhi_vf32

        m_sinPhi_f32 = -m_unitVec_vf32[0];
        m_cosPhi_f32 = m_unitVec_vf32[1];

        // project world points and set tau values (must be done after m_unitVec_vf32 is set)
        setTauValues(f_worldPointA_vf32, f_worldPointB_vf32);   
    }






    //f_worldPointB_vf32 is the default start point
    //f_worldPointA_vf32 is the default end point
    inline void initWithTwoWorldPoints(
        const Eigen::Vector3d& f_worldPointB_vf32,
        const Eigen::Vector3d& f_worldPointA_vf32,
        int& f_NumofInverseDuringProcess_i32)
    {
        f_NumofInverseDuringProcess_i32 = 0;

        setStartPoint(f_worldPointB_vf32);
        setEndPoint(f_worldPointA_vf32);

        Eigen::Vector3d tempUnitVec_vf32(
            (f_worldPointA_vf32[0] - f_worldPointB_vf32[0]),
            (f_worldPointA_vf32[1] - f_worldPointB_vf32[1]),
            0.0f);

        tempUnitVec_vf32.normalize();


        if (isNegative(tempUnitVec_vf32[0]))
        {
            tempUnitVec_vf32 = -tempUnitVec_vf32;
            f_NumofInverseDuringProcess_i32++;
        }  //angle of tempUnitVec_vf32 is within (-90 degree, 90 degree)

        const float signedDist_f32(calcCrossProd(f_worldPointA_vf32, tempUnitVec_vf32));

        // calculate perpendicular point of origin through cross product
        m_perpPointOfOrigin_vf32[0] = (signedDist_f32 * tempUnitVec_vf32[1]);  //tempUnitVec_vf32.y() =  sin(UnitVector_angle)
        m_perpPointOfOrigin_vf32[1] = (-signedDist_f32 * tempUnitVec_vf32[0]); //tempUnitVec_vf32.x() =  cos(UnitVector_angle)
        m_perpPointOfOrigin_vf32[2] = 0.0f;

        const Eigen::Vector3d normVector_vf32(-tempUnitVec_vf32[1], tempUnitVec_vf32[0], 0.f);

        float phi_f32(0.f);
        if(std::fabs(normVector_vf32[0]) > std::fabs(normVector_vf32[1]))
        {
            phi_f32 = std::asin(normVector_vf32[1]);
            // if normVector_vf32 is in 2nd quarter of the cos,sin circle
            // return the reflex angle of arcsin
            if (isNegative(normVector_vf32[0]))  //under this case, the got phi value after the uppon process is not the correct one
            {                                         //M_PI -phi is not used to inverse the
                phi_f32 = (M_PI - phi_f32);        //reason: under this case, the true phi = M_PI - phi
            }
        }
        else
        {
            phi_f32 = std::acos(normVector_vf32[0]);
            //if x() is negative, now phi is already an obtuse angle
        }//now, 0< phi < 180


        Eigen::Vector3d unitVectorFromPhi_vf32(-std::sin(phi_f32), std::cos(phi_f32), 0.0f);
        f_NumofInverseDuringProcess_i32++; //why?because angle(unitVectorFromPhi_vf32) - angle(normVector_vf32) = angle(normVector_vf32) - angle(tempUnitVec_vf32) = 90
        const float signedDistFromPhi_f32(calcCrossProd(f_worldPointA_vf32, unitVectorFromPhi_vf32));
        if (isNegative(signedDistFromPhi_f32))
        {
            // if origin is on the right hyperplane of the cirected-line(unit vector) , we need to inverse unit vector by adding pi to phi
            phi_f32 += M_PI;
            unitVectorFromPhi_vf32 = -unitVectorFromPhi_vf32;
            f_NumofInverseDuringProcess_i32++;
        }

        if(1 == f_NumofInverseDuringProcess_i32%2)
        {
            setIsUnitVectorInversed(true);
        }
        else
        {
            setIsUnitVectorInversed(false);
        }
        

        // phi can get 2Pi. Shift phi to valid area
        shiftPhiToValidRange(phi_f32);

        // set phi, dist, unit vector
        m_phi_f32  = phi_f32;
        m_dist_f32 = std::fabs(signedDist_f32);
        float dotProd_f32(0.f);
        calcDotProd(unitVectorFromPhi_vf32, tempUnitVec_vf32, dotProd_f32);
        //PLD_ASSERT(vfc::isEqual(1.f, vfc::abs(dotProd_f32), 0.000001f));
        m_unitVec_vf32 = (isPositive(dotProd_f32) ? tempUnitVec_vf32 : Eigen::Vector3d(-tempUnitVec_vf32)); //m_unitVec_vf32 =unitVectorFromPhi_vf32

        m_sinPhi_f32 = -m_unitVec_vf32[0];
        m_cosPhi_f32 = m_unitVec_vf32[1];

        // project world points and set tau values (must be done after m_unitVec_vf32 is set)
        setTauValues(f_worldPointA_vf32, f_worldPointB_vf32);



        
    }


    inline void exchangeTauminandTaumax()
    {
        float f_tau_f32 = m_tauMax_f32;
        m_tauMax_f32          = m_tauMin_f32;
        m_tauMin_f32          = f_tau_f32;

        bool f_isSet_b = m_tauMaxSet_b;
        m_tauMaxSet_b  = m_tauMinSet_b;
        m_tauMinSet_b  = f_isSet_b;
    }



    inline void setPhiAndDist(
        const rad_f32_t& f_phi_f32, 
        const metre_f32_t& f_dist_f32, 
        const bool f_checkValidityOfPhiAndDist_b)
    {
        // set phi
        m_phi_f32    = f_phi_f32;
        m_sinPhi_f32 = std::sin(f_phi_f32);
        m_cosPhi_f32 = std::cos(f_phi_f32);

        // set unit vector
        m_unitVec_vf32 = Eigen::Vector3d(-m_sinPhi_f32, m_cosPhi_f32, 0.f);

        //PLD_ASSERT2(isUnitVector(m_unitVec_vf32), "unit vector is not normalized");

        // set dist
        m_dist_f32 = f_dist_f32;

        // precalculate perpendicular point of origin for fast calculations
        m_perpPointOfOrigin_vf32[0] = (m_dist_f32 * m_cosPhi_f32);
        m_perpPointOfOrigin_vf32[1] = (m_dist_f32 * m_sinPhi_f32);
        m_perpPointOfOrigin_vf32[2] = 0.0f;

        // invalidate tau values because the old values do not fit to the new orientation
        m_tauMinSet_b           = false;
        m_tauMaxSet_b           = false;
        m_tauMin_f32            = metre_f32_t(0.f);
        m_tauMax_f32            = metre_f32_t(0.f);
    }


    inline void setPhiAndDistKeepTauWP(const rad_f32_t& f_phi_f32, const metre_f32_t& f_dist_f32)
    {
        rad_f32_t   phi_f32  = f_phi_f32;
        metre_f32_t dist_f32 = f_dist_f32;

        if(isNegative(dist_f32))
        {
            dist_f32 = -dist_f32;
            phi_f32 += M_PI;
            //exchangeTauminandTaumax();
            inverseIsUnitVectorInversed();
        }
        //else keep dist_f32, phi_f32 

        while(TwoPI <= phi_f32)  // [0, 2pi) includes 0 and not 2pi
        {
            phi_f32 -= TwoPI;
        }
        
        while (isNegative(phi_f32))
        {
            phi_f32 += TwoPI;
        }  

        // set phi
        m_phi_f32    = phi_f32;
        m_sinPhi_f32 = std::sin(m_phi_f32);
        m_cosPhi_f32 = std::cos(m_phi_f32);

        // set unit vector
        m_unitVec_vf32 = Eigen::Vector3d(-m_sinPhi_f32, m_cosPhi_f32, 0.0f);

        // set dist
        m_dist_f32   = dist_f32;

        // precalculate perpendicular point of origin for fast calculations
        m_perpPointOfOrigin_vf32[0] = (m_dist_f32 * m_cosPhi_f32);
        m_perpPointOfOrigin_vf32[1] = (m_dist_f32 * m_sinPhi_f32);
        m_perpPointOfOrigin_vf32[2] = (0.0f);

        // invalidate tau values because the old values do not fit to the new orientation
        // point position of taumin and taumax was not deleted cause they will be used in further step
        m_tauMinSet_b           = false;
        m_tauMaxSet_b           = false;
        m_tauMin_f32            = 0.0f;
        m_tauMax_f32            = 0.0f;
    }







    inline void setTauValues(const Eigen::Vector3d& f_worldEndPtA_vf32, const Eigen::Vector3d& f_worldEndPtB_vf32)
    {
        const float tauA_f32 = calcTauOfOrthProj(f_worldEndPtA_vf32);
        const float tauB_f32 = calcTauOfOrthProj(f_worldEndPtB_vf32);
        setTauMin(min(tauA_f32, tauB_f32));
        setTauMax(max(tauA_f32, tauB_f32));
    }




    inline void setTauMin(const float f_tauMin_f32)
    {
        m_tauMin_f32            = f_tauMin_f32;
        m_tauMinSet_b           = true;
    }

    inline void setTauMinAndWP(const float f_tauMin_f32)
    {
        m_tauMin_f32            = f_tauMin_f32;
        m_tauMinSet_b           = true;

        Eigen::Vector3d l_wpTauMin_f = calcWorldPointOnLine(m_tauMin_f32);
        if(false == m_isUnitVectorInversed_b)
        {
            m_startPoint_f32 = l_wpTauMin_f;
        }
        else
        {
            m_endPoint_f32 = l_wpTauMin_f;
        }
    }


    inline void setTauMax(const float f_tauMax_f32)
    {
        m_tauMax_f32            = f_tauMax_f32;
        m_tauMaxSet_b           = true;
    }

    inline void setTauMaxAndWP(const float f_tauMax_f32)
    {
        m_tauMax_f32            = f_tauMax_f32;
        m_tauMaxSet_b           = true;
        
        Eigen::Vector3d l_wpTauMax_f = calcWorldPointOnLine(m_tauMax_f32);
        if(false == m_isUnitVectorInversed_b)
        {
            m_endPoint_f32 = l_wpTauMax_f;
        }
        else
        {
            m_startPoint_f32 = l_wpTauMax_f;
        }
    }


    //---------------------------------------------------------------------
    //! @brief calculates orthogonal projection of point onto line described by unitVec, result is a tau value [m] (signed multiple of line unit vector)
    //! @param[in] f_point_vf32 : 3D-point
    //! @param[in] f_unitVec_vf32 : unit vector
    //! @return tau value [m] (signed multiple of line unit vector)
    //! @author  yinjie
    //---------------------------------------------------------------------
    inline float calcOrthPointProjOnLine(const Eigen::Vector3d& f_point_vf32, const Eigen::Vector3d& f_unitVec_vf32)
    {
        // PLD_REQUIRE(isValidVector(f_point_vf32));
        // PLD_REQUIRE(isUnitVector(f_unitVec_vf32));

        float dotProduct_f32(0.0f);
        calcDotProd(f_point_vf32, f_unitVec_vf32, dotProduct_f32);

        return dotProduct_f32;
    }

    inline float calcOrthPointProjOnLine(const Eigen::Vector3d& f_point_vf32, const Eigen::Vector3d& f_unitVec_vf32) const
    {
        // PLD_REQUIRE(isValidVector(f_point_vf32));
        // PLD_REQUIRE(isUnitVector(f_unitVec_vf32));

        float dotProduct_f32(0.0f);
        calcDotProd(f_point_vf32, f_unitVec_vf32, dotProduct_f32);

        return dotProduct_f32;
    }

    inline float calcTauOfOrthProj(const Eigen::Vector3d f_worldPoint_v3f32)
    {
        const float tau_f32 = calcOrthPointProjOnLine(f_worldPoint_v3f32, m_unitVec_vf32);
        return tau_f32;
    }

    
    inline float calcTauOfOrthProj(const Eigen::Vector3d f_worldPoint_v3f32) const
    {
        const float tau_f32 = calcOrthPointProjOnLine(f_worldPoint_v3f32, m_unitVec_vf32);
        return tau_f32;
    }

    


    inline float& getTauMin()
    {
        return m_tauMin_f32;
    }

    inline float getTauMin() const
    {
        return m_tauMin_f32;
    }
        
    inline bool& getIsTauMinSet()
    {
        return m_tauMinSet_b;
    }        
    
    inline float& getTauMax()
    {
        return m_tauMax_f32;
    }
    
    
    inline float getTauMax() const
    {
        return m_tauMax_f32;
    }

    inline bool& getIsTauMaxSet()
    {
        return m_tauMaxSet_b;
    }


    inline void shiftPhiToValidRange(float& f_phi_rf32) const
    {
        if (isNegative(f_phi_rf32))
        {
            f_phi_rf32 += 2*M_PI;
        }

        if (f_phi_rf32 >= 2*M_PI)
        {
            f_phi_rf32 -= 2*M_PI;
        }

    }


    inline void setStartPoint(const Eigen::Vector3d& f_startPoint_r)
    {
        m_startPoint_f32 = f_startPoint_r;
    }

    inline void setEndPoint(const Eigen::Vector3d& f_endPoint_r)
    {
        m_endPoint_f32 = f_endPoint_r;
    }

    inline void setIsUnitVectorInversed(const bool& f_isUnitVectorInversed_b)
    {
        m_isUnitVectorInversed_b = f_isUnitVectorInversed_b;
    }
    
    
    
    inline Eigen::Vector3d& getStartPoint()
    {
        return m_startPoint_f32;
    }

    inline float getStartPointX()
    {
        return m_startPoint_f32[0];
    }

    inline float getStartPointY()
    {
        return m_startPoint_f32[1];
    }

    inline Eigen::Vector3d& getEndPoint()
    {
        return m_endPoint_f32;
    }

    inline float getEndPointX()
    {
        return m_endPoint_f32[0];
    }

    inline float getEndPointY()
    {
        return m_endPoint_f32[1];
    }

    inline Eigen::Vector3d getStartPointConst() const
    {
        return m_startPoint_f32;
    }

    inline Eigen::Vector3d getEndPointConst() const
    {
        return m_endPoint_f32;
    }
    
    inline float& getPhi()
    {
        return m_phi_f32;
    }

    inline float& getSinPhi()
    {
        return m_sinPhi_f32;
    }

    inline float& getCosPhi()
    {
        return m_cosPhi_f32;
    }

    inline Eigen::Vector3d& getUnitVecRef()
    {
        return m_unitVec_vf32;
    }

    inline Eigen::Vector3d getUnitVec()
    {
        return m_unitVec_vf32;
    }
    
    inline float& getDist()
    {
        return m_dist_f32;
    }
    
    inline Eigen::Vector3d& getPerpPoint()
    {
        return m_perpPointOfOrigin_vf32;
    }



    inline bool& getIsUnitVectorInversed()
    {
        return m_isUnitVectorInversed_b;
    }

    inline void inverseIsUnitVectorInversed()
    {
        m_isUnitVectorInversed_b = !m_isUnitVectorInversed_b;
    }
    
    inline void setID(const int& f_id_ir)
    {
        m_id_i8 = f_id_ir;
    }

    inline int& getID()
    {
        return m_id_i8;
    }
    
    inline void setIsMatched(const bool& f_isMatched_b)
    {
        m_isMatched_b = f_isMatched_b;
    }

    inline bool& getIsMatched()
    {
        return m_isMatched_b;
    }

    inline void setIsParallelMatched(const bool& f_isParallelMatched_b)
    {
        m_isParallelMatched_b = f_isParallelMatched_b;
    }

    inline bool getIsParallelMatched()
    {
        return m_isParallelMatched_b;
    }
    
    inline void setMatchedID(const int f_matchedID_ir)
    {
        m_matchedID_i8 = f_matchedID_ir;
    }

    inline int& getMatchedID()
    {
        return m_matchedID_i8;
    }
    
    float calcOrthDistOfWorldPointSigned(const Eigen::Vector3d& f_worldPoint_v3f32) const
    {
        // signed distance of a point P from a line (A: point on the line and u: normalized direction vector) using the cross product
        // d = (P - A) x u
        const Eigen::Vector3d vec_v3f32(f_worldPoint_v3f32 - m_perpPointOfOrigin_vf32);
        const float   orthDist_f32 = calcCrossProd(vec_v3f32, m_unitVec_vf32);

        // Positive orthDist_f32 means that f_worldPoint_v3f32 is to the right of the line model.
        return orthDist_f32;
    }

    float calcOrthDistOfWorldPointAbs(const Eigen::Vector3d& f_worldPoint_v3f32) const
    {
        return std::fabs(calcOrthDistOfWorldPointSigned(f_worldPoint_v3f32));
    }

    inline Eigen::Vector3d calcWorldPointUsingTauValue(const float& f_tauValue_f) const
    {
        Eigen::Vector3d l_returnValue_f(m_perpPointOfOrigin_vf32[0] + f_tauValue_f*m_unitVec_vf32[0],
                                           m_perpPointOfOrigin_vf32[1] + f_tauValue_f*m_unitVec_vf32[1],
                                           0.0f);
        return l_returnValue_f;
    }

    inline float calcAbsTauDiffBetweenPointAndEndPoint(const Eigen::Vector3d f_point_r)
    {
        return std::fabs( calcTauOfOrthProj(f_point_r) - ((false == m_isUnitVectorInversed_b)?m_tauMax_f32:m_tauMin_f32));
    }

    inline float calcAbsTauDiffBetweenPointAndStartPoint(const Eigen::Vector3d f_point_r)
    {
        return std::fabs( ((false == m_isUnitVectorInversed_b)?m_tauMin_f32:m_tauMax_f32) - calcTauOfOrthProj(f_point_r));
    }

    inline Eigen::Vector3d calcWorldPointOnLine(const Eigen::Vector3d f_point_r)
    {
        //calc orthogonal projection of line2's start and end
        const float l_tauValue_f = calcTauOfOrthProj(f_point_r);
       
        return calcWorldPointUsingTauValue(l_tauValue_f);
    }

    bool isOverlap(
        const Eigen::Vector3d& f_line2EndPtA_vf32, 
        const Eigen::Vector3d& f_line2EndPtB_vf32, 
        float& f_overlap_f32,
        Eigen::Vector3d& f_ptASideOverlapEdgePoint_f32,
        Eigen::Vector3d& f_ptBSideOverlapEdgePoint_f32)
    {
        //calc orthogonal projection of line2's start and end
        const float tauLine2EndPtA_f32 = calcTauOfOrthProj(f_line2EndPtA_vf32);
        const float tauLine2EndPtB_f32 = calcTauOfOrthProj(f_line2EndPtB_vf32);

        float l_overlapTauMin_f = 0.0f;
        float l_overlapTauMax_f = 0.0f;

        bool l_returnValue_b = isOverlap(
            tauLine2EndPtA_f32, 
            tauLine2EndPtB_f32, 
            f_overlap_f32,
            l_overlapTauMin_f, 
            l_overlapTauMax_f);
        if(true == l_returnValue_b)
        {
            if(std::fabs(m_tauMin_f32 - l_overlapTauMin_f) > std::fabs(m_tauMin_f32 - l_overlapTauMax_f))
            {
                f_ptASideOverlapEdgePoint_f32 = calcWorldPointUsingTauValue(l_overlapTauMax_f);
                f_ptBSideOverlapEdgePoint_f32 = calcWorldPointUsingTauValue(l_overlapTauMin_f);
            }
            else
            {
                f_ptASideOverlapEdgePoint_f32 = calcWorldPointUsingTauValue(l_overlapTauMin_f);
                f_ptBSideOverlapEdgePoint_f32 = calcWorldPointUsingTauValue(l_overlapTauMax_f);
            }

        }
        //calc long distance
        return l_returnValue_b;
    }


    bool isOverlap(
        const float f_tauLine2EndPtA_f32, 
        const float f_tauLine2EndPtB_f32, 
        float& f_overlap_f32,
        float f_overlapTauMin_f,
        float f_overlapTauMax_f) const
    {
        bool l_isOverlap_b = true;
        //calc long distance


        //calc long distance
        const float tauLine2Min_f32 = min(f_tauLine2EndPtA_f32, f_tauLine2EndPtB_f32);
        const float tauLine2Max_f32 = max(f_tauLine2EndPtA_f32, f_tauLine2EndPtB_f32);
        float longDist_f32(0.0f);

        // line2 is "before" line1
        if (tauLine2Max_f32 <= m_tauMin_f32)
        {
            longDist_f32 = m_tauMin_f32 - tauLine2Max_f32;
            l_isOverlap_b = false;
        }
        // line2 is "after" line1
        else if (tauLine2Min_f32 >= m_tauMax_f32)
        {
            longDist_f32 = tauLine2Min_f32 - m_tauMax_f32;
            l_isOverlap_b = false;
        }
        else
        {
            //do nothing
        }


        if(l_isOverlap_b == true)
        {
            f_overlapTauMin_f = max(m_tauMin_f32, tauLine2Min_f32);

            f_overlapTauMax_f = min(m_tauMax_f32, tauLine2Max_f32);

            
            f_overlap_f32 = f_overlapTauMax_f - f_overlapTauMin_f;
        

            //if calculated overlapping is negative there is no overlapping!
            //avoid side case
            if (isNegative(f_overlap_f32))
            {
                f_overlap_f32 = 0.0f;
                l_isOverlap_b = false;
            }

        }

        return l_isOverlap_b;
    }











    bool isOverlap(const Eigen::Vector3d& f_line2EndPtA_vf32, const Eigen::Vector3d& f_line2EndPtB_vf32, float& f_overlap_f32)
    {
        //calc orthogonal projection of line2's start and end
        const float tauLine2EndPtA_f32 = calcTauOfOrthProj(f_line2EndPtA_vf32);
        const float tauLine2EndPtB_f32 = calcTauOfOrthProj(f_line2EndPtB_vf32);

        //calc long distance
        return isOverlap(tauLine2EndPtA_f32, tauLine2EndPtB_f32, f_overlap_f32);
    }

    bool isOverlap(const float f_tauLine2EndPtA_f32, const float f_tauLine2EndPtB_f32, float& f_overlap_f32) const
    {
        bool l_isOverlap_b = true;
        //calc long distance
        const float tauLine2Min_f32 = min(f_tauLine2EndPtA_f32, f_tauLine2EndPtB_f32);
        const float tauLine2Max_f32 = max(f_tauLine2EndPtA_f32, f_tauLine2EndPtB_f32);

        float longDist_f32(0.0f);

        // line2 is "before" line1
        if (tauLine2Max_f32 <= m_tauMin_f32)
        {
            longDist_f32 = m_tauMin_f32 - tauLine2Max_f32;
            l_isOverlap_b = false;
        }
        // line2 is "after" line1
        else if (tauLine2Min_f32 >= m_tauMax_f32)
        {
            longDist_f32 = tauLine2Min_f32 - m_tauMax_f32;
            l_isOverlap_b = false;
        }
        else
        {
            //do nothing
        }

        if(l_isOverlap_b == true)
        {
            f_overlap_f32 = (min(m_tauMax_f32, tauLine2Max_f32) - max(m_tauMin_f32, tauLine2Min_f32));

            //if calculated overlapping is negative there is no overlapping!
            //avoid side case
            if (isNegative(f_overlap_f32))
            {
                f_overlap_f32 = 0.0f;
                l_isOverlap_b = false;
            }

        }

        return l_isOverlap_b;
    }



    float getLength() const
    {
        return (m_tauMax_f32 - m_tauMin_f32);
    }


    inline void setOverlapRatio(const float& f_overlapRation_f32)
    {
        m_overlapRatio_f32 = f_overlapRation_f32;
    }

    inline float& getOverlapRatio()
    {
        return m_overlapRatio_f32;
    }

    inline float getUnitVectorsAngle() //in Radian
    {
        float l_returnAngle_f(0);

        float l_deltaX_f = m_startPoint_f32[0] - m_endPoint_f32[0];
        float l_deltaY_f = m_startPoint_f32[1] - m_endPoint_f32[1];

        if(isZero(l_deltaX_f))
        {
            l_returnAngle_f = (double)M_PI/2;
        }
        else
        {
            l_returnAngle_f = std::atan((double)l_deltaY_f/l_deltaX_f);
        }


        return l_returnAngle_f;
    }

    inline Eigen::Vector3d calcWorldPointOnLine(const float& f_tau_f32) const
    {
        const Eigen::Vector3d worldPoint_v3f32(
            m_perpPointOfOrigin_vf32[0] + (m_unitVec_vf32[0] * f_tau_f32),
            m_perpPointOfOrigin_vf32[1] + (m_unitVec_vf32[1] * f_tau_f32),
            0.f);
        return worldPoint_v3f32;
    }

    inline Eigen::Vector3d getDefaultUnitVec()
    {
        Eigen::Vector3d l_returnVec_f;
        if(true == m_isUnitVectorInversed_b)
        {
            l_returnVec_f = -m_unitVec_vf32;
        }
        else
        {
            l_returnVec_f = m_unitVec_vf32;
        }
        return l_returnVec_f;
    }

    inline float getStartPointTau()
    {
        float l_returnValue_f(0.0f);
        if(true == m_isUnitVectorInversed_b)
        {
            l_returnValue_f = m_tauMax_f32;
        }
        else
        {
            l_returnValue_f = m_tauMin_f32;
        }
        return l_returnValue_f;
    }

    inline float getEndPointTau()
    {
        float l_returnValue_f(0.0f);
        if(true == m_isUnitVectorInversed_b)
        {
            l_returnValue_f = m_tauMin_f32;
        }
        else
        {
            l_returnValue_f = m_tauMax_f32;
        }
        return l_returnValue_f;
    }


    void setPhiAndDist(const rad_f32_t& f_phi_f32, const metre_f32_t& f_dist_f32)
    {
        // set phi
        m_phi_f32    = f_phi_f32;
        m_sinPhi_f32 = std::sin(f_phi_f32);
        m_cosPhi_f32 = std::cos(f_phi_f32);

        // set unit vector
        m_unitVec_vf32 = Eigen::Vector3d(-m_sinPhi_f32, m_cosPhi_f32, 0.f);

        // set dist
        m_dist_f32 = f_dist_f32;

        // precalculate perpendicular point of origin for fast calculations
        m_perpPointOfOrigin_vf32[0] = (m_dist_f32 * m_cosPhi_f32);
        m_perpPointOfOrigin_vf32[1] = (m_dist_f32 * m_sinPhi_f32);
        m_perpPointOfOrigin_vf32[2] = 0.0f;

        // invalidate tau values because the old values do not fit to the new orientation
        m_tauMinSet_b           = false;
        m_tauMaxSet_b           = false;
        m_tauMin_f32            = 0.0f;
        m_tauMax_f32            = 0.0f;
    }

    bool isPointOnLeftSideOfUnitVector(
        const Eigen::Vector3d f_point_f) const 
    {
        bool l_returnValue_b(false);
        Eigen::Vector3d l_vectorA_f(f_point_f[0] - m_startPoint_f32[0], f_point_f[1] - m_startPoint_f32[1], 0.0f);
        Eigen::Vector3d l_vectorB_f(m_endPoint_f32[0] - m_startPoint_f32[0], m_endPoint_f32[1] - m_startPoint_f32[1], 0.0f);
        
        if(Fusion::isPositive(Fusion::calcCrossProd(l_vectorA_f, l_vectorB_f)))
        {
            l_returnValue_b = false;
        }
        else
        {
            l_returnValue_b = true;
        }

        return l_returnValue_b;
    }

    bool isPointOnRightSideOfUnitVector(const Eigen::Vector3d f_point_f) const
    {
        return !(isPointOnLeftSideOfUnitVector(f_point_f));
    }


    private:

        int m_id_i8;

        bool m_isMatched_b;

        bool m_isParallelMatched_b;

        int m_matchedID_i8;
        
        Eigen::Vector3d m_startPoint_f32; //default start point

        Eigen::Vector3d m_endPoint_f32; //default end point

        //! Angle of the line model phi. Counterclockwise starting with DIN70k x-axis. [rad]
        float m_phi_f32;

        //! Sine of m_phi_f32 precalculated for performance reasons
        float m_sinPhi_f32;

        //! Cosine of m_phi_f32 precalculated for performance reasons
        float m_cosPhi_f32;

        //! Unit vector (normalized direction vector, origin always to the left)
        Eigen::Vector3d m_unitVec_vf32;

        //signed distance should be positive,which means the origin point of local vehicle coordinate should on the right point of vector
        //! Min distance of the line model to the origin [m]
        float m_dist_f32;

        //! Point on line model with minimal distance to origin [m], "Lotfu?punkt", precalculated for performance reasons
        Eigen::Vector3d m_perpPointOfOrigin_vf32;

        //! tau value describing the start of the line in direction of the unit vector with valid flag. The world pos
        //! will be precalculated for performance reasons
        float m_tauMin_f32;
        bool m_tauMinSet_b;
  
        //! tau value describing the end of the line in direction of the unit vector with valid flag. The world pos
        //! will be precalculated for performance reasons
        float m_tauMax_f32;
        bool m_tauMaxSet_b;

        float m_overlapRatio_f32;

        bool m_isUnitVectorInversed_b;
    };

} // namespace rsd

#endif

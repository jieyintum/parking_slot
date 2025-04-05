//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//=============================================================================
//  P R O J E C T   I N F O R M A T I O N
//-----------------------------------------------------------------------------
//     Projectname: E2
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: Jie Yin
//  Department: ADC
//=============================================================================
/// @file  rsd_type.hpp
/// @brief public interface of the PSF runnable
//=============================================================================

#ifndef _RSD_TYPE_INTERFACE_V1_HEADER_
#define _RSD_TYPE_INTERFACE_V1_HEADER_

#include <vector>
#include <string>
#include <map>
#include <algorithm>

#include <math.h>
#include <stdio.h>

#include <math.h>

#include <array>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


//#include <boost/array>

// #include "rsd_common_params.hpp"

using namespace std;

#define DegreePerPI 180
#define TwoPI 2*M_PI  // 6.2831853072

namespace Fusion {

    //typedef boost::array<float, 3> metre_3_f32_t;
    typedef Eigen::Vector3d metre_3_f32_t;
    typedef Eigen::Vector3d vector_3_f32_t;
    typedef Eigen::Vector2f vector_2_f64_t;

    typedef Eigen::Matrix3f matrix_33_f32_t;

    typedef Eigen::Matrix2f matrix_22_f64_t;
    
    typedef float square_metre_f32_t;

    typedef std::array<metre_3_f32_t, 4> CPointsList;

    typedef float metre_f32_t;
    
    typedef float rad_f32_t;

}
#endif // PLDTYPES_HPP



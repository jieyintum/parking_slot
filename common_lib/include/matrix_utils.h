#ifndef PP_CEM_MATRIX_UTILS_H_
#define PP_CEM_MATRIX_UTILS_H_

#include <Eigen/Eigen>

namespace PPcem {
/* 
 * 函数名： Vec2SkewSymetricMatrix
 * 功能：向量转反对称矩阵，常用于李代数运算
 * 输入：三维向量，即 Eigen::Vector3d(f)
 * 输出：反对称矩阵
*/
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3U, 3U> Vec2SkewSymetricMatrix(const Eigen::MatrixBase<Derived>& vec)
{
    using Scalar_t = typename Derived::Scalar;
    Eigen::Matrix<Scalar_t, 3U, 3U> ans;
    ans << Scalar_t(0.0), -vec.z(), vec.y(),
           vec.z(), Scalar_t(0.0), -vec.x(),
           -vec.y(), vec.x(), Scalar_t(0.0);
    return ans;
}

/* 
 * 函数名： Eular2RotationMatrixFxyz
 * 功能：欧拉角转为旋转矩阵，绕固定轴如map坐标系旋转，先绕固定轴的x旋转roll，再绕固定轴y旋转pitch，最后绕固定轴z旋转yaw
 * 输入：三维向量，即 Eigen::Vector3d(f), 其中x表示 roll， y表示pitch，z表示yaw，单位rad。
 * 输出：旋转矩阵 R = Rz * Ry * Rx
*/
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3U, 3U> Eular2RotationMatrixFxyz(const Eigen::MatrixBase<Derived>& rpy)
{
    using Scalar_t = typename Derived::Scalar;

    Eigen::Matrix<Scalar_t, 3, 3> yawAngle;
    yawAngle << cos(rpy.z()), -sin(rpy.z()), 0.,
                sin(rpy.z()), cos(rpy.z()), 0.,
                0., 0., 1.;

    Eigen::Matrix<Scalar_t, 3, 3> pitchAngle;
    pitchAngle << cos(rpy.y()), 0., sin(rpy.y()),
                  0., 1., 0.,
                  -sin(rpy.y()), 0., cos(rpy.y());

    Eigen::Matrix<Scalar_t, 3, 3> rollAngle;
    rollAngle << 1., 0., 0.,
                 0., cos(rpy.x()), -sin(rpy.x()),
                 0., sin(rpy.x()), cos(rpy.x());

    return (yawAngle * pitchAngle * rollAngle);
}

/* 
 * 函数名： Eular2QuaternionFxyz
 * 功能：欧拉角转为四元数，实际先转了旋转矩阵，Eigen内部有自己转换逻辑，参见Eular2RotationMatrix。
 * 输入：三维向量，即 Eigen::Vector3d(f), 其中x表示 roll， y表示pitch，z表示yaw，单位rad。
 * 输出：四元数 Eigen::Quaterniond(f)
*/
template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> Eular2QuaternionFxyz(const Eigen::MatrixBase<Derived>& rpy)
{
    Eigen::Quaternion<typename Derived::Scalar> quat;
    quat = Eular2RotationMatrixFxyz(rpy);
    return quat;
}

/* 
 * 函数名： Quaternion2EularFxyz
 * 功能：四元数转欧拉角，旋转方式同Eular2RotationMatrix
 * 输入：四元数，一般为Eigen::Quaterniond(f)
 * 输出：欧拉角，三维向量，其中x表示roll，y表示pitch，z表示yaw，单位rad
*/
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3U, 1U> Quaternion2EularFxyz(const Eigen::QuaternionBase<Derived>& quat)
{
    using Scalar_t = typename Derived::Scalar;
    Eigen::Matrix<Scalar_t, 3U, 1U> rpy;
    rpy.x() = std::atan2(2. * (quat.w() * quat.x() + quat.y() * quat.z()), 1. - 2. * (quat.x() * quat.x() + quat.y() * quat.y()));
    rpy.y() = std::asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
    rpy.z() = std::atan2(2. * (quat.w() * quat.z() + quat.x() * quat.y()), 1. - 2. * (quat.y() * quat.y() + quat.z() * quat.z()));

    return rpy;
}
}

#endif

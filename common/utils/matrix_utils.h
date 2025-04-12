#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <Eigen/Eigen>

namespace Fusion
{
    /*
     * 函数名： Vec2SkewSymetricMatrix
     * 功能：向量转反对称矩阵，常用于李代数运算
     * 输入：三维向量，即 Eigen::Vector3d(f)
     * 输出：反对称矩阵
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3U, 3U> Vec2SkewSymetricMatrix(const Eigen::MatrixBase<Derived> &vec)
    {
        using Scalar_t = typename Derived::Scalar;
        Eigen::Matrix<Scalar_t, 3U, 3U> ans;
        ans << Scalar_t(0.0), -vec.z(), vec.y(),
            vec.z(), Scalar_t(0.0), -vec.x(),
            -vec.y(), vec.x(), Scalar_t(0.0);
        return ans;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> Angular2DeltaQ(const Eigen::MatrixBase<Derived> &theta)
    {

        using Scalar_t = typename Derived::Scalar;
        Eigen::AngleAxis<Scalar_t> rotationVec(theta.norm(), theta.normalized());

        return Eigen::Quaternion<Scalar_t>(rotationVec);
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> Positify(const Eigen::QuaternionBase<Derived> &q)
    {
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4U, 4U> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = Positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4U, 4U> ans;
        ans(0U, 0U) = qq.w();
        ans.template block<1U, 3U>(0U, 1U) = -qq.vec().transpose();
        ans.template block<3U, 1U>(1U, 0U) = qq.vec();
        ans.template block<3U, 3U>(1U, 1U) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3U, 3U>::Identity() +
                                             Vec2SkewSymetricMatrix(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4U, 4U> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = Positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4U, 4U> ans;
        ans(0U, 0U) = pp.w();
        ans.template block<1U, 3U>(0U, 1U) = -pp.vec().transpose();
        ans.template block<3U, 1U>(1U, 0U) = pp.vec();
        ans.template block<3U, 3U>(1U, 1U) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3U, 3U>::Identity() -
                                             Vec2SkewSymetricMatrix(pp.vec());
        return ans;
    }

    /*
     * 函数名： Eular2RotationMatrix
     * 功能：欧拉角转为旋转矩阵，绕固定轴如map坐标系旋转，先绕固定轴的x旋转roll，再绕固定轴y旋转pitch，最后绕固定轴z旋转yaw
     * 输入：三维向量，即 Eigen::Vector3d(f), 其中x表示 roll， y表示pitch，z表示yaw，单位rad。
     * 输出：旋转矩阵 R = Rz * Ry * Rx
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3U, 3U> Eular2RotationMatrix(const Eigen::MatrixBase<Derived> &rpy)
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
     * 函数名： Eular2Quaternion
     * 功能：欧拉角转为四元数，实际先转了旋转矩阵，Eigen内部有自己转换逻辑，参见Eular2RotationMatrix。
     * 输入：三维向量，即 Eigen::Vector3d(f), 其中x表示 roll， y表示pitch，z表示yaw，单位rad。
     * 输出：四元数 Eigen::Quaterniond(f)
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> Eular2Quaternion(const Eigen::MatrixBase<Derived> &rpy)
    {
        Eigen::Quaternion<typename Derived::Scalar> quat;
        const auto rot = Eular2RotationMatrix(rpy);
        quat = rot;
        return quat;
    }

    /*
     * 函数名： Quaternion2Eular
     * 功能：四元数转欧拉角，旋转方式同Eular2RotationMatrix
     * 输入：四元数，一般为Eigen::Quaterniond(f)
     * 输出：欧拉角，三维向量，其中x表示roll，y表示pitch，z表示yaw，单位rad
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3U, 1U> Quaternion2Eular(const Eigen::QuaternionBase<Derived> &quat)
    {
        using Scalar_t = typename Derived::Scalar;
        Eigen::Matrix<Scalar_t, 3U, 1U> rpy;
        rpy.x() = std::atan2(2. * (quat.w() * quat.x() + quat.y() * quat.z()), 1. - 2. * (quat.x() * quat.x() + quat.y() * quat.y()));
        rpy.y() = std::asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
        rpy.z() = std::atan2(2. * (quat.w() * quat.z() + quat.x() * quat.y()), 1. - 2. * (quat.y() * quat.y() + quat.z() * quat.z()));

        return rpy;
    }

    /*
     * 函数名： TwoVec2Quat
     * 功能：求取V1向量旋转到V2向量的四元数
     * 输入：向量v1, 向量v2
     * 输出：四元数
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> TwoVec2Quat(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2)
    {
        using Scalar_t = typename Derived::Scalar;
        Eigen::Quaternion<Scalar_t> quat;
        quat.setFromTwoVectors(v1, v2);
        return quat;
    }

    template <typename T>
    static T WarpAngle(const T input)
    {
        const T dPi = 2.0 * M_PI;
        T output = input;
        while (output > M_PI)
        {
            output -= dPi;
        }
        while (output < -M_PI)
        {
            output += dPi;
        }

        return output;
    }

}

#endif
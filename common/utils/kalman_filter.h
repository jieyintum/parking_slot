#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <cstdint>
#include <Eigen/Eigen>

namespace Fusion
{
    template <typename T, int stateSize>
    class KalmanFilter
    {
    public:
        KalmanFilter()
        {
            unitMatrix.setIdentity();
        }

        ~KalmanFilter() = default;

        void SetInitialState(const Eigen::Matrix<T, stateSize, 1U> &state,
                             const Eigen::Matrix<T, stateSize, stateSize> &covariance)
        {
            x_ = state;
            p_ = covariance;
            pInit_ = covariance;
        }

        void Predict(const Eigen::Matrix<T, stateSize, stateSize> &transferMatrix,
                     const Eigen::Matrix<T, stateSize, stateSize> &noiseMatrix)
        {
            x_ = transferMatrix * x_;
            p_ = transferMatrix * p_ * transferMatrix.transpose() + noiseMatrix;
        }

        template <int controlSize>
        void Predict(const Eigen::Matrix<T, stateSize, stateSize> &transferMatrix,
                     const Eigen::Matrix<T, stateSize, stateSize> &noiseMatrix,
                     const Eigen::Matrix<T, stateSize, controlSize> &controlMatrix,
                     const Eigen::Matrix<T, controlSize, 1U> &controlInput)
        {
            x_ = transferMatrix * x_ + controlMatrix * controlInput;
            p_ = transferMatrix * p_ * transferMatrix.transpose() + noiseMatrix;
        }

        template <int measureSize>
        void Update(const Eigen::Matrix<T, measureSize, 1U> &measurement,
                    const Eigen::Matrix<T, measureSize, 1U> &estimated,
                    const Eigen::Matrix<T, measureSize, stateSize> &h,
                    const Eigen::Matrix<T, measureSize, measureSize> &measureCov)
        {
            const Eigen::Matrix<T, measureSize, measureSize> chiqure = ComputeChiqureMatrix(h);
            const Eigen::Matrix<T, stateSize, measureSize> k = p_ * h.transpose() * (chiqure + measureCov).inverse();
            x_ = x_ + k * (measurement - estimated);
            p_ = (unitMatrix - k * h) * p_;
        }

        template <int measureSize>
        void Update(const Eigen::Matrix<T, measureSize, 1U> &measurement,
                    const Eigen::Matrix<T, measureSize, stateSize> &h,
                    const Eigen::Matrix<T, measureSize, measureSize> &measureCov)
        {
            const Eigen::Matrix<T, measureSize, measureSize> chiqure = ComputeChiqureMatrix(h);
            const Eigen::Matrix<T, stateSize, measureSize> k = p_ * h.transpose() * (chiqure + measureCov).inverse();
            x_ = x_ + k * (measurement - h * x_);
            p_ = (unitMatrix - k * h) * p_;
        }

        template <int measureSize>
        const Eigen::Matrix<T, measureSize, measureSize> ComputeChiqureMatrix(const Eigen::Matrix<T, measureSize, stateSize> &h)
        {
            return h * p_ * h.transpose();
        }

        template <int measureSize>
        void Update(const Eigen::Matrix<T, measureSize, 1U> &measurement,
                    const Eigen::Matrix<T, measureSize, stateSize> &h,
                    const Eigen::Matrix<T, measureSize, measureSize> &measureCov,
                    const Eigen::Matrix<T, measureSize, measureSize> &chiqure)
        {
            const Eigen::Matrix<T, stateSize, measureSize> k = p_ * h.transpose() * (chiqure + measureCov).inverse();
            x_ = x_ + k * (measurement - h * x_);
            p_ = (unitMatrix - k * h) * p_;
        }

        inline void ResetState()
        {
            x_.setZero();
        }

        inline void ResetState(const size_t start, const size_t num)
        {
            for (size_t i = start; i < num; ++i)
            {
                x_(i, 0U) = 0;
            }
        }

        inline void Reset()
        {
            x_.setZero();
            p_ = pInit_;
        }

        inline const Eigen::Matrix<T, stateSize, 1U> &GetState() const
        {
            return x_;
        }

        inline const Eigen::Matrix<T, stateSize, stateSize> &GetCovariance() const
        {
            return p_;
        }

    private:
        Eigen::Matrix<T, stateSize, stateSize> unitMatrix;
        Eigen::Matrix<T, stateSize, 1U> x_;        // state
        Eigen::Matrix<T, stateSize, stateSize> p_; // covariance matrix
        Eigen::Matrix<T, stateSize, stateSize> pInit_;
    };
}
#endif
#include "pld_fusion/pld_frame.h"

#include "utils/matrix_utils.h"

#include <iostream>

namespace Fusion
{

    enum EStateParams : int
    {
        PHI = 0,
        DIST = 1
    };

    //---------------------------------------------------------------------
    //! @brief computes the Cholesky system noise matrix according to ego motion
    //! @param[in]  f_deltaDist_f32                   ego motion's translation distance
    //! @param[in]  f_deltaPhi_f32                    ego motion's rotation angle
    //! @param[in]  f_minCholPhiSystemNoise_f64       minimum cholesky system noise for phi
    //! @param[in]  f_minCholDistSystemNoise_f64      minimum cholesky system noise for dist
    //! @param[in]  f_factorSystemNoiseDeltaDist_f64  factor for the calculating of the system noise for the line distance based on the delta distance
    //! @param[out] f_choleskySystemNoiseMatrix_rmf64 computed system noise's Cholesky factor based on ego motion
    //! @author yij7szh
    //---------------------------------------------------------------------
    inline void computeCholeskySystemNoiseMatrix(
        const float f_deltaDist_f32,
        const float &f_deltaPhi_f32,
        Eigen::Matrix2d &f_choleskySystemNoiseMatrix_rmf64)
    {

        Eigen::Vector2d g_minCholSystemNoiseCarMoving_v2f64(0.0014, 0.002);
        float minCholPhiSystemNoise = g_minCholSystemNoiseCarMoving_v2f64[EStateParams::PHI];
        float minCholDistSystemNoise = g_minCholSystemNoiseCarMoving_v2f64[EStateParams::DIST];
        float factorSystemNoiseDeltaDist = 0.005;

        //! compute Cholesky factor of System noise by a factor of ego-motion and clips to min. noise when it is too small.
        f_choleskySystemNoiseMatrix_rmf64(EStateParams::PHI, EStateParams::PHI) =
            (double)std::max(minCholPhiSystemNoise, f_deltaPhi_f32);

        f_choleskySystemNoiseMatrix_rmf64(EStateParams::DIST, EStateParams::DIST) =
            (double)std::max(minCholDistSystemNoise, (factorSystemNoiseDeltaDist * f_deltaPhi_f32));
    }

    inline void predictStateVector(
        const Eigen::Vector3d &f_vehicleTranslation_vf32,
        const float &f_deltaYawAngle_f32,
        Fusion::CTrackLine::Ptr &line)
    {
        // predict distance
        // the deltaDist is based on the current origin
        const float deltaDist_f32 =
            calcCrossProd(f_vehicleTranslation_vf32, line->getUnitVec());

        // update model with predicted state variables
        line->setPhiAndDist(
            (line->getPhi() - f_deltaYawAngle_f32),
            (line->getDist() - deltaDist_f32));

        // system noise is factor of ego-motion:
        Eigen::Matrix2d choleskySystemNoise_mf64;
        choleskySystemNoise_mf64(EStateParams::DIST, EStateParams::PHI) = 0.0;
        choleskySystemNoise_mf64(EStateParams::PHI, EStateParams::DIST) = 0.0;
        computeCholeskySystemNoiseMatrix(
            deltaDist_f32,
            f_deltaYawAngle_f32,
            choleskySystemNoise_mf64);

        // set system noise for current frame
        line->m_choleskySystemNoiseMatrix_mf64 = choleskySystemNoise_mf64;
    }

    inline void predictLineModel(
        const Eigen::Vector3d &f_vehicleTranslation_vf32,
        const float &f_deltaYawAngle_f32,
        Fusion::CTrackLine::Ptr &line)
    {
        predictStateVector(
            f_vehicleTranslation_vf32,
            f_deltaYawAngle_f32,
            line);
    }

    //---------------------------------------------------------------------
    //! @brief calculates the transition derivative matrix according to ego motion
    //! @param[in]  f_egoShiftLat_f32 ego motion's translation in y (dy)
    //! @param[in]  f_egoShiftLong_f32 ego motion's translation in x (dx)
    //! @param[in]  f_sinPhi_f32 sin phi before prediction
    //! @param[in]  f_cosPhi_f32 cos phi before prediction
    //! @param[out] f_transitionDerivativeA_rmf64 calculated transition derivative matrix A of transition function on ego motion (output-as-reference)
    //! @author yij7szh
    //---------------------------------------------------------------------
    inline void calcTransitionDerivativeA(
        const float f_egoShiftLat_f32,
        const float f_egoShiftLong_f32,
        const float f_sinPhi_f32,
        const float f_cosPhi_f32,
        Eigen::Matrix2d &f_transitionDerivativeA_rmf64)
    {
        // dx = f_egoShiftLong_f32
        // dy = f_egoShiftLat_f32
        //  identity() is not used as vfc 64 bit operation is within nrc2 not supported.
        f_transitionDerivativeA_rmf64(0, 0) = 1.0;
        f_transitionDerivativeA_rmf64(0, 1) = 0.0;
        f_transitionDerivativeA_rmf64(1, 0) = static_cast<double>(
            (f_egoShiftLong_f32 * f_sinPhi_f32) - (f_egoShiftLat_f32 * f_cosPhi_f32));
        f_transitionDerivativeA_rmf64(1, 1) = 1.0;
    }

    //---------------------------------------------------------------------
    //! @brief updates Cholesky factors of state covariance matrix by SchmidtHouseholder triangulation
    //! Grewal and Andrews: Kalman Filtering, 2nd Edition, 2001, 6.5.1 Carlson-Schmidt Square-Root Filtering
    //! @param[in]      f_transitionDerivativeA_mf64    : transition derivative ,i.e. partial derivative of prediction function on ego motion
    //! @param[in]      f_systemNoise_mf64              : Cholesky factor of system noise (i.e. noise from ego motion)
    //! @param[in,out]  f_stateCovMatrix_rmf64          : input Cholesky factor of previous state covariance matrix and output Cholesky factor of predicted state covariance matrix
    //! @author yij7szh
    //---------------------------------------------------------------------
    //! @deviation NRCS2_030, QACPP-4.3.0-6041 "Avoid functions with a high static program path count."
    //! Kalman update textbook implementation with high static program count should not be refactored to keep comparability.
    inline void schmidtHouseholderTemporalUpdate( // PRQA S 6041
        const Eigen::Matrix2d &f_transitionDerivativeA_mf64,
        const Eigen::Matrix2d &f_cholSystemNoise_mf64,
        Eigen::Matrix2d &f_cholStateCovMatrix_rvf64)
    {
        //! (use in pair with Carlson's Fast Triangularization)
        //! Schmidt-Householder Temporal Update (Table 6.14 - Kalman Filtering Theory and Practice Using Matlab, 3rd Edition)
        //!
        //! Cholesky factorize the following matrix:
        //! Cp(-) Cp(-)^t = phi Cp(+) Cp(+)^t phi^t + GCq Cq^tG^t
        //! where: phi Cp(+) = A * Cp(+); A : transition derivative & Cp(+) : input Cholesky factor of previous state covariance
        //! and GCq = B: input Cholesky factor of system noise.
        //!
        //! The algorithm outputs Cholesky factor of predicted state covariance Cp(-).

        // require Cholesky factor of state covariance is an upper triangular matrix
        // PLD_REQUIRE(rsd::isEqual(0.0, f_cholStateCovMatrix_rvf64(1, 0), 0.00001));

        // // require Cholesky factor of system noise is a diagonal matrix w. positive members
        // PLD_REQUIRE(rsd::isEqual(0.0, f_cholSystemNoise_mf64(1, 0), 0.00001));
        // PLD_REQUIRE(rsd::isEqual(0.0, f_cholSystemNoise_mf64(0, 1), 0.00001));
        // PLD_REQUIRE(rsd::isPositive(f_cholSystemNoise_mf64(0, 0)));
        // PLD_REQUIRE(rsd::isPositive(f_cholSystemNoise_mf64(1, 1)));

        // input
        // A : partial derivation of prediction function (a.k.a transition derivative)
        const Eigen::Matrix2d &A_mf64 = f_transitionDerivativeA_mf64;

        // B : system noise Cholesky factor (GCq)
        Eigen::Matrix2d B_mf64 = f_cholSystemNoise_mf64;

        // input/output
        Eigen::Matrix2d &C_rmf64(f_cholStateCovMatrix_rvf64); // a reference

        // init local variables
        const int dimensionCount_i32 = 2;
        float sigma_f64 = 0.0;
        float alpha_f64 = 0.0;
        float beta_f64 = 0.0;
        Eigen::Vector2d v_vf64(0, 0);
        Eigen::Vector2d w_vf64(0, 0);

        //! A * Cp(+)
        Eigen::Matrix2d tempC_mf64;
        for (int i = 0; i < dimensionCount_i32; ++i)
        {
            for (int j = 0; j < dimensionCount_i32; ++j)
            {
                beta_f64 = 0.0;
                for (int k = 0; k <= j; k++) // Cp(+) is an upper triangular matrix
                {
                    beta_f64 += (A_mf64(i, k) * C_rmf64(k, j));
                }
                tempC_mf64(i, j) = beta_f64;
            }
        }
        C_rmf64 = tempC_mf64;

        // Householder triangularization:
        for (int k = dimensionCount_i32 - 1; k >= 0; k--)
        {
            sigma_f64 = 0.0;
            for (int j = 0; j < dimensionCount_i32; j++)
            {
                sigma_f64 += (B_mf64(k, j) * B_mf64(k, j));
            }
            for (int j = 0; j <= k; j++)
            {
                sigma_f64 += (C_rmf64(k, j) * C_rmf64(k, j));
            }
            // sigma is for sure positive
            // because B_mf64 = f_cholSystemNoise_mf64 has at least non-zero element in each row    (1)
            // see computeCholeskySystemNoiseMatrix()
            // sigma_f64 must be greater or equal to g_minCholSystemNoise_v2f64[k]^2
            // PLD_ASSERT(rsd::isPositive(sigma_f64));
            alpha_f64 = std::sqrt(sigma_f64);
            sigma_f64 = 0.0;
            for (int j = 0; j < dimensionCount_i32; j++)
            {
                w_vf64[j] = B_mf64(k, j);
                sigma_f64 += (w_vf64[j] * w_vf64[j]); // sigma = B(k,j)^2
            }

            for (int j = 0; j <= k; j++)
            {
                if (j == k)
                {
                    v_vf64[j] = (C_rmf64(k, j) - alpha_f64);
                }
                else
                {
                    v_vf64[j] = C_rmf64(k, j);
                }
                sigma_f64 += (v_vf64[j] * v_vf64[j]);
            }
            // sigma_f64 is sum of all squared numbers and there are at least one positive among them (see (1))
            // therefore sigma_f64 is for sure positive
            // min(g_minCholSystemNoise_v2f64[0], g_minCholSystemNoise_v2f64[1])^2 < sigma_f64
            if (Fusion::isZero(sigma_f64))
            {
                // can never happen
                return;
            }

            alpha_f64 = (2.0 / sigma_f64);
            for (int i = 0; i <= k; i++)
            {
                sigma_f64 = 0.0;
                for (int j = 0; j < dimensionCount_i32; j++)
                {
                    sigma_f64 += (B_mf64(i, j) * w_vf64[j]);
                }
                for (int j = 0; j <= k; j++)
                {
                    sigma_f64 += (C_rmf64(i, j) * v_vf64[j]);
                }
                beta_f64 = (alpha_f64 * sigma_f64);

                for (int j = 0; j < dimensionCount_i32; j++)
                {
                    B_mf64(i, j) -= (beta_f64 * w_vf64[j]);
                }
                for (int j = 0; j <= k; j++)
                {
                    C_rmf64(i, j) -= (beta_f64 * v_vf64[j]);
                }
            }
        }
        // ensure Cholesky factor of state covariance is still an upper triangular matrix
        // PLD_ENSURE(rsd::isEqual(0.0, f_cholStateCovMatrix_rvf64(1, 0), 0.00001));
    }

    //---------------------------------------------------------------------
    //! @brief computes state covariance maxtrix (its Cholesky factor) & updated state (phi, d) based on the measurement data using Carlsons Fast Triangulation.
    //! Grewal and Andrews: Kalman Filtering, 2nd Edition, 2001, 6.5.1 Carlson-Schmidt Square-Root Filtering
    //! @param[in]      f_residual_f32                  : residual between measurement and predicted model
    //! @param[in]      f_measurementNoise_f32          : residual noise
    //! @param[in]      f_jacobiPhi_f64                 : Jacobian in phi  of residual
    //! @param[in]      f_jacobiDist_f64                : Jacobian in dist of residual
    //! @param[in,out]  f_stateCovMatrix_rmf64          : input predicted state covariance's Cholesky factor and output updated state covariance matrix's Cholesky factor
    //! @param[in,out]  f_stateVector_rvf64             : input predicted state, output updated state
    //! @author yij7szh
    //---------------------------------------------------------------------
    inline void carlsonsFastTriangularObservationalUpdate(
        const float f_residual_f32,
        const float f_measurementNoise_f32,
        const float &f_jacobiPhi_f64,
        const float &f_jacobiDist_f64,
        Eigen::Matrix2d &f_cholStateCovMatrix_rvf64,
        Eigen::Vector2d &f_stateVector_rvf64)
    {
        //! (use in pair with Schmidt Householder Triangularization Temporal Update)
        //! update state vector given estimated_P's Cholesky factor (C*_P), Jacobian(H) and measurement:
        //!
        //! updated_state = estimated_state + C*_P C*_P^t H^t * (residual/alpha); where alpha is a scalar intermediate result of the algorithm.
        //!
        //! and Cholesky factorize updated state covariance(updated_P):
        //! updated_P = (I - C*_P C*_P^t H^t) * C*_P C*_P^t
        //!
        //! which outputs: Cholesky factor C_P of updated_P (i.e. C_P C_P^t = updated_P)

        // // require positive measurement noise
        // PLD_REQUIRE(rsd::isPositive(f_measurementNoise_f32));

        // // require Cholesky factor of state covariance is an upper triangular matrix
        // PLD_REQUIRE(rsd::isEqual(0.0, f_cholStateCovMatrix_rvf64(1, 0), 0.00001));

        // input
        const float z_f64 = static_cast<float>(f_residual_f32);
        const float R_f64 = static_cast<float>(f_measurementNoise_f32);
        const Eigen::Vector2d H_vf64(f_jacobiPhi_f64, f_jacobiDist_f64);

        // std::cout<<"carlson 1"<<"\n";
        // input/output
        Eigen::Matrix2d &C_rmf64(f_cholStateCovMatrix_rvf64); // a reference

        // std::cout<<"carlson 1.0"<<"\n";
        // init local variables
        const int dimension_i32 = 2;
        Eigen::Vector2d w_vf64(0, 0);
        // std::cout<<"carlson 1.1"<<"\n";
        float alpha_f64(0.);
        float beta_f64(0.);
        float gamma_f64(0.);
        float delta_f64(0.);
        float eta_f64(0.);
        float zeta_f64(0.);
        float sigma_f64(0.);

        // std::cout<<"carlson 2"<<"\n";
        // Carlsons's observational update
        alpha_f64 = R_f64; // variance of measurement error must be always positive
        // PLD_ASSERT(rsd::isPositive(R_f64));
        delta_f64 = z_f64; // scalar measurement

        for (int j = 0; j < dimension_i32; j++)
        {
            // delta -= (H[j] * x[j]); // This line is commented on purpose as delta is already residual
            sigma_f64 = 0;
            for (int i = 0; i <= j; i++) // C is upper triangular matrix
            {
                sigma_f64 += (C_rmf64(i, j) * H_vf64[i]); // sigma1 = c11 h1; sigma2 = c12 h1 + c22 h2
            }
            beta_f64 = alpha_f64; //                           (2)
            // beta_f64 = alpha_f64 > 0                        (3)
            // PLD_ASSERT2(
            //    rsd::isPositive(beta_f64),
            //    "carlsonsFastTriangularObservationalUpdate(): variance of measurement error must be positive");

            alpha_f64 += (sigma_f64 * sigma_f64); //           (4)
            // from (2) & (4): we have alpha_f64 * beta_f64 = (beta_f64 + sigma_f64^2) * beta_f64            (5)

            // from (3) & (5), we can conclude alpha_f64 * beta_f64 > 0
            // PLD_ASSERT(rsd::isPositive(alpha_f64 * beta_f64));
            if (Fusion::isNegative(alpha_f64 * beta_f64))
            {
                // std::cout<<"alpha_f64 * beta_f64 is negative"<<"\n";
                return; // can never happen
            }

            // std::cout<<"carlson 3"<<"\n";
            gamma_f64 = std::sqrt(alpha_f64 * beta_f64);
            // std::cout<<"carlson 4"<<"\n";

            if (Fusion::isZero(gamma_f64))
            {
            std:
                cout << "gama_f64 zero" << "\n";
                std::cout << "alpha_f64: " << alpha_f64 << " beta_f64: " << beta_f64 << "\n";
                return; // can never happen
            }

            eta_f64 = (beta_f64 / gamma_f64);
            zeta_f64 = (sigma_f64 / gamma_f64);
            w_vf64[j] = 0;

            // W(2x1): unscaled Kalman gain
            // W = C*_P  C*_P^t * H^t
            // expands to: w1 = (c11^2 + c12^2) h1 + c12c22 h2
            //             w2 = c11c22 h1 + c22^2 h2
            // where, [c11,c12; 0, c22] = C*_P & H = [h1, h2]

            for (int i = 0; i <= j; i++)
            {
                float tau = C_rmf64(i, j);
                C_rmf64(i, j) = (eta_f64 * C_rmf64(i, j) - zeta_f64 * w_vf64[i]);
                w_vf64[i] += (tau * sigma_f64);
            }
        }

        // std::cout<<"carlson 5"<<"\n";
        //! alpha = R + sigma1^2 + sigma2^2  > 0 (see (2))
        // PLD_ASSERT(rsd::isPositive(alpha_f64));
        const float epsilon_f64 = (delta_f64 / alpha_f64); // scale for unscaled Kalman gain W

        // when alpha is insignificant, it will result in too large Kalman gain, and therefore unreasonable update
        // PLD_ASSERT(!rsd::isEqual(alpha_f64, 0.0, 0.00000001));

        // Kalman update:
        f_stateVector_rvf64[0] += (epsilon_f64 * w_vf64[0]);
        f_stateVector_rvf64[1] += (epsilon_f64 * w_vf64[1]);

        // ensure Cholesky factor is an upper triangular matrix
        // PLD_ENSURE(rsd::isEqual(0.0, f_cholStateCovMatrix_rvf64(1, 0), 0.00001));
    }

    //---------------------------------------------------------------------
    //! @brief calculate the distance (residual) between a point and the line model
    //! @param[in]      f_point_vf32         : a world point used to update the line
    //! @param[in]      f_lineModel          : line model that will be updated later
    //! @author yij7szh
    //---------------------------------------------------------------------
    inline float calcResidualPoint(const Eigen::Vector3d &f_point_vf32, const CTrackLine::Ptr &f_lineModel)
    {
        // /PLD_REQUIRE(rsd::isValidVector(f_point_vf32));
        return f_lineModel->calcOrthDistOfWorldPointSigned(f_point_vf32);
    }

    //---------------------------------------------------------------------
    //! @brief updatePhiAndDist
    //! @param[out]   f_lineModel_r             : updated line model with new (phi, d) from measurement point
    //! @brief updates Kalman state vector (phi,d) from poi
    //! @param[in]    f_point_vf32              : the update world point
    //! @param[in]    f_measurementNoise_f32    : measurement noise of the update point
    //! @param[in]    f_isShouldinverse_b       : get the flag from setPhiAndDistKeepTauWP, whether "dist" is negative or not, if yes, Startpoint and Endpoint should be exchanged
    //! @author yij7szh
    //!---------------------------------------------------------------------
    inline void updatePhiAndDist(
        CTrackLine::Ptr &f_lineModel_r,
        const Eigen::Vector3d f_point_vf32,
        const float f_measurementNoise_f32)
    {
        // residual = (measurement - model)
        float f_residual_rf32;

        f_residual_rf32 = calcResidualPoint(f_point_vf32, f_lineModel_r);

        // variance of measurement error
        const float residualNoise_f32 = f_measurementNoise_f32;

        // partial derivatives of ( -1 * residual )
        const float jacobiPhi_f64 = (f_point_vf32[0] * f_lineModel_r->getSinPhi()) -
                                    (f_point_vf32[1] * f_lineModel_r->getCosPhi());

        const float jacobiDist_f64 = 1.0f;

        Eigen::Matrix2d cholMat_f64 = f_lineModel_r->m_cholStateCov_mf64;
        // std::cout<<"cholMat_f64(0,0): "<<cholMat_f64(0,0)<<"\n";
        // std::cout<<"cholMat_f64(0,1): "<<cholMat_f64(0,1)<<"\n";
        // std::cout<<"cholMat_f64(1,0): "<<cholMat_f64(1,0)<<"\n";
        // std::cout<<"cholMat_f64(1,1): "<<cholMat_f64(1,1)<<"\n";

        Eigen::Vector2d stateVector_vf64(
            f_lineModel_r->getPhi(), static_cast<double>(f_lineModel_r->getDist()));

        // std::cout<<"before carlsonsFastTriangle"<<"\n";
        carlsonsFastTriangularObservationalUpdate(
            f_residual_rf32, residualNoise_f32, jacobiPhi_f64, jacobiDist_f64, cholMat_f64, stateVector_vf64);
        // std::cout<<"after carlsonsFast"<<"\n";
        f_lineModel_r->m_cholStateCov_mf64 = cholMat_f64;

        f_lineModel_r->setPhiAndDistKeepTauWP(
            float(static_cast<float>(stateVector_vf64[0])),
            float(static_cast<float>(stateVector_vf64[1])));
    }

}
#ifndef FUSION_CONSTANT_H_
#define FUSION_CONSTANT_H_

#include <cmath>
#include <Eigen/Eigen>

namespace Fusion
{
    namespace Constant
    {
        static constexpr double RAD2DEG = 180.0 / M_PI;
        static constexpr double DEG2RAD = M_PI / 180.0;
        static Eigen::Vector3d GRAVITY(0.0, 0.0, -9.78);
    }

    struct Glv
    {
    public:
        static constexpr double g_Re = 6378137.0;
        static constexpr double g_f = (1.0 / 298.257);
        static constexpr double g_Rp = (1.0 - g_f) * g_Re;
        static constexpr double g_e = sqrt(2 * g_f - g_f * g_f);
        static constexpr double g_e2 = g_e * g_e;
        static constexpr double g_ep = std::sqrt(g_Re * g_Re - g_Rp * g_Rp) / g_Rp;
        static constexpr double g_ep2 = g_ep * g_ep;

        static constexpr double g_wie = 7.2921151467e-5; // g_selfRotAngularVel
        static constexpr double g_g0 = 9.7803267714;     // GRAVITY
        static constexpr double g_mg = g_g0 * 1e-3;
        static constexpr double g_ug = g_mg * 1e-3;

        static constexpr double g_deg = M_PI / 180.0;
        static constexpr double g_min = g_deg / 60.0;
        static constexpr double g_sec = g_min / 60.0;

        static constexpr double g_ppm = 1.0e-6;
        static constexpr double g_hour2sec = 3600.0;

        static constexpr double g_degPerSecond = g_deg / 1.0;
        static constexpr double g_degPerHour = g_deg / g_hour2sec;
        static constexpr double g_dpsh = g_deg / std::sqrt(g_hour2sec);
        static constexpr double g_dphpsh = g_degPerHour / sqrt(g_hour2sec); //  ??
        static constexpr double g_ugPerSqrtHz = g_ug / sqrt(1.0);
        static constexpr double g_mgPerSqrtHz = g_mg / sqrt(1.0);
        static constexpr double g_ugPerSqrtHour = g_ug / sqrt(g_hour2sec);
        static constexpr double g_mgPerSqrtHour = g_mg / sqrt(g_hour2sec);

        static constexpr double g_mPerSqrtHour = 1.0 / sqrt(g_hour2sec);
        static constexpr double g_mpsPerSqrtHout = 1.0 / 1.0 / sqrt(g_hour2sec);
        static constexpr double g_ppmPerSqrtHout = g_ppm / sqrt(g_hour2sec);
        static constexpr double g_secPerSqrtHour = g_sec / sqrt(g_hour2sec);
    };
}
#endif
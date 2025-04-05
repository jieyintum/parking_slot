#ifndef FUSION_EARTH_PARAM_H
#define FUSION_EARTH_PARAM_H

#include <cmath>

namespace Fusion {
    static const double g_Re = 6378137; // m
    static const double g_Rp = 6356752; // m

    static const double g_elOblateCurv = ((g_Re - g_Rp) / g_Re);

    static const double g_selfRotAngularVel = 360. / 24. / 3600. / 180. * M_PI;

    static double GetRm(const double lat)  // rad
    {
        return g_Re * (1. - (2. * g_elOblateCurv) + (3. * g_elOblateCurv * std::sin(lat) * std::sin(lat)));
    }

    static double GetRn(const double lat)  // rad
    {
        return g_Re * (1. + (g_elOblateCurv * std::sin(lat) * std::sin(lat)));
    }
}

#endif
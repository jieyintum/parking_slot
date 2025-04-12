/*
 * @Author: guxiaojie
 * @Date: 2022-06-07 14:16:37
 * @LastEditTime: 2022-06-07 14:35:40
 * @LastEditors: guxiaojie
 * @Description: Do not edit
 * @FilePath: /loc_hdm/include/common/utils/CoodTrans.h
 */

#ifndef FUSION_WGS84_TO_GCJ02_H
#define FUSION_WGS84_TO_GCJ02_H

#include <cmath>

namespace Fusion
{
    class Wgs84ToGcj02
    {
    public:
        static void Wgs2Gcj(const double wgs84lon, const double wgs84lat, double &gcj02lon, double &gcj02lat)
        {
            double dlon, dlat;
            GetGeodeticOffset(wgs84lon, wgs84lat, dlon, dlat);
            gcj02lon = wgs84lon + dlon;
            gcj02lat = wgs84lat + dlat;
        }

    private:
        static void GetGeodeticOffset(const double wgs84lon, const double wgs84lat, double &lon, double &lat)
        {
            // get geodetic offset relative to 'center china'
            double lon0 = wgs84lon - 105.0;
            double lat0 = wgs84lat - 35.0;

            // generate an pair offset roughly in meters
            double lon1 = 300.0 + lon0 + 2.0 * lat0 + 0.1 * lon0 * lon0 + 0.1 * lon0 * lat0 + 0.1 * sqrt(fabs(lon0));
            lon1 = lon1 + (20.0 * sin(6.0 * lon0 * PI) + 20.0 * sin(2.0 * lon0 * PI)) * 2.0 / 3.0;
            lon1 = lon1 + (20.0 * sin(lon0 * PI) + 40.0 * sin(lon0 / 3.0 * PI)) * 2.0 / 3.0;
            lon1 = lon1 + (150.0 * sin(lon0 / 12.0 * PI) + 300.0 * sin(lon0 * PI / 30.0)) * 2.0 / 3.0;
            double lat1 = -100.0 + 2.0 * lon0 + 3.0 * lat0 + 0.2 * lat0 * lat0 + 0.1 * lon0 * lat0 + 0.2 * sqrt(fabs(lon0));
            lat1 = lat1 + (20.0 * sin(6.0 * lon0 * PI) + 20.0 * sin(2.0 * lon0 * PI)) * 2.0 / 3.0;
            lat1 = lat1 + (20.0 * sin(lat0 * PI) + 40.0 * sin(lat0 / 3.0 * PI)) * 2.0 / 3.0;
            lat1 = lat1 + (160.0 * sin(lat0 / 12.0 * PI) + 320.0 * sin(lat0 * PI / 30.0)) * 2.0 / 3.0;

            // latitude in radian
            double B = Deg2Rad(wgs84lat);
            double sinB = sin(B), cosB = cos(B);
            double W = sqrt(1 - kKRASOVSKY_ECCSQ * sinB * sinB);
            double N = kKRASOVSKY_A / W;

            // geodetic offset used by GCJ-02
            lon = Rad2Deg(lon1 / (N * cosB));
            lat = Rad2Deg(lat1 * W * W / (N * (1 - kKRASOVSKY_ECCSQ)));
        }

        constexpr inline double Deg2Rad(const double deg)
        {
            return deg * kDEG2RAD;
        }

        constexpr inline double Rad2Deg(const double rad)
        {
            return rad * kRAD2DEG;
        }

    private:
        constexpr double kKRASOVSKY_A = 6378245.0;                  // equatorial radius [unit: meter]
        constexpr double kKRASOVSKY_B = 6356863.0187730473;         // polar radius
        constexpr double kKRASOVSKY_ECCSQ = 6.6934216229659332e-3;  // first eccentricity squared
        constexpr double kKRASOVSKY_ECC2SQ = 6.7385254146834811e-3; // second eccentricity squared
        constexpr double PI = 3.14159265358979323846;               // ¦Ð

        constexpr double kDEG2RAD = PI / 180.0;
        constexpr double kRAD2DEG = 180.0 / PI;
    };
}

#endif
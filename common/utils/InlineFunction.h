/**
 * @File Name: InlineFunction.h
 * @Brief : 
 * @Author : Huang JianYu (hjyneverdie@163.com)
 * @Version : 1.0
 * @Creat Date : 2021-04-25
 * 
 * @copyright Copyright (c) 2021 saicmotor
 * modification history : First comment
 * Date: 2021-04-25  Version: 0.1  Author: Huang JianYu 
 * Changes: 
 */
#ifndef _MAP_INLINEFUNCTION_H_
#define _MAP_INLINEFUNCTION_H_

#include <string>
#include <cmath>
#include <float.h>
#include <iostream>
/**
 * @Brief : Transform WGS84 coordinate to UTM coordinate
 * @param  longitude_  WGS84 longitude
 * @param  latitude_   WGS84 latitude
 * @param  height_ellipsoid_ WGS84 altitude
 * @param  utm_x       UTM x
 * @param  utm_y       UTM y
 * @param  utm_z       UTM z
 */
inline void GC2UTM(const double longitude_, 
                   const double latitude_, 
                   const double height_ellipsoid_, 
                   double *utm_x, 
                   double *utm_y, 
                   double *utm_z)
{
    if(utm_x == NULL ||
       utm_y == NULL ||
       utm_z == NULL){
           return;
    }else{
        /* go on*/
    }
    int ProjNo   = 0;
    int ZoneWide = 0;
    double longitude1(0.0),latitude1(0.0), longitude0(0.0), X0(0.0),Y0(0.0), xval(0.0),yval(0.0);
    double a(0.0),f(0.0), e2(0.0),ee(0.0), NN(0.0), T(0.0),C(0.0),A(0.0), M(0.0), iPI(0.0);
    iPI      = 0.017453293;//=DEGREE_TO_RADIAN;
    ZoneWide = 3; //3 DEGREE width

    a = 6378137;
    f = 1 / 298.257223563;//earth ellipse constant

    double longitude = longitude_;
    double latitude  = latitude_;
    double altitude  = height_ellipsoid_;

    ProjNo     = (int)(longitude / ZoneWide) ;
    longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
    longitude0 = longitude0 * iPI ;
    longitude1 = longitude * iPI ; //经度转换为弧度
    latitude1  = latitude * iPI ; //纬度转换为弧度
    e2         = 2*f-f*f;
    ee         = e2*(1.0-e2);
    NN         = a/sqrt(1.0-e2*sin(latitude1)*sin(latitude1));
    T          = tan(latitude1)*tan(latitude1);
    C          = ee*cos(latitude1)*cos(latitude1);
    A          = (longitude1-longitude0)*cos(latitude1);
    M          = a*((1-e2/4-3*e2*e2/64-5*e2*e2*e2/256)*latitude1
                 - (3*e2/8+3*e2*e2/32+45*e2*e2*e2/1024)*sin(2*latitude1)
                 + (15*e2*e2/256+45*e2*e2*e2/1024)*sin(4*latitude1)
                 - (35*e2*e2*e2/3072)*sin(6*latitude1));
    xval       = NN*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*ee)*A*A*A*A*A/120);
    yval       = M+NN*tan(latitude1)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24 +(61-58*T+T*T+600*C-330*ee)*A*A*A*A*A*A/720);
    X0         = 1000000L*(ProjNo+1)+500000L;
    Y0         = 0;
    xval       = xval+X0; yval = yval+Y0;

    *utm_x     = xval;
    *utm_y     = yval;
    *utm_z     = altitude;
    //std::cout<<"GC2UTM "<<longitude<<" "<<latitude<<" "<<altitude<<" "<<xval<<" "<<yval<<" "<<altitude<<std::endl;
}

/**
 * @Brief : Transform UTM coordinate to WGS84 coordinate
 * @param  utm_x      UTM x
 * @param  utm_y      UTM y 
 * @param  utm_z      UTM z 
 * @param  longitude         WGS84 longitude
 * @param  latitude          WGS84 latitude
 * @param  height_ellipsoid  WGS84 altitude
 */
inline void UTM2GC(const double utm_x, 
                   const double utm_y, 
                   const double utm_z, 
                   double *longitude,
                   double *latitude,
                   double *height_ellipsoid)
{

    if(longitude == NULL ||
       latitude == NULL ||
       height_ellipsoid == NULL){
           return;
    }else{

    }
    int ProjNo(0);
    int ZoneWide(0);
    double longitude1(0.0),latitude1(0.0), longitude0(0.0), X0(0.0),Y0(0.0), xval(0.0),yval(0.0);
    double e1(0.0),e2(0.0),f(0.0),a(0.0), ee(0.0), NN(0.0), T(0.0),C(0.0), M(0.0), D(0.0),R(0.0),u(0.0),fai(0.0), iPI(0.0);
    iPI      = 0.017453293;// DEGREE_TO_RADIAN;
    a        = 6378137;
    f        = 1 / 298.257223563;
    ZoneWide = 3; // 3 degree width

    double X        = utm_x;
    double Y        = utm_y;
    double altitude = utm_z;

    ProjNo     = (int)(X/1000000L) ; //find zone in earth
    longitude0 = (ProjNo-1)*ZoneWide + ZoneWide / 2;
    longitude0 = longitude0*iPI ; //center longitude
    X0   = ProjNo*1000000L+500000L;
    Y0   = 0;
    xval = X-X0;
    yval = Y-Y0; //带内大地坐标
    e2   = 2*f-f*f;
    e1   = (1.0-sqrt(1-e2))/(1.0+sqrt(1-e2));
    ee   = e2/(1-e2);
    M    = yval;
    u    = M/(a*(1-e2/4-3*e2*e2/64-5*e2*e2*e2/256));
    fai  = u + (3*e1/2-27*e1*e1*e1/32)*sin(2*u)+(21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*u)
             + (151*e1*e1*e1/96)*sin(6*u)+(1097*e1*e1*e1*e1/512)*sin(8*u);
    C  = ee*cos(fai)*cos(fai);
    T  = tan(fai)*tan(fai);
    NN = a/sqrt(1.0-e2*sin(fai)*sin(fai));
    R  = a*(1-e2)/sqrt((1-e2*sin(fai)*sin(fai))*(1-e2*sin(fai)*sin(fai))*(1-e2*sin(fai)*sin(fai)));
    D = xval/NN;
    //计算经度(Longitude) 纬度(Latitude)
    longitude1 = longitude0+(D-(1+2*T+C)*D*D*D/6+(5-2*C+28*T-3*C*C+8*ee+24*T*T)*D
                             *D*D*D*D/120)/cos(fai);
    latitude1 = fai -(NN*tan(fai)/R)*(D*D/2-(5+3*T+10*C-4*C*C-9*ee)*D*D*D*D/24
                                      +(61+90*T+298*C+45*T*T-256*ee-3*C*C)*D*D*D*D*D*D/720);

    *longitude        = longitude1 / iPI;
    *latitude         = latitude1 / iPI;
    *height_ellipsoid = altitude;

   // std::cout<<"UTM2GC "<<utm_x<<" "<<utm_y<<" "<<utm_z<<" "<<longitude1 / iPI<<" "<<latitude1 / iPI<<altitude<<std::endl;
}

const double X_PI_ = 3.14159265358979324 * 3000.0 / 180.0;
const double PI_ = 3.1415926535897932384626;  // π
const double A_ = 6378245.0;  // 长半轴
const double EE_ = 0.00669342162296594323;  // 偏心率平方

inline double transformlat(const double &lon, const double &lat)
{
	double ret = -100.0 + 2.0 * lon +3.0 * lat + 0.2 * lat * lat + \
          0.1 * lon * lat + 0.2 * sqrt(fabs(lon));
	ret += (20.0 * sin(6.0 * lon * PI_) + 20.0 *
            sin(2.0 * lon * PI_)) * 2.0 / 3.0;
    ret += (20.0 * sin(lat * PI_) + 40.0 *
            sin(lat / 3.0 * PI_)) * 2.0 / 3.0;
    ret += (160.0 * sin(lat / 12.0 * PI_) + 320 *
            sin(lat * PI_ / 30.0)) * 2.0 / 3.0;
	return ret;
}

inline double transformlon(const double &lon, const double &lat)
{
	double ret = 300.0 + lon + 2.0 * lat + 0.1 * lon * lon + \
          0.1 * lon * lat + 0.1 * sqrt(fabs(lon));
    ret += (20.0 * sin(6.0 * lon * PI_) + 20.0 *
            sin(2.0 * lon * PI_)) * 2.0 / 3.0;
    ret += (20.0 * sin(lon * PI_) + 40.0 *
            sin(lon / 3.0 * PI_)) * 2.0 / 3.0;
    ret += (150.0 * sin(lon / 12.0 * PI_) + 300.0 *
            sin(lon / 30.0 * PI_)) * 2.0 / 3.0;
	return ret;
}

inline void WGS2GCJ(const double wgs_long, const double wgs_lat,
				double &longitude, double &latitude)
{
	double dlat = transformlat(wgs_long - 105.0, wgs_lat - 35.0);
	double dlon = transformlon(wgs_long - 105.0, wgs_lat - 35.0);
	double radlat = wgs_lat / 180.0 * PI_;
	double magic = sin(radlat);
	magic = 1 - EE_ * magic * magic;
	double sqrtmagic = sqrt(magic);
	dlat = (dlat * 180.0) / ((A_ * (1 - EE_)) / (magic * sqrtmagic) * PI_);
    dlon = (dlon * 180.0) / (A_ / sqrtmagic * cos(radlat) * PI_);
    latitude = wgs_lat + dlat;
    longitude = wgs_long + dlon;
}

inline void GCJ2WGS(const double gcj_long, const double gcj_lat,
				double &longitude, double &latitude)
{
	double dlat = transformlat(gcj_long - 105.0, gcj_lat - 35.0);
	double dlon = transformlon(gcj_long - 105.0, gcj_lat - 35.0);
	double radlat = gcj_lat / 180.0 * PI_;
	double magic = sin(radlat);
	magic = 1 - EE_ * magic * magic;
	double sqrtmagic = sqrt(magic);
	dlat = (dlat * 180.0) / ((A_ * (1 - EE_)) / (magic * sqrtmagic) * PI_);
    dlon = (dlon * 180.0) / (A_ / sqrtmagic * cos(radlat) * PI_);
	double mglat = gcj_lat + dlat;
	double mglon = gcj_long + dlon;
    latitude = gcj_lat * 2 - mglat;
    longitude = gcj_long * 2 - mglon;
}

/**
 * @Brief : Transform UTM coordinates to ego coordinates
 * @param  utm_ori_x    ego coordinate x in utm coordinate system
 * @param  utm_ori_y    ego coordinate y in utm coordinate system
 * @param  utm_ori_z    ego coordinate z in utm coordinate system
 * @param  ori_heading_gps  ego heading angle
 * @param  utm_x        utm coordinate x
 * @param  utm_y        utm coordinate y
 * @param  utm_z        utm coordinate z
 * @param  utm_heading  utm heading angle
 * @param  ec_x         ego coordinate x
 * @param  ec_y         ego coordinate y
 * @param  ec_z         ego coordinate z
 * @param  ec_heading   ego coordinate heading
 */
inline void UTM2EC(double utm_ori_x, double utm_ori_y,double utm_ori_z,double ori_heading_gps,
                   double utm_x,double utm_y,double utm_z,double utm_heading,
                   double & ec_x,double & ec_y,double  & ec_z,double & ec_heading
                   ){
    ec_x = cos(ori_heading_gps) * (utm_y - utm_ori_y) + sin(ori_heading_gps) * (utm_ori_x - utm_x);
    ec_y = cos(ori_heading_gps) * (utm_ori_x - utm_x) - sin(ori_heading_gps) * (utm_y - utm_ori_y);
    ec_z = utm_z - utm_ori_z;
    ec_heading = utm_heading - ori_heading_gps;
}

inline void EC2UTM( double utm_ori_x, double utm_ori_y, double utm_ori_z, double ori_heading_gps,
                    double ec_x, double ec_y, double ec_z, double ec_heading,
                    double *utm_x, double *utm_y, double *utm_z, double *utm_heading)
{
    *utm_x = -cos(ori_heading_gps) * ec_y - sin(ori_heading_gps) * ec_x + utm_ori_x;
    *utm_y = -sin(ori_heading_gps) * ec_y + cos(ori_heading_gps) * ec_x + utm_ori_y;
    *utm_z = ec_z + utm_ori_z;
    *utm_heading = ec_heading + ori_heading_gps;
}

inline void BODY2UTM( double utm_ori_x, double utm_ori_y, double utm_ori_z, double ori_heading_gps,
                    double ec_x, double ec_y, double ec_z, double ec_heading,
                    double *utm_x, double *utm_y, double *utm_z, double *utm_heading)
{
    *utm_x = cos(ori_heading_gps) * ec_x - sin(ori_heading_gps) * ec_y + utm_ori_x;
    *utm_y = sin(ori_heading_gps) * ec_x + cos(ori_heading_gps) * ec_y + utm_ori_y;
    *utm_z = ec_z + utm_ori_z;
    *utm_heading = ec_heading + ori_heading_gps;
}

inline void UTM2BODY( double utm_ori_x, double utm_ori_y, double utm_ori_z, double ori_heading_gps,
                    double utm_x, double utm_y, double utm_z, double utm_heading,
                    double *ec_x, double *ec_y, double *ec_z, double *ec_heading)
{
    *ec_x = cos(ori_heading_gps) * (utm_x - utm_ori_x) + sin(ori_heading_gps) * (utm_y - utm_ori_y);
    *ec_y = -sin(ori_heading_gps) * (utm_x - utm_ori_x) + cos(ori_heading_gps) * (utm_y - utm_ori_y); 
    *ec_z = utm_z - utm_ori_z;
    *ec_heading = utm_heading - ori_heading_gps;
}

inline double WarpAngle(const double input)
{
    double output = input;
    while (output > M_PI) {
        output -= (M_PI * 2.0);
    }
    while (output < -M_PI) {
        output += (M_PI * 2.0);
    }
    return output;
}

/**
 * @Brief : 3d vector uniformation
 * @param  x                
 * @param  y                
 * @param  z                
 */
inline void uniformization(double & x,double & y,double & z){
    double length = sqrt(x*x + y*y + z*z);
    if(length > DBL_MIN){
        x = x/length;
        y = y/length;
        z = z/length;
    }else{
        /* do nothing*/
    }
}

/**
 * @Brief : multiplication cross
 * @param  x0               
 * @param  x1               
 * @param  x2               
 * @param  y0               
 * @param  y1               
 * @param  y2               
 * @param  z0               
 * @param  z1               
 * @param  z2               
 */
inline void multiCross(double x0,double x1,double x2,
                       double y0,double y1,double y2,
                       double & z0,double & z1,double & z2){
    z0 = x1*y2 - x2*y1;
    z1 = x2*y0 - x0*y2;
    z2 = x0*y1 - x1*y0;
}

/**
 * @Brief : multiplication cross,and uniform result
 * @param  x0               
 * @param  x1               
 * @param  x2               
 * @param  y0               
 * @param  y1               
 * @param  y2               
 * @param  z0               
 * @param  z1               
 * @param  z2               
 */
inline void multiCrossUnit(double x0,double x1,double x2,
                           double y0,double y1,double y2,
                           double & z0,double & z1,double & z2){
    multiCross(x0,x1,x2,
               y0,y1,y2,
               z0,z1,z2);
    uniformization(z0,z1,z2);
} 

/**
 * @Brief : distance of two 3d vector
 * @param  x1               
 * @param  y1               
 * @param  z1               
 * @param  x2               
 * @param  y2               
 * @param  z2               
 * @return double           
 */
inline double distance(double x1,double y1,double z1,
                       double x2,double y2,double z2){
    return sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
}

/**
 * @Brief : update string with src_str
 * @param  p_dest_str       
 * @param  src_str          
 */
inline void updateString(std::string *       p_dest_str,
                         const std::string & src_str){
    if(p_dest_str != nullptr){
        p_dest_str->assign(src_str);
    }else{
        /* do nothing*/
    }
}

#endif //_MAP_INLINEFUNCTION_H_
#ifndef FUSION_UTILS_PLDUTILS_H_
#define FUSION_UTILS_PLDUTILS_H_

#include <Eigen/Eigen>

namespace Fusion {

struct veh_params {

    void calcAfterTransfer()
    {
        g_rearAxleToVehicleFront = g_vehicleLength - g_rearAxleToVehicleRear;

        g_vehicleModelMaxX = g_rearAxleToVehicleFront + g_vehicleRangeBufferX;
        g_vehicleModelMinX = -g_rearAxleToVehicleRear - g_vehicleRangeBufferX;
        g_vehicleModelMaxY = g_vehicleWidth/2.0f + g_vehicleRangeBufferY;
        g_vehicleModelMinY = -g_vehicleWidth/2.0f - g_vehicleRangeBufferY;

        g_maxVehicleCoordX_f = ((float)g_vehicleOrignal[1]/(float)g_resolutionY_i)*g_worldWidth_f;
        g_maxVehicleCoordY_f = g_worldLength_f*0.5;

        g_minVehicleCoordX_f = -((float)(g_resolutionY_i - g_vehicleOrignal[1])/(float)g_resolutionY_i)*g_worldWidth_f;
        g_minVehicleCoordY_f = -g_worldLength_f*0.5;

        g_maxObserveVehicleCoordX_f = g_maxVehicleCoordX_f - g_buffer_f;
        g_maxObserveVehicleCoordY_f = g_maxVehicleCoordY_f - g_buffer_f;

        g_minObserveVehicleCoordX_f = g_minVehicleCoordX_f + g_buffer_f;
        g_minObserveVehicleCoordY_f = g_minVehicleCoordY_f + g_buffer_f;
    }

    float g_worldWidth_f = 18.0f;
    float g_worldLength_f = 18.0f;
    int g_resolutionX_i = 512; //512 pixels
    int g_resolutionY_i = 512; //512 pixles


    float g_vehicleLength = 5.0f;
    float g_vehicleWidth = 2.0f;
    float g_rearAxleToVehicleRear = 0.8f;

    std::vector<uint16_t> g_vehicleOrignal = {256, 280};

    float g_rearAxleToVehicleFront = 0.0f;
    float g_vehicleRangeBufferX = 0.5f;
    float g_vehicleRangeBufferY = 0.5f;
    float g_vehicleModelMaxX = 0.0f;
    float g_vehicleModelMinX = 0.0f;
    float g_vehicleModelMaxY = 0.0f;
    float g_vehicleModelMinY = 0.0f;


    float g_maxVehicleCoordX_f = 0.0f;
    float g_maxVehicleCoordY_f = 0.0f;
    float g_minVehicleCoordX_f = 0.0f;
    float g_minVehicleCoordY_f = 0.0f;

    float g_buffer_f = 2.0f;
    float g_maxObserveVehicleCoordX_f = 0.0f;
    float g_maxObserveVehicleCoordY_f = 0.0f;
    float g_minObserveVehicleCoordX_f = 0.0f;
    float g_minObserveVehicleCoordY_f = 0.0f;



};


namespace PldUtils {
struct Point2f {
    float x;
    float y;
    
    Point2f(const float x_in = 0.0f, const float y_in = 0.0f)
    {
        x = x_in;
        y = y_in;
    }
};

const float g_worldWidth_f(18.0f);
const float g_worldLength_f(18.0f);

const int g_resolutionX_i(512); //512 pixels
const int g_resolutionY_i(512); //512 pixles

// const float g_precisionX(g_worldWidth_f / static_cast<float>(g_resolutionX_i));
// const float g_precisionY(g_worldLength_f / static_cast<float>(g_resolutionY_i));
const float g_precisionX(0.03515625f);
const float g_precisionY(0.03515625f);


const int g_vehicleOrignalXInImage(256);
const float g_vehicleOrignalYInImageForES33(297);
const float g_vehicleOrignalYInImageForEP35(299);

const float g_buffer_f(2.0f);
const float g_maxObserveVehicleCoordX_f = ((float)g_vehicleOrignalYInImageForES33/(float)g_resolutionY_i)*g_worldWidth_f - g_buffer_f;
const float g_maxObserveVehicleCoordY_f = g_worldLength_f*0.5 - g_buffer_f;

const float g_minObserveVehicleCoordX_f = -((float)(g_resolutionY_i - g_vehicleOrignalYInImageForES33)/(float)g_resolutionY_i)*g_worldWidth_f + g_buffer_f;
const float g_minObserveVehicleCoordY_f = -g_worldLength_f*0.5 + g_buffer_f;


const float g_maxVehicleCoordX_f = ((float)g_vehicleOrignalYInImageForES33/(float)g_resolutionY_i)*g_worldWidth_f;
const float g_maxVehicleCoordY_f = g_worldLength_f*0.5;

const float g_minVehicleCoordX_f = -((float)(g_resolutionY_i - g_vehicleOrignalYInImageForES33)/(float)g_resolutionY_i)*g_worldWidth_f;
const float g_minVehicleCoordY_f = -g_worldLength_f*0.5;


const float g_vehicleRangeBufferX = 0.35f;
const float g_vehicleRangeBufferY = 0.5f;
const float g_vehicleModelMaxX = 3.0f + g_vehicleRangeBufferX;
const float g_vehicleModelMinX = -2.0f - g_vehicleRangeBufferX;
const float g_vehicleModelMaxY = 1.0f + g_vehicleRangeBufferY;
const float g_vehicleModelMinY = -1.0f - g_vehicleRangeBufferY;



static void PointCoordinateTransfer(Point2f& f_point)
{
    const float l_vehicleX_f = (g_vehicleOrignalYInImageForES33 - f_point.y) * g_precisionX;
    const float l_vehicleY_f = (g_vehicleOrignalXInImage - f_point.x) * g_precisionY;

    f_point.x = l_vehicleX_f;
    f_point.y = l_vehicleY_f;
}

static void PointCoordinateTransfer(Point2f& f_point, const std::string vehicleModel)
{
    float vehicleOriginalCoordY(0.0f);
    if(vehicleModel == "ES33") {
        vehicleOriginalCoordY = g_vehicleOrignalYInImageForES33;
    }
    else {
        vehicleOriginalCoordY = g_vehicleOrignalYInImageForEP35;
    }
    const float l_vehicleX_f = (vehicleOriginalCoordY - f_point.y) * g_precisionX;
    const float l_vehicleY_f = (g_vehicleOrignalXInImage - f_point.x) * g_precisionY;

    f_point.x = l_vehicleX_f;
    f_point.y = l_vehicleY_f;
}

static void PointCoordinateTransfer(Point2f& f_point, Fusion::veh_params vehicleParams)
{
    const float l_vehicleX_f = (vehicleParams.g_vehicleOrignal[1] - f_point.y) * g_precisionX;
    const float l_vehicleY_f = (vehicleParams.g_vehicleOrignal[0] - f_point.x) * g_precisionY;

    f_point.x = l_vehicleX_f;
    f_point.y = l_vehicleY_f;
}


}   // namespace PldUtils
}   // namespace Fusion

#endif
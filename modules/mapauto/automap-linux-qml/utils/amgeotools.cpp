#include "amgeotools.h"
#include <math.h>

NAMESPACE_START

AMGeoTools::AMGeoTools(QObject *parent) : QObject(parent)
{

}

void AMGeoTools::locate(double& longitude, double& latitude)
{
    //奎科 gcj02ll
    longitude = 116.30734;
    latitude = 40.04171;

    //深圳福田
//    longitude = 114.05326;
//    latitude = 22.53969;
}

double AMGeoTools::angle2Arc(double angle)
{
    return angle * AM_PI / 180.0;
}

double AMGeoTools::ll2Distance(double longitude1, double latitude1, double longitude2, double latitude2)
{
    double lon1 = angle2Arc(longitude1);
    double lat1 = angle2Arc(latitude1);
    double lon2 = angle2Arc(longitude2);
    double lat2 = angle2Arc(latitude2);

    double ret = AM_Earth_Radius * acos(cos(lat1) * cos(lat2) * cos(lon1 - lon2) + sin(lat1) * sin(lat2));
    return ret;
}

double AMGeoTools::distanceFromHere(double longitude, double latitude)
{
    double cur_lon = 0;
    double cur_lat = 0;
    AMGeoTools::locate(cur_lon, cur_lat);
    return AMGeoTools::ll2Distance(cur_lon, cur_lat, longitude, latitude);
}

double AMGeoTools::merc2Distance()
{

}

NAMESPACE_END

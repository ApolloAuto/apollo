#ifndef AMGEOTOOLS_H
#define AMGEOTOOLS_H

#include <QObject>
#include "utils/def.h"
#include "BNAInterface/navi_auto_adapter_if.h"

NAMESPACE_START

const double AM_Earth_Radius = 6371004; //单位为m
const double AM_PI = 3.14159265358979323846;

class AMGeoTools : public QObject
{
    Q_OBJECT
public:
    explicit AMGeoTools(QObject *parent = 0);

    static void locate(double& longitude, double& latitude);

    //longitude, latitude distance
    static double ll2Distance(double longitude1, double latitude1, double longitude2, double latitude2);

    static double distanceFromHere(double longitude, double latitude);

    //Mercator distance
    static double merc2Distance();

    static int getDistrictInfoByPoint(const double& longitude, const double& latitude)
    {
        int nDistrictID = 0;
        BNAGetDistrictInfoByPoint(longitude, latitude, nDistrictID);
        return nDistrictID;
    }

signals:

public slots:

private:
    static double angle2Arc(double angle);
};

NAMESPACE_END

#endif // AMGEOTOOLS_H

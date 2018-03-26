/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <cmath>
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {
namespace msf {

/* Ellipsoid model constants (actual values here are for WGS84) */
const double sm_a = 6378137.0;
const double sm_b = 6356752.31425;
// const double sm_EccSquared = 6.69437999013e-03;
const double utm_scale_factor = 0.9996;
const double sins_rad_to_deg = 57.295779513;
const double sins_deg_to_rad = 0.01745329252;

const double sins_pi = 3.1415926535897932;
const double sins_r0 = 6378137.0;
const double sins_e = 0.08181919108425;
const double sins_e2 = 0.00669437999013;

/*
 * arclength0f_meridian
 *
 * Computes the ellipsoidal distance from the equator to a point at a
 * given latitude.
 *
 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 * GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 *
 * Inputs:
 *     phi - Latitude of the point, in radians.
 *
 * Globals:
 *     sm_a - Ellipsoid model major axis.
 *     sm_b - Ellipsoid model minor axis.
 *
 * Returns:
 *     The ellipsoidal distance of the point from the equator, in meters.
 *
 */
static double arclength0f_meridian(double phi) {
    double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;
    double delta = 0.0;
    double epsilon = 0.0;
    double n = 0.0;
    double result = 0.0;

    /* Precalculate n */
    n = (sm_a - sm_b) / (sm_a + sm_b);

    /* Precalculate alpha */
    alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0)
            + (pow(n, 4.0) / 64.0));

    /* Precalculate beta */
    beta = (-3.0 * n / 2.0)
           + (9.0 * pow(n, 3.0) / 16.0)
           + (-3.0 * pow(n, 5.0) / 32.0);

    /* Precalculate gamma */
    gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta */
    delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);

    /* Precalculate epsilon */
    epsilon = (315.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series and return */
    result = alpha * (phi + (beta * sin(2.0 * phi))
             + (gamma * sin(4.0 * phi))
             + (delta * sin(6.0 * phi)) + (epsilon * sin(8.0 * phi)));

    return result;
}

/*
 * utm_central_meridian
 *
 * Determines the central meridian for the given UTM zone.
 *
 * Inputs:
 *     zone - An integer value designating the UTM zone, range [1,60].
 *
 * Returns:
 *   The central meridian for the given UTM zone, in radians, or zero
 *   if the UTM zone parameter is outside the range [1,60].
 *   Range of the central meridian is the radian equivalent of [-177,+177].
 *
 */
inline double utm_central_meridian(int zone) {
    return (-183.0 + (zone * 6.0)) * sins_deg_to_rad;
}

/*
 * footpoint_latitude
 *
 * Computes the footpoint latitude for use in converting transverse
 * Mercator coordinates to ellipsoidal coordinates.
 *
 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 *
 * Inputs:
 *   y - The UTM northing coordinate, in meters.
 *
 * Returns:
 *   The footpoint latitude, in radians.
 *
 */
double footpoint_latitude(double y) {
    double y_ = 0.0;
    double alpha_ = 0.0;
    double beta_ = 0.0;
    double gamma_ = 0.0;
    double delta_ = 0.0;
    double epsilon_ = 0.0;
    double n = 0.0;
    double result = 0.0;

    /* Precalculate n (Eq. 10.18) */
    n = (sm_a - sm_b) / (sm_a + sm_b);

    /* Precalculate alpha_ (Eq. 10.22) */
    /* (Same as alpha in Eq. 10.17) */
    alpha_ = ((sm_a + sm_b) / 2.0) * (1 + (pow(n, 2.0) / 4)
             + (pow(n, 4.0) / 64));

    /* Precalculate y_ (Eq. 10.23) */
    y_ = y / alpha_;

    /* Precalculate beta_ (Eq. 10.22) */
    beta_ = (3.0 * n / 2.0)
            + (-27.0 * pow(n, 3.0) / 32.0) + (269.0 * pow(n, 5.0) / 512.0);

    /* Precalculate gamma_ (Eq. 10.22) */
    gamma_ = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta_ (Eq. 10.22) */
    delta_ = (151.0 * pow(n, 3.0) / 96.0)   + (-417.0 * pow(n, 5.0) / 128.0);

    /* Precalculate epsilon_ (Eq. 10.22) */
    epsilon_ = (1097.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series (Eq. 10.21) */
    result = y_ + (beta_ * sin(2.0 * y_)) + (gamma_ * sin(4.0 * y_))
             + (delta_ * sin(6.0 * y_)) + (epsilon_ * sin(8.0 * y_));

    return result;
}

/*
 * maplatlon_to_xy
 *
 * Converts a latitude/longitude pair to x and y coordinates in the
 * Transverse Mercator projection.  Note that Transverse Mercator is not
 * the same as UTM; a scale factor is required to convert between them.
 *
 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 * GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 *
 * Inputs:
 *    phi - Latitude of the point, in radians.
 *    lambda - Longitude of the point, in radians.
 *    lambda0 - Longitude of the central meridian to be used, in radians.
 *
 * Outputs:
 *    xy - A 2-element array containing the x and y coordinates
 *         of the computed point.
 *
 * Returns:
 *    The function does not return a value.
 *
 */
void maplatlon_to_xy(double phi, double lambda,
                     double lambda0, UTMCoor *xy) {
    double nn = 0.0;
    double nu2 = 0.0;
    double ep2 = 0.0;
    double t = 0.0;
    double t2 = 0.0;
    double l = 0.0;
    double l3coef = 0.0;
    double l4coef = 0.0;
    double l5coef = 0.0;
    double l6coef = 0.0;
    double l7coef = 0.0;
    double l8coef = 0.0;
    double tmp = 0.0;

    /* Precalculate ep2 */
    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);

    /* Precalculate nu2 */
    nu2 = ep2 * pow(cos(phi), 2.0);

    /* Precalculate nn */
    nn = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));

    /* Precalculate t */
    t = tan(phi);
    t2 = t * t;
    tmp = (t2 * t2 * t2) - pow(t, 6.0);

    /* Precalculate l */
    l = lambda - lambda0;

    /* Precalculate coefficients for l**nn in the equations below
     so a normal human being can read the expressions for easting
     and northing
     -- l**1 and l**2 have coefficients of 1.0 */
    l3coef = 1.0 - t2 + nu2;

    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;

    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    /* Calculate easting (x) */
    xy->x = nn * cos(phi) * l
           + (nn / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0))
           + (nn / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0))
           + (nn / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    /* Calculate northing (y) */
    xy->y = arclength0f_meridian(phi)
           + (t / 2.0 * nn * pow(cos(phi), 2.0) * pow(l, 2.0))
           + (t / 24.0 * nn * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0))
           + (t / 720.0 * nn * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0))
           + (t / 40320.0 * nn * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
}

/*
 * mapxy_to_latlon
 *
 * Converts x and y coordinates in the Transverse Mercator projection to
 * a latitude/longitude pair.  Note that Transverse Mercator is not
 * the same as UTM; a scale factor is required to convert between them.
 *
 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 *
 * Inputs:
 *   x - The easting of the point, in meters.
 *   y - The northing of the point, in meters.
 *   lambda0 - Longitude of the central meridian to be used, in radians.
 *
 * Outputs:
 *   philambda - A 2-element containing the latitude and longitude
 *               in radians.
 *
 * Returns:
 *   The function does not return a value.
 *
 * Remarks:
 *   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
 *   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
 *   to the footpoint latitude phif.
 *
 *   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
 *   to optimize computations.
 *
 */
void mapxy_to_latlon(double x, double y, double lambda0, WGS84Corr *philambda) {
    double phif = 0.0;
    double nf = 0.0;
    double nfpow = 0.0;
    double nuf2 = 0.0;
    double ep2 = 0.0;
    double tf = 0.0;
    double tf2 = 0.0;
    double tf4 = 0.0;
    double cf = 0.0;

    double x1frac = 0.0;
    double x2frac = 0.0;
    double x3frac = 0.0;
    double x4frac = 0.0;
    double x5frac = 0.0;
    double x6frac = 0.0;
    double x7frac = 0.0;
    double x8frac = 0.0;

    double x2poly = 0.0;
    double x3poly = 0.0;
    double x4poly = 0.0;
    double x5poly = 0.0;
    double x6poly = 0.0;
    double x7poly = 0.0;
    double x8poly = 0.0;

    /* Get the value of phif, the footpoint latitude. */
    phif = footpoint_latitude(y);

    /* Precalculate ep2 */
    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);

    /* Precalculate cos (phif) */
    cf = cos(phif);

    /* Precalculate nuf2 */
    nuf2 = ep2 * pow(cf, 2.0);

    /* Precalculate nf and initialize nfpow */
    nf = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nuf2));
    nfpow = nf;

    /* Precalculate tf */
    tf = tan(phif);
    tf2 = tf * tf;
    tf4 = tf2 * tf2;

    /* Precalculate fractional coefficients for x**n in the equations
     below to simplify the expressions for latitude and longitude. */
    x1frac = 1.0 / (nfpow * cf);

    nfpow *= nf;   /* now equals nf**2) */
    x2frac = tf / (2.0 * nfpow);

    nfpow *= nf;   /* now equals nf**3) */
    x3frac = 1.0 / (6.0 * nfpow * cf);

    nfpow *= nf;   /* now equals nf**4) */
    x4frac = tf / (24.0 * nfpow);

    nfpow *= nf;   /* now equals nf**5) */
    x5frac = 1.0 / (120.0 * nfpow * cf);

    nfpow *= nf;   /* now equals nf**6) */
    x6frac = tf / (720.0 * nfpow);

    nfpow *= nf;   /* now equals nf**7) */
    x7frac = 1.0 / (5040.0 * nfpow * cf);

    nfpow *= nf;   /* now equals nf**8) */
    x8frac = tf / (40320.0 * nfpow);

    /* Precalculate polynomial coefficients for x**n.
     -- x**1 does not have a polynomial coefficient. */
    x2poly = -1.0 - nuf2;

    x3poly = -1.0 - 2 * tf2 - nuf2;

    x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2
             - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 *nuf2)
             - 9.0 * tf2 * (nuf2 * nuf2);

    x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

    x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4
             - 107.0 * nuf2 + 162.0 * tf2 * nuf2;

    x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

    x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

    /* Calculate latitude */
    philambda->lat = phif + x2frac * x2poly * (x * x)
                    + x4frac * x4poly * pow(x, 4.0)
                    + x6frac * x6poly * pow(x, 6.0)
                    + x8frac * x8poly * pow(x, 8.0);

    /* Calculate longitude */
    philambda->log = lambda0 + x1frac * x + x3frac * x3poly * pow(x, 3.0)
                    + x5frac * x5poly * pow(x, 5.0)
                    + x7frac * x7poly * pow(x, 7.0);
}

/*
 * latlon_to_utmxy
 *
 * Converts a latitude/longitude pair to x and y coordinates in the
 * Universal Transverse Mercator projection.
 *
 * Inputs:
 *   lat - Latitude of the point, in radians.
 *   lon - Longitude of the point, in radians.
 *   zone - UTM zone to be used for calculating values for x and y.
 *          If zone is less than 1 or greater than 60, the routine
 *          will determine the appropriate zone from the value of lon.
 *
 * Outputs:
 *   xy - A 2-element array where the UTM x and y values will be stored.
 *
 * Returns:
 *   void
 *
 */
void latlon_to_utmxy(double lon_rad, double lat_rad, UTMCoor *xy) {
    int zone = 0;
    zone = static_cast<int>((lon_rad * sins_rad_to_deg + 180) / 6) + 1;

    maplatlon_to_xy(lat_rad, lon_rad, utm_central_meridian(zone), xy);

    /* Adjust easting and northing for UTM system. */
    xy->x = xy->x * utm_scale_factor + 500000.0;
    xy->y = xy->y * utm_scale_factor;
    if (xy->y < 0.0) {
        xy->y += 10000000.0;
    }
}

/*
 * utmxy_to_latlon
 *
 * Converts x and y coordinates in the Universal Transverse Mercator
 * projection to a latitude/longitude pair.
 *
 * Inputs:
 *	x - The easting of the point, in meters.
 *	y - The northing of the point, in meters.
 *	zone - The UTM zone in which the point lies.
 *	southhemi - True if the point is in the southern hemisphere;
 *               false otherwise.
 *
 * Outputs:
 *	latlon - A 2-element array containing the latitude and
 *            longitude of the point, in radians.
 *
 * Returns:
 *	The function does not return a value.
 *
 */
void utmxy_to_latlon(double x, double y, int zone,
                     bool southhemi, WGS84Corr *latlon) {
    double cmeridian = 0.0;

    x -= 500000.0;
    x /= utm_scale_factor;

    /* If in southern hemisphere, adjust y accordingly. */
    if (southhemi) {
        y -= 10000000.0;
    }

    y /= utm_scale_factor;

    cmeridian = utm_central_meridian(zone);
    mapxy_to_latlon(x, y, cmeridian, latlon);
}

void xyz_to_blh(const Eigen::Vector3d &xyz, Eigen::Vector3d *blh) {
    //    double e2=FE_WGS84*(2.0-FE_WGS84);
    double r2 = xyz[0] * xyz[0] + xyz[1] * xyz[1];
    double z = 0.0;
    double zk = 0.0;
    double v = sins_r0;
    double sinp = 0.0;

    for (z = xyz[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
        zk = z;
        sinp = z / sqrt(r2 + z * z);
        v = sins_r0 / sqrt(1.0 - sins_e2 * sinp * sinp);
        z = xyz[2] + v * sins_e2 * sinp;
    }
    (*blh)[0] = r2 > 1E-12 ? atan2(xyz[1], xyz[0]) : 0.0;
    (*blh)[1] = r2 > 1E-12 ? atan(z / sqrt(r2))
                    : (xyz[2] > 0.0 ? sins_pi / 2.0: - sins_pi / 2.0);
    (*blh)[2]= sqrt(r2 + z * z) - v;

    return;
}

void blh_to_xyz(const Eigen::Vector3d &blh, Eigen::Vector3d *xyz) {
    double sin_lati_2 = sin(blh[1]) * sin(blh[1]);
    double temp_a = sqrt(1.0 - sins_e2 * sin_lati_2);
    double rn = sins_r0 / temp_a;
    // double rm = rn * (1.0 - sins_e2) / (1.0 - sins_e2 * sin_lati_2);

    double cos_lat = cos(blh[1]);
    double sin_lat = sin(blh[1]);
    double cos_long = cos(blh[0]);
    double sin_long = sin(blh[0]);

    (*xyz)[0] = (rn + blh[2]) * cos_lat * cos_long;
    (*xyz)[1] = (rn + blh[2]) * cos_lat * sin_long;
    (*xyz)[2] = ((1 - sins_e2) * rn + blh[2]) * sin_lat;

    return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo

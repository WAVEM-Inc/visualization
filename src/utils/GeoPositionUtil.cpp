//
// Created by antique on 24. 3. 22.
//

#include <iostream>
#include "utils/GeoPositionUtil.h"

double toRadians(double degree) {
    return degree * M_PI / 180.0;
}

// 도로 변환
double toDegrees(double radian) {
    return radian * 180.0 / M_PI;
}

double getBearingBetweenPoints(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = toRadians(lat1);
    double lat2_rad = toRadians(lat2);
    double lon_diff_rad = toRadians(lon2 - lon1);
    double y = sin(lon_diff_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon_diff_rad);

    return ((int) toDegrees(atan2(y, x)) + 360) % 360;
}

Position translateLatLng(double lat, double lng, double distance, double degree) {
    const double EARTH_RADIUS = 6371.0;

    double latRad = toRadians(lat);
    double lngRad = toRadians(lng);
    double rad = toRadians(degree);

    double distRatio = distance * 0.001 / EARTH_RADIUS;


    double lat2 = asin(sin(latRad) * cos(distRatio) +
                       cos(latRad) * sin(distRatio) * cos(rad));
    double lng2 = lngRad + atan2(sin(rad) * sin(distRatio) * cos(latRad),
                                 cos(distRatio) - sin(latRad) * sin(lat2));

    Position position;
    position.latitude = toDegrees(lat2);
    position.longitude = toDegrees(lng2);

    return position;
}

std::array<double, 2> convert_wgs84_to_utm(double lat, double lng) {
    PJ *P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:32633", NULL);
    if (P == NULL) {
        std::cout << "Projection object creation failed." << "\n";
        return {0.0, 0.0};
    }

    PJ_COORD a = proj_coord(lng, lat, 0, 0);
    PJ_COORD b = proj_trans(P, PJ_FWD, a);

    return {b.enu.e, b.enu.n};
}
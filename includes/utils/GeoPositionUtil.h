//
// Created by antique on 24. 3. 19.
//

#ifndef ROUTE_EDITOR_GEOPOSITIONUTIL_H
#define ROUTE_EDITOR_GEOPOSITIONUTIL_H

#include <cmath>

double convertDecimalDegreesToRadians(double deg) {
    return (deg * M_PI / 180);
}

double convertRadiansToDecimalDegrees(double rad) {
    return (rad * 180 / M_PI);
}

double getBearingBetweenPoints(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = convertDecimalDegreesToRadians(lat1);
    double lat2_rad = convertDecimalDegreesToRadians(lat2);
    double lon_diff_rad = convertDecimalDegreesToRadians(lon2 - lon1);
    double y = sin(lon_diff_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon_diff_rad);

    return ((int) convertRadiansToDecimalDegrees(atan2(y, x)) + 360) % 360;
}

#endif //ROUTE_EDITOR_GEOPOSITIONUTIL_H

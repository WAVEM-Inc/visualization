//
// Created by antique on 24. 3. 19.
//

#ifndef ROUTE_EDITOR_GEOPOSITIONUTIL_H
#define ROUTE_EDITOR_GEOPOSITIONUTIL_H

#include <cmath>
#include "struct/Position.h"


// 라디안으로 변환
double toRadians(double degree);

// 도로 변환
double toDegrees(double radian);

double getBearingBetweenPoints(double lat1, double lon1, double lat2, double lon2);

Position translateLatLng(double lat, double lng, double distance, double degree);

std::array<double, 2> convert_wgs84_to_utm(double lat, double lng);

std::array<double, 2> convert_utm_to_wgs84(double x, double y);


#endif //ROUTE_EDITOR_GEOPOSITIONUTIL_H

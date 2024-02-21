//
// Created by antique on 24. 2. 14.
//

#ifndef NODE_EDITOR_POSITION_H
#define NODE_EDITOR_POSITION_H

struct Position {
    double latitude;
    double longitude;

    Position() : latitude(0), longitude(0) {};
    Position(const double lat, const double lng) : latitude(lat), longitude(lng) {};
};



#endif //NODE_EDITOR_POSITION_H

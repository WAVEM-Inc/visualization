//
// Created by antique on 24. 3. 12.
//

#include "model/map_state_model.h"
#include "struct/ConfigFile.h"

MapStateModel::MapStateModel(QObject *parent) : QObject(parent), _zoomLevel(ConfigFile().zoomLevel), _centerLocation(ConfigFile().center) {}

void MapStateModel::updateCenterLocation(Position location) {
    _centerLocation = location;

    emit centerLocationChanged(_centerLocation);

}

void MapStateModel::updateCenterLocation(double lat, double lng) {
    _centerLocation.latitude = lat;
    _centerLocation.longitude = lng;

    emit centerLocationChanged(_centerLocation);
}

void MapStateModel::updateZoomLevel(int zoomLevel) {
    _zoomLevel = zoomLevel;

    emit zoomLevelChanged(_zoomLevel);
}

Position MapStateModel::getCenterLocation() {
    return _centerLocation;
}

int MapStateModel::getZoomLevel() {
    return _zoomLevel;
}



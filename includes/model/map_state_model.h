//
// Created by antique on 24. 3. 12.
//

#ifndef ROUTE_EDITOR_MAP_STATE_MODEL_H
#define ROUTE_EDITOR_MAP_STATE_MODEL_H


#include <QObject>
#include "struct/Position.h"

class MapStateModel : public QObject {
Q_OBJECT
public:
    static MapStateModel &getInstance() {
        static MapStateModel instance;
        return instance;
    }

    MapStateModel(const MapStateModel &) = delete;

    MapStateModel &operator=(const MapStateModel &) = delete;

public:
    void updateCenterLocation(Position location);

    void updateCenterLocation(double lat, double lng);

    void updateZoomLevel(int zoomLevel);

    Position getCenterLocation();

    int getZoomLevel();

signals:
    void centerLocationChanged(Position location);

    void zoomLevelChanged(int zoomLevel);

private:
    explicit MapStateModel(QObject *parent = nullptr);

private:
    Position _centerLocation;
    int _zoomLevel;
};


#endif //ROUTE_EDITOR_MAP_STATE_MODEL_H

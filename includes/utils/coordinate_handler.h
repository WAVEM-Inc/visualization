//
// Created by antique on 24. 2. 13.
//

#ifndef NODE_EDITOR_COORDINATE_HANDLER_H
#define NODE_EDITOR_COORDINATE_HANDLER_H


#include <QObject>
#include <iostream>
#include "model/Node.h"
#include "model/MapNode.h"
#include "viewmodel/map_node_view_model.h"

class CoordinateHandler : public QObject {
Q_OBJECT

public:
    explicit CoordinateHandler(QObject *parent = nullptr) : QObject(parent) {}

public slots:

    Q_INVOKABLE void onClickEvent(double lat, double lng) {
        MapNode node("", Position(lat, lng));
        MapNodeViewModel::Instance().notify(node);
    }
};


#endif //NODE_EDITOR_COORDINATE_HANDLER_H

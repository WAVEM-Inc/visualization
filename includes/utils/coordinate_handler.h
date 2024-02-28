//
// Created by antique on 24. 2. 13.
//

#ifndef NODE_EDITOR_COORDINATE_HANDLER_H
#define NODE_EDITOR_COORDINATE_HANDLER_H


#include <QObject>
#include <iostream>
#include "struct/Node.h"
#include "struct/MapNode.h"
#include "viewmodel/map_node_view_model.h"
#include "utils/file/file_manager.h"
#include "viewmodel/route_file_view_model.h"

class CoordinateHandler : public QObject {
Q_OBJECT

public:
    explicit CoordinateHandler(QObject *parent = nullptr) : QObject(parent) {}

public slots:

    Q_INVOKABLE void onClickEvent(double lat, double lng) {
        if (RouteFileViewModel::Instance().getSavableState()) {
            char buffer[100];
            count++;

            snprintf(buffer, sizeof(buffer), "test%d", count);
            MapNodeViewModel::Instance().add_map_node(std::string(buffer), Position(lat, lng));
        }
    }

    Q_INVOKABLE void onRightClickEvent(QString nodeId) {
        std::cout << nodeId.toStdString() << "\n";
        MapNodeViewModel::Instance().remove_map_node(nodeId.toStdString());
    }

    Q_INVOKABLE void onDragendEvent(QString nodeId, double lat, double lng) {
        MapNodeViewModel::Instance().edit_map_node(nodeId.toStdString(), Position(lat, lng));
        std::cout << "NodeID: " << nodeId.toStdString() << ", Position: " << lat << ", " << lng << "\n";
    }

private:
    int count = 0;
};


#endif //NODE_EDITOR_COORDINATE_HANDLER_H
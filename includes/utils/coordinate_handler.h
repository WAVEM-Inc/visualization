//
// Created by antique on 24. 2. 13.
//

#ifndef NODE_EDITOR_COORDINATE_HANDLER_H
#define NODE_EDITOR_COORDINATE_HANDLER_H


#include <QObject>
#include <iostream>
#include <QInputDialog>
#include "struct/Node.h"
#include "struct/MapNode.h"
#include "model/map_node_model.h"
#include "utils/file/file_manager.h"
#include "model/file_info_model.h"
#include "ui/dialog/node_type_dialog.h"
#include "enum/NodeType.h"

class CoordinateHandler : public QObject {
Q_OBJECT

public:
    explicit CoordinateHandler(QObject *parent = nullptr) : QObject(parent) {}

public slots:

    Q_INVOKABLE void onClickEvent(double lat, double lng) {
        if (MapNodeModel::getInstance().getAddModeUsability()) {
            bool ok;
            QString nodeId = QInputDialog::getText(nullptr, "새 노드 추가", "추가할 노드의 아이디를 입력하주세요.",
                                                   QLineEdit::Normal, "", &ok, Qt::WindowFlags());

            if (!ok || nodeId.isEmpty()) {
                return;
            }

            NodeTypeDialog  nodeTypeDialog;
            std::string typeName;

            if (nodeTypeDialog.exec() == QDialog::Accepted) {
                std::string selected = nodeTypeDialog.selectedOption();
                NodeType type = getTypeFromKorName(selected);
                typeName = getTypeName(type);
            }

            if (ok && MapNodeModel::getInstance().checkIdUsability(nodeId.toStdString())) {
                MapNode node;
                node.nodeId = nodeId.toStdString();
                node.type = typeName;
                node.position = Position(lat, lng);

                MapNodeModel::getInstance().addMapNode(node);
            }
        }

        MapNodeModel::getInstance().updateUseAddMode(false);
    }

    Q_INVOKABLE void onNodeClickEvent(QString nodeId) {
        MapNodeModel::getInstance().selectMapNode(nodeId.toStdString());
    }

    Q_INVOKABLE void onRightClickEvent(QString nodeId) {
/*        std::cout << nodeId.toStdString() << "\n";
        MapNodeViewModel::Instance().remove_map_node(nodeId.toStdString());*/
    }

    Q_INVOKABLE void onDragendEvent(QString nodeId, double lat, double lng) {
        MapNodeModel::getInstance().updateMapNode(nodeId.toStdString(), lat, lng);
    }

private:
    int count = 0;
};


#endif //NODE_EDITOR_COORDINATE_HANDLER_H
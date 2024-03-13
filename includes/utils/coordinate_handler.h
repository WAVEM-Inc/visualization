//
// Created by antique on 24. 2. 13.
//

#ifndef NODE_EDITOR_COORDINATE_HANDLER_H
#define NODE_EDITOR_COORDINATE_HANDLER_H


#include <QObject>
#include <iostream>
#include <QInputDialog>
#include <QMessageBox>
#include "struct/Node.h"
#include "struct/GraphNode.h"
#include "model/map_node_model.h"
#include "model/file_info_model.h"
#include "ui/dialog/node_type_dialog.h"
#include "model/map_state_model.h"

class CoordinateHandler : public QObject {
Q_OBJECT

public:
    explicit CoordinateHandler(QObject *parent = nullptr) : QObject(parent) {}

public slots:

    Q_INVOKABLE void onClickEvent(double lat, double lng) {
        if (FileInfoModel::getInstance().getFileSavable()) {
            bool ok;
            QString nodeId = QInputDialog::getText(nullptr, "새 노드 추가", "추가할 노드의 아이디를 입력하주세요.",
                                                   QLineEdit::Normal, "", &ok, Qt::WindowFlags());

            if (!ok || nodeId.isEmpty()) {
                return;
            }

            NodeTypeDialog nodeTypeDialog;
            std::string typeCode;

            if (nodeTypeDialog.exec() == QDialog::Accepted) {
                typeCode = nodeTypeDialog.selectedOption();
            }

            if (ok && MapNodeModel::getInstance().checkIdUsability(nodeId.toStdString())) {
                GraphNode node;
                node.nodeId = "NO-" + FileInfoModel::getInstance().getFileInfo().mapId + "-" + nodeId.toStdString();
                node.nodeName = node.nodeId;
                node.type = typeCode;
                node.position = Position(lat, lng);

                MapNodeModel::getInstance().addMapNode(node);
            }
        } else {
            QMessageBox msgBox;
            msgBox.setWindowTitle(QString("선택된 파일 없음"));
            msgBox.setText(QString("선택된 파일이 없습니다.\n파일을 새로 만들거나 존재하는 파일을\n 불러온 후 다시시도해주세요."));
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.setDefaultButton(QMessageBox::Ok);
            msgBox.exec();
        }
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

    Q_INVOKABLE void onMouseScrollEnded(QString locationStr) {
        nlohmann::json json = nlohmann::json::parse(locationStr.toStdString());
        double lat = json["lat"];
        double lng = json["lng"];

        MapStateModel::getInstance().updateCenterLocation(lat, lng);
    }

    Q_INVOKABLE void onWheelScrollEnded(QString zoomLevelStr) {
        int zoomLevel = zoomLevelStr.toInt();

        MapStateModel::getInstance().updateZoomLevel(zoomLevel);
    }

private:
    int count = 0;
};


#endif //NODE_EDITOR_COORDINATE_HANDLER_H
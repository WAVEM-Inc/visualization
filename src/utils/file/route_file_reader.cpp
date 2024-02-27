//
// Created by antique on 24. 2. 26.
//

#include <qfile.h>
#include <QMessageBox>
#include <QTextStream>
#include "utils/file/route_file_reader.h"
#include "nlohmann/json.hpp"
#include "model/RouteFile.h"

#include "utils/file/file_manager.h"
#include "viewmodel/path_view_model.h"
#include "viewmodel/map_node_view_model.h"

RouteFileReader::RouteFileReader(QObject *parent) : QObject(parent){}

bool RouteFileReader::loadFile(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(nullptr, "실패", "파일을 불러오지 못했습니다. 다시 시도해 주세요");

        return false;
    }

    QTextStream in(&file);
    QString fileContent = in.readAll();

    nlohmann::json json = nlohmann::json::parse(fileContent.toStdString());
    RouteFile fileData = json.get<RouteFile>();

    FileManager::Instance().updateOriginFileData(fileData);
    PathViewModel::Instance().updatePaths(fileData.path);
    MapNodeViewModel::Instance().update_map_nodes(fileData.node);

    return true;
}

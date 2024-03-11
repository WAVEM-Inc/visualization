//
// Created by antique on 24. 2. 26.
//

#include <qfile.h>
#include <QMessageBox>
#include <QTextStream>
#include "utils/file/route_file_reader.h"
#include "nlohmann/json.hpp"
#include "struct/RouteFile.h"
#include "model/file_info_model.h"
#include "utils/patterns/singleton/singleton.h"
#include "model/path_info_model.h"
#include "model/map_node_model.h"
#include "model/node_info_model.h"

RouteFileReader::RouteFileReader(QObject *parent) : QObject(parent){
    connect(&FileInfoModel::getInstance(), &FileInfoModel::fileInfoChanged, this, &RouteFileReader::onFileInfoChanged);
}

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

    FileInfo info;
    info.filePath = filePath.toStdString();
    info.fileVersion = fileData.version;
    info.mapId = fileData.mapId;

    FileInfoModel::getInstance().updateFileInfo(info);
    PathInfoModel::getInstance().setPathInfoMap(fileData.path);
    QMap<std::string, QList<Node>> nodes;
    for (const auto &path : fileData.path) {
        nodes.insert(path.id, QList<Node>::fromVector(QVector<Node>::fromStdVector(path.nodelist)));
    }
    NodeInfoModel::getInstance().updateNodes(nodes);
    MapNodeModel::getInstance().setMapNodes(fileData.node);

    return true;
}

void RouteFileReader::onFileInfoChanged(const FileInfo &info) {
    std::cout << info.filePath << "\n";
}

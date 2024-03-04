//
// Created by antique on 24. 2. 26.
//

#include <qfileinfo.h>
#include <QMessageBox>
#include <QTextStream>
#include "utils/file/file_manager.h"
#include "utils/file/route_file_writer.h"
#include "utils/patterns/singleton/singleton.h"
#include "model/file_info_model.h"
#include "model/path_info_model.h"
#include "model/map_node_model.h"


RouteFileWriter::RouteFileWriter(QObject *parent) : QObject(parent) {

}

bool RouteFileWriter::saveFile(const QString &filePath, const RouteFile &routeFileData) {
    QString finalPath = filePath;
    QFileInfo fileInfo(finalPath);

    // 확장자 확인 및 추가
    if (fileInfo.suffix().compare("dat", Qt::CaseInsensitive) != 0) {
        finalPath += ".dat";
        fileInfo.setFile(finalPath); // 확장자가 추가된 최종 경로로 QFileInfo 업데이트
    }


    // 파일이 이미 존재하는 경우 덮어쓰기 여부 확인
    if (fileInfo.exists()) {
        auto response = QMessageBox::question(
                nullptr,
                tr("덮어쓰기"),
                tr("이미 존재하는 파일입니다. 덮어 쓰시겠습니까?"),
                QMessageBox::Yes | QMessageBox::No
        );

        if (response == QMessageBox::No) {
            // 사용자가 'No'를 선택한 경우, 저장 작업을 취소합니다.
            return false;
        }
    }

    // 파일 저장
    QFile file(finalPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Failed to save file: %1").arg(finalPath));
        return false;
    }

    nlohmann::json dataJson = routeFileData;
    QString textData = QString(dataJson.dump(4).c_str());

    QTextStream out(&file);
    out << textData;
    file.close();

    if (out.status() == QTextStream::Ok) {
        FileInfo info;
        info.filePath = finalPath.toStdString();
        info.fileVersion = routeFileData.fileVersion;
        info.mapId = routeFileData.mapId;

        FileInfoModel::getInstance().updateFileInfo(info);
        PathInfoModel::getInstance().setPathInfoMap(routeFileData.path);
        MapNodeModel::getInstance().setMapNodes(routeFileData.node);

    } else {
        QMessageBox::warning(nullptr, "실패", "파일 저장에 실패하였습니다.");

        return false;
    }

    std::cout << "Saved path: " << finalPath.toStdString() << "\n";

    return true;
}

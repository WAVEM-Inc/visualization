//
// Created by antique on 24. 2. 29.
//

#include "model/file_info_model.h"

FileInfoModel::FileInfoModel(QObject *parent) : QObject(parent) {

}

void FileInfoModel::updateFileInfo(const FileInfo &info) {
    _fileInfo = info;
    emit fileInfoChanged(_fileInfo);

    updateLatestFilePath(info.filePath);

    if (!_fileInfo.fileVersion.empty()) {
        emit fileSavableChanged(true);
    } else {
        emit fileSavableChanged(false);
    }
}

FileInfo FileInfoModel::getFileInfo() const {
    return _fileInfo;
}

void FileInfoModel::updateFileSavable(bool savable) {
    _fileSavable = savable;
}

bool FileInfoModel::getFileSavable() const {
    return _fileSavable;
}

std::string FileInfoModel::getLatestFilePath() const {
    return _latestFilePath;
}

void FileInfoModel::updateLatestFilePath(const std::string &path) {
    _latestFilePath = path;
    emit latestFilePathChanged(_latestFilePath);
}

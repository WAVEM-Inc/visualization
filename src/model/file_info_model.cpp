//
// Created by antique on 24. 2. 29.
//

#include "model/file_info_model.h"

FileInfoModel::FileInfoModel(QObject *parent) : QObject(parent) {

}

void FileInfoModel::updateFileInfo(const FileInfo &info) {
    _fileInfo = info;
    emit fileInfoChanged(_fileInfo);

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

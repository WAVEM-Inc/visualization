//
// Created by antique on 24. 2. 29.
//

#include "model/path_info_model.h"


PathInfoModel::PathInfoModel(QObject *parent) : QObject(parent) {}

void PathInfoModel::setPathInfoMap(const QMap<std::string, std::string>& pathInfoMap) {
    _pathInfoMap = pathInfoMap;
    emit pathInfoMapChanged(_pathInfoMap);
}

void PathInfoModel::setPathInfoMap(const std::vector<Path> &paths) {
    clearPathInfoMap();

    for (const Path &path : paths) {
        _pathInfoMap.insert(path.id, path.name);
    }

    emit  pathInfoMapChanged(_pathInfoMap);
}

void PathInfoModel::addPathInfo(const std::string& id, const std::string& name) {
    _pathInfoMap.insert(id, name);
    emit pathInfoMapChanged(_pathInfoMap);
}

bool PathInfoModel::updatePathInfo(const std::string& id, const std::string& name) {
    if (!_pathInfoMap.contains(id)) {
        return false;
    }

    _pathInfoMap.insert(id, name);
    emit pathInfoMapChanged(_pathInfoMap);

    return true;
}

void PathInfoModel::selectCurrentPathId(const std::string &pathId) {
    if (_pathInfoMap.contains(pathId)) {
        _currentPathId = pathId;
        emit currentPathIdChanged(_currentPathId);
    }
}

std::string PathInfoModel::getCurrentPathId() {
    return _currentPathId;
}

void PathInfoModel::clearPathInfoMap() {
    _pathInfoMap.clear();
}

std::string PathInfoModel::getCurrentPathName() const {
    return _pathInfoMap[_currentPathId];
}

QMap<std::string, std::string> PathInfoModel::getPathInfoMap() const {
    return _pathInfoMap;
}

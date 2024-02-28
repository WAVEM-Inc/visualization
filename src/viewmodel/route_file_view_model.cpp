//
// Created by antique on 24. 2. 28.
//

#include "viewmodel/route_file_view_model.h"

#include <utility>

RouteFileViewModel::RouteFileViewModel() :
        m_fileInfo_ptr(std::make_shared<Subject<FileInfo>>()),
        m_originFile_ptr(std::make_shared<Subject<RouteFile>>()),
        m_savable_ptr(std::make_shared<Subject<bool>>()) {

}

void RouteFileViewModel::updateOriginFile(const std::string &filePath, const RouteFile &file) {
    FileInfo info;
    info.filePath = filePath;
    info.fileVersion = file.fileVersion;
    info.mapId = file.mapId;

    m_fileInfo_ptr->notify(info);
    m_originFile_ptr->notify(file);

    if (!file.fileVersion.empty()) {
        updateSavableState(true);
    } else {
        updateSavableState(false);
    }
}

bool RouteFileViewModel::getSavableState() {
    return m_savable_ptr->value();
}

void RouteFileViewModel::attachToSavable(std::shared_ptr<Observer<bool>> observer) {
    m_savable_ptr->attach(std::move(observer));
}

void RouteFileViewModel::detachToSavable(std::shared_ptr<Observer<bool>> observer) {
    m_savable_ptr->detach(std::move(observer));
}

void RouteFileViewModel::updateSavableState(bool savable) {
    m_savable_ptr->notify(savable);
}



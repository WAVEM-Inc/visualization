//
// Created by antique on 24. 2. 23.
//

#include "utils/file/file_manager.h"
#include "utils/events/FileEvents.h"

/*FileManager::FileManager() :
        m_originFileData_ptr(std::make_shared<RouteFile>()),
        m_savable_state(std::make_shared<Subject<bool>>()) {}


void FileManager::updateOriginFileData(const RouteFile &routeFile) {
    m_originFileData_ptr = std::make_shared<RouteFile>(routeFile);

    if (!(m_originFileData_ptr->fileVersion.empty() && m_originFileData_ptr->mapId.empty())) {
        m_savable_state->notify(true);
    } else {
        m_savable_state->notify(false);
    }
}

RouteFile FileManager::getOriginFileData() {
    return *m_originFileData_ptr;
}

std::shared_ptr<Subject<bool>> FileManager::savableState() {
    return m_savable_state;
}*/

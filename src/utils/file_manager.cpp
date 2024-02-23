//
// Created by antique on 24. 2. 23.
//

#include "utils/file_manager.h"
#include "utils/events/FileEvents.h"

FileManager::FileManager() :
        m_bus_ptr(std::make_shared<dexode::EventBus>()),
        m_originFileData_ptr(std::make_shared<RouteFile>()),
        m_cacheFileData_ptr(std::make_shared<RouteFile>()) {}

std::shared_ptr<dexode::EventBus> &FileManager::getChannel() {
    return m_bus_ptr;
}

void FileManager::updateOriginFileData(const RouteFile &routeFile) {
    m_originFileData_ptr = std::make_shared<RouteFile>(routeFile);
}

void FileManager::updateCacheFileData(const RouteFile &routeFile) {
    m_cacheFileData_ptr = std::make_shared<RouteFile>(routeFile);
/*    if (!(m_cacheFileData_ptr->fileVersion.empty() && m_cacheFileData_ptr->mapId.empty())) {
        m_bus_ptr->postpone(event::FILE_SAVABLE{true});
    } else {
        m_bus_ptr->postpone(event::FILE_SAVABLE{false});
    }
    m_bus_ptr->process();*/
}

RouteFile FileManager::getOriginFileData() {
    return *m_originFileData_ptr;
}

RouteFile FileManager::getCacheFileData() {
    return *m_cacheFileData_ptr;
}

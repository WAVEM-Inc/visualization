//
// Created by antique on 24. 2. 23.
//

#ifndef NODE_EDITOR_FILE_MANAGER_H
#define NODE_EDITOR_FILE_MANAGER_H


#include "utils/patterns/singleton/singleton.h"
#include "dexode/EventBus.hpp"
#include "model/RouteFile.h"

class FileManager : public Singleton<FileManager> {
    friend class Singleton<FileManager>;

public:
    FileManager();

    std::shared_ptr<dexode::EventBus> &getChannel();

    void updateOriginFileData(const RouteFile &routeFile);

    void updateCacheFileData(const RouteFile &routeFile);

    RouteFile getOriginFileData();

    RouteFile getCacheFileData();

private:
    std::shared_ptr<dexode::EventBus> m_bus_ptr;
    std::shared_ptr<RouteFile> m_originFileData_ptr;
    std::shared_ptr<RouteFile> m_cacheFileData_ptr;
};


#endif //NODE_EDITOR_FILE_MANAGER_H

//
// Created by antique on 24. 2. 23.
//

#ifndef NODE_EDITOR_FILE_MANAGER_H
#define NODE_EDITOR_FILE_MANAGER_H


#include "utils/patterns/singleton/singleton.h"
#include "struct/RouteFile.h"
#include "utils/patterns/observer/subject.h"

/*class FileManager : public Singleton<FileManager> {
    friend class Singleton<FileManager>;

public:
    FileManager();

    void updateOriginFileData(const RouteFile &routeFile);

    RouteFile getOriginFileData();

    std::shared_ptr<Subject<bool>> savableState();

private:
    std::shared_ptr<RouteFile> m_originFileData_ptr;
    std::shared_ptr<Subject<bool>> m_savable_state;
};*/


#endif //NODE_EDITOR_FILE_MANAGER_H

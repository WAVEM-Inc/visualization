//
// Created by antique on 24. 2. 28.
//

#ifndef NODE_EDITOR_ROUTE_FILE_VIEW_MODEL_H
#define NODE_EDITOR_ROUTE_FILE_VIEW_MODEL_H


#include "utils/patterns/singleton/singleton.h"
#include "utils/patterns/observer/subject.h"
#include "model/FileInfo.h"
#include "model/RouteFile.h"


class RouteFileViewModel : public Singleton<RouteFileViewModel> {
    friend class Singleton<RouteFileViewModel>;

public:
    RouteFileViewModel();

    void updateOriginFile(const std::string& filePath, const RouteFile& file);

    void updateSavableState(bool savable);

    bool getSavableState();

    void attachToSavable(std::shared_ptr<Observer<bool>> observer);

    void detachToSavable(std::shared_ptr<Observer<bool>> observer);

private:
    std::shared_ptr<Subject<FileInfo>> m_fileInfo_ptr;
    std::shared_ptr<Subject<RouteFile>> m_originFile_ptr;
    std::shared_ptr<Subject<bool>> m_savable_ptr;
};


#endif //NODE_EDITOR_ROUTE_FILE_VIEW_MODEL_H

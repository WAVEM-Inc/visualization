//
// Created by antique on 24. 2. 28.
//

#ifndef NODE_EDITOR_PATH_VIEW_MODEL_H
#define NODE_EDITOR_PATH_VIEW_MODEL_H


#include <map>
#include "utils/patterns/singleton/singleton.h"
#include "utils/patterns/observer/subject.h"
#include "model/Path.h"

class PathViewModel : public Singleton<PathViewModel> {
    friend class Singleton<PathViewModel>;

public:
    explicit PathViewModel();

    virtual ~PathViewModel();

    void updatePathMap(std::map<std::string, Path> pathMap);

    void updatePathMap(const std::vector<Path>& paths);

    void addPath(const Path& path);

    void removePath(const std::string &pathId);

    std::map<std::string, Path> getPathMap();

    void selectCurrentPath(const std::string& pathId);

    void updateCurrentPath(const Path &path);

    Path getCurrentPath();

    void addNode(const Node &node);

    void editNode(const Node &node);

    void removeNode(const std::string &nodeId);

    void selectNode(const Node &node);

    void selectNode(const std::string &nodeId);

    void updateCurrentNode(const Node &node);

    void attachToPathMap(std::shared_ptr<Observer<std::map<std::string, Path>>> observer);

    void detachToPathMap(std::shared_ptr<Observer<std::map<std::string, Path>>> observer);

    void attachToCurrentPath(std::shared_ptr<Observer<Path>> observer);

    void detachToCurrentPath(std::shared_ptr<Observer<Path>> observer);

private:
    std::shared_ptr<Subject<std::map<std::string, Path>>> m_pathMap_ptr;
    std::shared_ptr<Subject<Path>> m_currentPath_ptr;
    std::shared_ptr<Subject<Node>> m_currentNode_ptr;
};


#endif //NODE_EDITOR_PATH_VIEW_MODEL_H

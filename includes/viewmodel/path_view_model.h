//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_PATH_VIEW_MODEL_H
#define NODE_EDITOR_PATH_VIEW_MODEL_H


#include "utils/patterns/singleton/singleton.h"
#include "model/Path.h"
#include "utils/patterns/observer/subject.h"

class PathViewModel : public Singleton<PathViewModel> {
    friend class Singleton<PathViewModel>;

public:
    PathViewModel();

    void addPath(const Path& path);

    void removePath(const std::string& pathId);

    void updatePaths(const std::vector<Path>& paths);

    void updatePaths(std::map<std::string, Path> paths);

    void updateCurrentPath(const std::string& pathId);

    void addNodeToCurrentPath(const Node& node);

    void attachPathsObserver(std::shared_ptr<Observer<std::map<std::string, Path>>> observer);

private:
    std::shared_ptr<Subject<std::map<std::string, Path>>> m_paths_ptr;
    std::shared_ptr<Subject<Path>> m_current_path;
};


#endif //NODE_EDITOR_PATH_VIEW_MODEL_H

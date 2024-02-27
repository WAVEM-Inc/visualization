//
// Created by antique on 24. 2. 27.
//

#include "viewmodel/path_view_model.h"

#include <utility>

PathViewModel::PathViewModel() :
        m_current_path(std::make_shared<Subject<Path>>()),
        m_paths_ptr(std::make_shared<Subject<std::map<std::string, Path>>>()) {

}

void PathViewModel::updatePaths(const std::vector<Path>& paths) {
    std::map<std::string, Path> pathsMap;
    for (const Path& path : paths) {
        pathsMap.insert({path.id, path});
    }

    m_paths_ptr->notify(pathsMap);
}

void PathViewModel::updatePaths(std::map<std::string, Path> paths) {
    m_paths_ptr->notify(std::move(paths));
}

void PathViewModel::addPath(const Path& path) {
    std::map<std::string, Path> paths = m_paths_ptr->value();
    paths.insert({path.id, path});
    m_paths_ptr->notify(paths);
}

void PathViewModel::removePath(const std::string& pathId) {
    std::map<std::string, Path> paths =  m_paths_ptr->value();
    paths.erase(pathId);
    m_paths_ptr->notify(paths);
}

void PathViewModel::updateCurrentPath(const std::string& pathId) {
    std::map<std::string, Path> paths = m_paths_ptr->value();
    m_current_path->notify(paths[pathId]);
}

void PathViewModel::addNodeToCurrentPath(const Node& node) {
    Path path = m_current_path->value();
    path.nodelist.push_back(node);
}

void PathViewModel::attachPathsObserver(std::shared_ptr<Observer<std::map<std::string, Path>>> observer) {
    m_paths_ptr->attach(std::move(observer));
}

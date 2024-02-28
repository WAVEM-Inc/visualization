//
// Created by antique on 24. 2. 28.
//

#include "viewmodel/path_view_model.h"

#include <utility>

PathViewModel::PathViewModel() :
        m_pathMap_ptr(std::make_shared<Subject<std::map<std::string, Path>>>()),
        m_currentPath_ptr(std::make_shared<Subject<Path>>()),
        m_currentNode_ptr(std::make_shared<Subject<Node>>()){

}

PathViewModel::~PathViewModel() = default;

void PathViewModel::updatePathMap(std::map<std::string, Path> pathMap) {
    m_pathMap_ptr->notify(std::move(pathMap));
}

void PathViewModel::updatePathMap(const std::vector<Path>& paths) {
    std::map<std::string, Path> pathMap;
    for (const Path& path: paths) {
        pathMap.insert({path.id, path});
    }

    m_pathMap_ptr->notify(pathMap);
}

void PathViewModel::addPath(const Path& path) {
    std::map<std::string, Path> pathMap = getPathMap();
    pathMap.insert({path.id, path});

    m_pathMap_ptr->notify(pathMap);
}

void PathViewModel::removePath(const std::string &pathId) {
    std::map<std::string, Path> pathMap = getPathMap();
    pathMap.erase(pathId);

    m_pathMap_ptr->notify(pathMap);
}

std::map<std::string, Path> PathViewModel::getPathMap() {
    return m_pathMap_ptr->value();
}

void PathViewModel::selectCurrentPath(const std::string &pathId) {
    std::map<std::string, Path> pathMap = getPathMap();
    m_currentPath_ptr->notify(pathMap[pathId]);
}

Path PathViewModel::getCurrentPath() {
    return m_currentPath_ptr->value();
}


void PathViewModel::updateCurrentPath(const Path &path) {
    std::map<std::string, Path> pathMap = getPathMap();
    pathMap[path.id] = path;

    m_currentPath_ptr->notify(path);
    m_pathMap_ptr->notify(pathMap);
}

void PathViewModel::addNode(const Node &node) {
    Path path = getCurrentPath();
    path.nodelist.push_back(node);

    updateCurrentPath(path);
}


void PathViewModel::editNode(const Node &node) {
    Path path = getCurrentPath();

    auto it = std::find_if(path.nodelist.begin(), path.nodelist.end(),
                           [&node](const Node& n) {
                               return n.nodeId == node.nodeId;
                           });

    if (it != path.nodelist.end()) {
        *it = node;
    }

    updateCurrentPath(path);
}

void PathViewModel::removeNode(const std::string &nodeId) {
    Path path = getCurrentPath();
    auto newEnd = std::remove_if(path.nodelist.begin(), path.nodelist.end(),
                                 [&nodeId](const Node& node) {
                                     return node.nodeId == nodeId;
                                 });
    path.nodelist.erase(newEnd, path.nodelist.end());

    updateCurrentPath(path);
}

void PathViewModel::selectNode(const Node &node) {
    m_currentNode_ptr->notify(node);
}


void PathViewModel::selectNode(const std::string &nodeId) {
    Path path = getCurrentPath();

    auto it = std::find_if(path.nodelist.begin(), path.nodelist.end(),
                           [&nodeId](const Node& node) {
                               return node.nodeId == nodeId;
                           });

    if (it != path.nodelist.end()) {
        Node selectedNode = *it;
        selectNode(selectedNode);
    }
}

void PathViewModel::attachToPathMap(std::shared_ptr<Observer<std::map<std::string, Path>>> observer) {
    m_pathMap_ptr->attach(std::move(observer));
}

void PathViewModel::detachToPathMap(std::shared_ptr<Observer<std::map<std::string, Path>>> observer) {
    m_pathMap_ptr->detach(std::move(observer));
}

void PathViewModel::attachToCurrentPath(std::shared_ptr<Observer<Path>> observer) {
    m_currentPath_ptr->attach(std::move(observer));
}

void PathViewModel::detachToCurrentPath(std::shared_ptr<Observer<Path>> observer) {
    m_currentPath_ptr->detach(std::move(observer));
}

void PathViewModel::updateCurrentNode(const Node &node) {
    Path path = getCurrentPath();

    Node currentNode = m_currentNode_ptr->value();
    if (currentNode.nodeId == node.nodeId) {
        editNode(node);
    }
}

//
// Created by antique on 24. 2. 29.
//

#include "model/node_info_model.h"
#include "model/path_info_model.h"

NodeInfoModel::NodeInfoModel(QObject *parent) : QObject(parent) {

}

bool NodeInfoModel::addNodeToCurrentPath(const Node &node) {
    std::string pathId = PathInfoModel::getInstance().getCurrentPathId();

    if (pathId.empty()) {
        return false;
    }

    _nodesMap[pathId].insert(node.nodeId, node);
    emit nodesMapChanged(_nodesMap);

    return true;
}

bool NodeInfoModel::removeNodeFromCurrentPath(const std::string &nodeId) {
    std::string  pathId = PathInfoModel::getInstance().getCurrentPathId();

    if (pathId.empty()) {
        return false;
    }

    _nodesMap[pathId].remove(nodeId);
    emit nodesMapChanged(_nodesMap);

    return false;
}

QMap<std::string, Node> NodeInfoModel::getNodesFromCurrentPath() {
    std::string pathId = PathInfoModel::getInstance().getCurrentPathId();
    return _nodesMap[pathId];
}

bool NodeInfoModel::selectCurrentNode(const std::string &nodeId) {
    std::string pathId = PathInfoModel::getInstance().getCurrentPathId();

    if (!_nodesMap.contains(pathId)) {
        return false;
    }

    QMap<std::string, Node> nodes = _nodesMap.value(pathId);

    if (!nodes.contains(nodeId)) {
        return false;
    }

    _currentNode = nodes[nodeId];
    emit currentNodeChanged(_currentNode);

    return true;
}

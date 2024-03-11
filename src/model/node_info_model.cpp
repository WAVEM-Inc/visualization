//
// Created by antique on 24. 2. 29.
//

#include "model/node_info_model.h"
#include "model/path_info_model.h"

NodeInfoModel::NodeInfoModel(QObject *parent) : QObject(parent) {
    connect(&PathInfoModel::getInstance(), &PathInfoModel::currentPathIdChanged, this, [this](const std::string &pathId) {
       QList<Node> nodes = _nodesMap[pathId];
       _currentNodeList = nodes;
       emit currentNodeListChanged(_currentNodeList);
    });
}

void NodeInfoModel::updateNodes(const QMap<std::string, QList<Node>>& nodesMap) {
    _nodesMap = nodesMap;
    emit nodesMapChanged(_nodesMap);

    _currentNodeList = _nodesMap[PathInfoModel::getInstance().getCurrentPathId()];
    emit currentNodeListChanged(_currentNodeList);
}

bool NodeInfoModel::addNodeToCurrentPath(const Node &node) {
    std::string pathId = PathInfoModel::getInstance().getCurrentPathId();
    if (pathId.empty()) {
        return false;
    }

    QList<Node>& nodeList = _nodesMap[pathId];

    // 동일한 nodeId를 가진 노드가 이미 리스트에 있는지 확인합니다.
    for (const Node& existingNode : nodeList) {
        if (existingNode.nodeId == node.nodeId) {
            // 동일한 nodeId를 가진 노드가 이미 존재합니다.
            return false; // 중복이므로 추가하지 않고 함수를 종료합니다.
        }
    }

    nodeList.push_back(node);

    // 변경 사항을 신호로 알립니다.
    emit nodesMapChanged(_nodesMap);

    _currentNodeList = nodeList;
    emit currentNodeListChanged(_currentNodeList);

    return true;
}

bool NodeInfoModel::removeNodeFromCurrentPath(int index) {
    std::string pathId = PathInfoModel::getInstance().getCurrentPathId();
    if (pathId.empty()) {
        return false;
    }

    _nodesMap[pathId].removeAt(index);
    emit nodesMapChanged(_nodesMap);

    _currentNodeList = _nodesMap[pathId];
    emit currentNodeListChanged(_currentNodeList);

    return false;
}

bool NodeInfoModel::selectCurrentNode(int index) {
    std::string pathId = PathInfoModel::getInstance().getCurrentPathId();
    if (pathId.empty()) {
        return false;
    }

    _currentNodeIndex = index;
    _currentNode = _nodesMap[pathId][index];
    emit currentNodeChanged(_currentNode);

    return false;
}

bool NodeInfoModel::updateCurrentNode(const Node &node) {
    if (node.nodeId == _currentNode.nodeId) {
        _currentNode = node;
        emit currentNodeChanged(_currentNode);

        _nodesMap[PathInfoModel::getInstance().getCurrentPathId()][_currentNodeIndex] = _currentNode;
        emit nodesMapChanged(_nodesMap);
    }

    return true;
}

QList<Node> NodeInfoModel::getNodesFromCurrentPath() const {
    return _currentNodeList;
}

QMap<std::string, QList<Node>> NodeInfoModel::getAllNodes() const {
    return _nodesMap;
}

std::string NodeInfoModel::getPreNodeId() {
    if (_currentNodeIndex > 0 && _currentNodeIndex <= _currentNodeList.size()) {
        std::cout << "Current Node Index: " << _currentNodeIndex << "\n";
        return _currentNodeList[_currentNodeIndex - 1].nodeId;
    }
    return "";
}

std::string NodeInfoModel::getNextNodeId() {
    if (_currentNodeIndex >= 0 && _currentNodeIndex < _currentNodeList.size() - 1) {
        std::cout << "Current Node Index: " << _currentNodeIndex << "\n";
        return _currentNodeList[_currentNodeIndex + 1].nodeId;
    }
    return "";
}

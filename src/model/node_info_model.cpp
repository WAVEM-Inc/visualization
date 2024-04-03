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
    connect(&PathInfoModel::getInstance(), &PathInfoModel::pathInfoMapChanged, this, [this](const QMap<std::string , std::string> &pathInfoMap) {
        for (const std::string &pathId: _nodesMap.keys()) {
            if (!pathInfoMap.contains(pathId)) {
                _nodesMap.remove(pathId);
            }
        }
    });
}

void NodeInfoModel::updateNodes(const QMap<std::string, QList<Node>>& nodesMap) {
    _nodesMap.clear();

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

        std::string pathId = PathInfoModel::getInstance().getCurrentPathId();
        _currentNodeList = _nodesMap[pathId];
        emit currentNodeListChanged(_currentNodeList);
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

bool NodeInfoModel::changeIndexes(int sourceIndex, int beginIndex) {
    try {
        QList<Node> nodes = getNodesFromCurrentPath();

        // 인덱스 유효성 검사
        if (sourceIndex < 0 || sourceIndex >= nodes.size() || beginIndex < 0 || beginIndex >= nodes.size()) {
            return false; // 유효하지 않은 인덱스
        }

        // 노드 이동 로직
        Node nodeToMove = nodes.takeAt(sourceIndex); // sourceIndex 위치의 노드를 제거하고 반환
        nodes.insert(beginIndex, nodeToMove); // beginIndex 위치에 노드 삽입

        std::string pathId = PathInfoModel::getInstance().getCurrentPathId();

        _nodesMap[pathId] = nodes;
        emit nodesMapChanged(_nodesMap);

        _currentNodeList = nodes;
        emit currentNodeListChanged(_currentNodeList);

        // currentNodeIndex 업데이트 로직 개선 필요
        if (sourceIndex < beginIndex) {
            _currentNodeIndex = beginIndex - 1;
        } else {
            _currentNodeIndex = beginIndex;
        }

        return true; // 성공적으로 위치 변경

    } catch (const std::exception& e) {
        // 예외 처리 로직, 필요에 따라 로깅 등을 수행
        std::cout << "Exception caught in changeIndexes:" << e.what();
        return false; // 예외 발생 시 false 반환
    }
}

Node NodeInfoModel::getSelectedNode() const {
    return _currentNode;
}

Node NodeInfoModel::getNextNode() const {
    if (_currentNodeIndex >= 0 && _currentNodeIndex < _currentNodeList.size() - 1) {
        std::cout << "Current Node Index: " << _currentNodeIndex << "\n";
        return _currentNodeList[_currentNodeIndex + 1];
    }

    throw std::out_of_range("getNextNode() called with _currentNodeIndex out of valid range.");
}

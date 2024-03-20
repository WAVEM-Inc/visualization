//
// Created by antique on 24. 2. 29.
//

#include "model/map_node_model.h"
#include "model/path_info_model.h"
#include "model/node_info_model.h"

MapNodeModel::MapNodeModel(QObject *parent) : QObject(parent) {
    connect(this, &MapNodeModel::showAllNodesOptionChanged, this, [this](bool showAll) {
        showNodes(showAll);
    });
    connect(&PathInfoModel::getInstance(), &PathInfoModel::currentPathIdChanged, this,
            [this](const std::string &pathId) {
                showNodes(_showAllNodes);
            });
    connect(this, &MapNodeModel::mapNodesChanged, this, [this](const QMap<std::string, GraphNode> &nodeMap) {
        showNodes(_showAllNodes);
    });
}

void MapNodeModel::setMapNodes(const QMap<std::string, GraphNode> &mapNodes) {
    _mapNodes.clear();

    _mapNodes = mapNodes;
    emit mapNodesChanged(_mapNodes);

    showNodes(_showAllNodes);
}

void MapNodeModel::setMapNodes(const std::vector<GraphNode> &mapNodes) {
    _mapNodes.clear();
    _showingNodes.clear();

    QMap<std::string, GraphNode> nodes;
    for (const GraphNode &node: mapNodes) {
        nodes.insert(node.nodeId, node);
    }

    _mapNodes = nodes;
    emit mapNodesChanged(_mapNodes);

    showNodes(_showAllNodes);
}

void MapNodeModel::updateMapNode(const std::string &nodeId, double lat, double lng) {
    QMap<std::string, GraphNode> nodes = getMapNodes();
    GraphNode node = nodes[nodeId];
    node.position.latitude = lat;
    node.position.longitude = lng;

    nodes[nodeId] = node;
    _mapNodes = nodes;
    emit mapNodesChanged(_mapNodes);
}

void MapNodeModel::addMapNode(const GraphNode &node) {
    _mapNodes.insert(node.nodeId, node);
    emit mapNodesChanged(_mapNodes);
}

void MapNodeModel::removeMapNode(const std::string &nodeId) {
    _mapNodes.remove(nodeId);
    emit mapNodesChanged(_mapNodes);
}

GraphNode MapNodeModel::getMapNodeById(const std::string &nodeId) const {
    return _mapNodes[nodeId];
}


QMap<std::string, GraphNode> MapNodeModel::getMapNodes() const {
    return _mapNodes;
}

bool MapNodeModel::checkIdUsability(const std::string &nodeId) {
    if (_mapNodes.contains(nodeId)) {
        return false;
    } else {
        return true;
    }
}

void MapNodeModel::selectMapNode(const std::string &nodeId) {
    _selectedMapNode_ptr = getMapNodeById(nodeId);
    emit selectedMapNodeChanged(_selectedMapNode_ptr);
}

GraphNode MapNodeModel::getSelectedMapNode() const {
    return _selectedMapNode_ptr;
}

void MapNodeModel::setChangeCenterToSelectedNode(bool use) {
    _changeCenterToSelectedNode = use;
}

bool MapNodeModel::getChangeCenterToSelectedNode() const {
    return _changeCenterToSelectedNode;
}

void MapNodeModel::setShowAllNodes(bool show) {
    _showAllNodes = show;
    emit showAllNodesOptionChanged(_showAllNodes);
}

bool MapNodeModel::getShowAllNodes() const {
    return _showAllNodes;
}

void MapNodeModel::showNodes(bool showAll) {
    try {
        if (showAll) {
            _showingNodes.clear();
            for (const GraphNode &node: _mapNodes.values()) {
                _showingNodes.push_back(node);
            }

            emit showingNodesChanged(_showingNodes);
        } else {
            _showingNodes.clear();

            QList<Node> currentNodes = NodeInfoModel::getInstance().getNodesFromCurrentPath();
            for (const Node &node: currentNodes) {
                _showingNodes.push_back(_mapNodes[node.nodeId]);
            }

            emit showingNodesChanged(_showingNodes);
        }
    } catch (std::exception &exception) {
        std::cout << "Error occured in showAllNodeOptionChanged event: " << exception.what() << "\n";
    }
}

//
// Created by antique on 24. 2. 29.
//

#include "model/map_node_model.h"

MapNodeModel::MapNodeModel(QObject *parent) : QObject(parent) {}

void MapNodeModel::setMapNodes(const QMap<std::string, MapNode> &mapNodes) {
    _mapNodes = mapNodes;
    emit mapNodesChanged(_mapNodes);
}

void MapNodeModel::setMapNodes(const std::vector<MapNode> &mapNodes) {
    QMap<std::string, MapNode> nodes;
    for (const MapNode &node : mapNodes) {
        nodes.insert(node.nodeId, node);
    }

    _mapNodes = nodes;
    emit mapNodesChanged(_mapNodes);
}

void MapNodeModel::updateMapNode(const std::string &nodeId, double lat, double lng) {
    QMap<std::string, MapNode> nodes = getMapNodes();
    MapNode node = nodes[nodeId];
    node.position.latitude = lat;
    node.position.longitude = lng;

    nodes[nodeId] = node;
    _mapNodes = nodes;
    emit mapNodesChanged(_mapNodes);
}

void MapNodeModel::addMapNode(const MapNode &node) {
    _mapNodes.insert(node.nodeId, node);
    emit mapNodesChanged(_mapNodes);
}

void MapNodeModel::removeMapNode(const std::string &nodeId) {
    _mapNodes.remove(nodeId);
    emit mapNodesChanged(_mapNodes);
}

MapNode MapNodeModel::getMapNodeById(const std::string &nodeId) const {
    return _mapNodes[nodeId];
}


QMap<std::string, MapNode> MapNodeModel::getMapNodes() const {
    return _mapNodes;
}

bool MapNodeModel::checkIdUsability(const std::string &nodeId) {
    if (_mapNodes.contains(nodeId)) {
        return false;
    } else {
        return true;
    }
}

void MapNodeModel::updateUseAddMode(bool usable) {
    _useAddMode = usable;
    emit useAddModeChanged(_useAddMode);
}

bool MapNodeModel::getAddModeUsability() const {
    return _useAddMode;
}

void MapNodeModel::selectMapNode(const std::string &nodeId) {
    _selectedMapNode_ptr = getMapNodeById(nodeId);
    emit selectedMapNodeChanged(_selectedMapNode_ptr);
}

MapNode MapNodeModel::getSelectedMapNode() const {
    return _selectedMapNode_ptr;
}

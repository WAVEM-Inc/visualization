//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_MAP_NODE_MODEL_H
#define NODE_EDITOR_MAP_NODE_MODEL_H


#include <QObject>
#include <QMap>
#include "struct/GraphNode.h"

class MapNodeModel : public QObject {
Q_OBJECT

public:
    static MapNodeModel &getInstance() {
        static MapNodeModel instance;
        return instance;
    }

    MapNodeModel(const MapNodeModel &) = delete;

    MapNodeModel &operator=(const MapNodeModel &) = delete;

    void setMapNodes(const QMap<std::string, GraphNode>& mapNodes);

    void setMapNodes(const std::vector<GraphNode> &mapNodes);

    void updateMapNode(const std::string &nodeId, double lat, double lng);

    void addMapNode(const GraphNode &node);

    void removeMapNode(const std::string& nodeId);

    GraphNode getMapNodeById(const std::string &nodeId) const;

    void selectMapNode(const std::string &nodeId);

    GraphNode getSelectedMapNode() const;

    QMap<std::string, GraphNode> getMapNodes() const;

    bool checkIdUsability(const std::string& nodeId);

    void setChangeCenterToSelectedNode(bool use);

    bool getChangeCenterToSelectedNode() const;

    void setShowAllNodes(bool show);

    bool getShowAllNodes() const;

    void showNodes(bool showAll);

signals:
    void mapNodesChanged(const QMap<std::string, GraphNode> &nodeMap);

    void selectedMapNodeChanged(const GraphNode &mapNode);

    void showAllNodesOptionChanged(bool show);

    void showingNodesChanged(std::vector<GraphNode> showingNodes);

private:
    explicit MapNodeModel(QObject *parent = nullptr);

private:
    QMap<std::string, GraphNode> _mapNodes;
    std::vector<GraphNode> _showingNodes;
    GraphNode _selectedMapNode_ptr;
    bool _changeCenterToSelectedNode;
    bool _showAllNodes;
};


#endif //NODE_EDITOR_MAP_NODE_MODEL_H

//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_MAP_NODE_MODEL_H
#define NODE_EDITOR_MAP_NODE_MODEL_H


#include <QObject>
#include <QMap>
#include "struct/MapNode.h"

class MapNodeModel : public QObject {
Q_OBJECT

public:
    static MapNodeModel &getInstance() {
        static MapNodeModel instance;
        return instance;
    }

    MapNodeModel(const MapNodeModel &) = delete;

    MapNodeModel &operator=(const MapNodeModel &) = delete;

    void setMapNodes(const QMap<std::string, MapNode>& mapNodes);

    void setMapNodes(const std::vector<MapNode> &mapNodes);

    void updateMapNode(const std::string &nodeId, double lat, double lng);

    void addMapNode(const MapNode &node);

    void removeMapNode(const std::string& nodeId);

    MapNode getMapNodeById(const std::string &nodeId) const;

    void selectMapNode(const std::string &nodeId);

    MapNode getSelectedMapNode() const;

    QMap<std::string, MapNode> getMapNodes() const;

    bool checkIdUsability(const std::string& nodeId);

    void updateUseAddMode(bool usable);

    bool getAddModeUsability() const;

signals:
    void mapNodesChanged(const QMap<std::string, MapNode> &nodeMap);

    void selectedMapNodeChanged(const MapNode &mapNode);

    void useAddModeChanged(bool usable);

private:
    explicit MapNodeModel(QObject *parent = nullptr);

private:
    QMap<std::string, MapNode> _mapNodes;
    MapNode _selectedMapNode_ptr;
    bool _useAddMode = false;
};


#endif //NODE_EDITOR_MAP_NODE_MODEL_H

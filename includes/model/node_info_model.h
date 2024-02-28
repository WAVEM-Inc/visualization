//
// Created by antique on 24. 2. 29.
//

#ifndef NODE_EDITOR_NODE_INFO_MODEL_H
#define NODE_EDITOR_NODE_INFO_MODEL_H


#include <QObject>
#include <qmap.h>
#include "struct/Node.h"

class NodeInfoModel : public QObject {
Q_OBJECT

public:
    static NodeInfoModel &getInstance() {
        static NodeInfoModel instance;
        return instance;
    }

    NodeInfoModel(const NodeInfoModel &) = delete;

    NodeInfoModel &operator=(const NodeInfoModel &) = delete;

    bool addNodeToCurrentPath(const Node &node);

    bool removeNodeFromCurrentPath(const std::string &nodeId);

    bool selectCurrentNode(const std::string &nodeId);

    bool updateCurrentNode(const Node &node);

    QMap<std::string, Node> getNodesFromCurrentPath();

signals:
    void nodesMapChanged(const QMap<std::string, QMap<std::string, Node>> &nodesMap);

    void currentNodeChanged(const Node &node);

private:
    explicit NodeInfoModel(QObject *parent = nullptr);

private:
    QMap<std::string, QMap<std::string, Node>> _nodesMap; // key: pathId, value: {key: nodeId, value: node}
    Node _currentNode;
};


#endif //NODE_EDITOR_NODE_INFO_MODEL_H

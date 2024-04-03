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

    void updateNodes(const QMap<std::string, QList<Node>>& nodesMap);

    bool addNodeToCurrentPath(const Node &node);

    bool removeNodeFromCurrentPath(int index);

    bool selectCurrentNode(int index);

    bool updateCurrentNode(const Node &node);

    bool changeIndexes(int sourceIndex, int beginIndex);

    Node getSelectedNode() const;

    std::string getPreNodeId();

    Node getNextNode() const;

    std::string getNextNodeId();

    QList<Node> getNodesFromCurrentPath() const;

    QMap<std::string, QList<Node>> getAllNodes() const;

signals:
    void nodesMapChanged(const QMap<std::string, QList<Node>> &nodesMap);

    void currentNodeListChanged(const QList<Node> &nodeList);

    void currentNodeChanged(const Node &node);

private:
    explicit NodeInfoModel(QObject *parent = nullptr);

private:
    QMap<std::string, QList<Node>> _nodesMap; // key: pathId, value: list<Node>
    QList<Node> _currentNodeList;
    Node _currentNode;
    int _currentNodeIndex;
};


#endif //NODE_EDITOR_NODE_INFO_MODEL_H

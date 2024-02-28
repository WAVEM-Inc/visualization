//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_NODE_LIST_MODEL_H
#define NODE_EDITOR_NODE_LIST_MODEL_H


#include <QAbstractListModel>
#include "model/Node.h"

class NodeListModel : public QAbstractListModel {
Q_OBJECT
public:
    explicit NodeListModel(QObject *parent = nullptr);

    void setNodes(const std::vector<Node> nodes);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

private:
    QList<Node> m_nodes;
};


#endif //NODE_EDITOR_NODE_LIST_MODEL_H

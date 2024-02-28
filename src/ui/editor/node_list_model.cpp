//
// Created by antique on 24. 2. 27.
//

#include "ui/editor/node_list_model.h"

NodeListModel::NodeListModel(QObject *parent) : QAbstractListModel(parent) {

}

void NodeListModel::setNodes(const std::vector<Node> nodes) {
    for (const Node &node : nodes) {
        beginInsertRows(QModelIndex(), rowCount(), rowCount());
        m_nodes << node;
        endInsertRows();
    }
}

int NodeListModel::rowCount(const QModelIndex &parent) const {
    Q_UNUSED(parent);
    return m_nodes.count();
}

QVariant NodeListModel::data(const QModelIndex &index, int role) const {
    if (index.row() < 0 || index.row() >= m_nodes.count()) {
        return QVariant();
    }

    if (role == Qt::DisplayRole) {
        const Node &node = m_nodes[index.row()];

        return QString("%1: ID=%2, Type=%3")
        .arg(QString::number(index.row() + 1))
        .arg(QString(node.nodeId.c_str()))
        .arg(QString(node.type.c_str()));
    }

    return QVariant();
}

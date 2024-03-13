//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_NODE_LIST_MODEL_H
#define NODE_EDITOR_NODE_LIST_MODEL_H


#include <QAbstractListModel>
#include <QStandardItemModel>
#include "struct/Node.h"

class NodeListModel : public QStandardItemModel {
Q_OBJECT
public:
    explicit NodeListModel(QObject *parent = nullptr);

    ~NodeListModel() override;

    QMimeData * mimeData(const QModelIndexList &indexes) const override;

    bool dropMimeData(const QMimeData *data, Qt::DropAction action, int row, int column, const QModelIndex &parent) override;

protected:
    mutable QModelIndex _dragStartIndex;
};


#endif //NODE_EDITOR_NODE_LIST_MODEL_H

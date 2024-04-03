//
// Created by antique on 24. 3. 12.
//

#ifndef ROUTE_EDITOR_NODE_LIST_VIEW_H
#define ROUTE_EDITOR_NODE_LIST_VIEW_H


#include <QTableView>

class NodeListView : public QTableView {
Q_OBJECT
public:
    explicit NodeListView(QWidget *parent = nullptr);

    ~NodeListView() override;

protected:
    void mousePressEvent(QMouseEvent *event) override;

    void dropEvent(QDropEvent *event) override;

private:
    QModelIndex _dragStartIndex;
};


#endif //ROUTE_EDITOR_NODE_LIST_VIEW_H

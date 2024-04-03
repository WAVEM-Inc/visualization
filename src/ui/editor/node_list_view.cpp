//
// Created by antique on 24. 3. 12.
//

#include "ui/editor/node_list_view.h"

void NodeListView::mousePressEvent(QMouseEvent *event) {
/*    if (event->buttons() == Qt::LeftButton) {
        _dragStartIndex = indexAt(event->pos());
    }*/
    QAbstractItemView::mousePressEvent(event);
}

void NodeListView::dropEvent(QDropEvent *event) {
    QAbstractItemView::dropEvent(event);
}

NodeListView::~NodeListView() {

}

NodeListView::NodeListView(QWidget *parent) : QTableView(parent) {

}

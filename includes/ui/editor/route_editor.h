//
// Created by antique on 24. 2. 26.
//

#ifndef NODE_EDITOR_ROUTE_EDITOR_H
#define NODE_EDITOR_ROUTE_EDITOR_H


#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QTableWidget>
#include <QGridLayout>
#include <QListView>
#include <QStandardItemModel>
#include "struct/RouteFile.h"
#include "utils/patterns/observer/observer.h"

class RouteEditor : public QWidget {
Q_OBJECT

public:
    explicit RouteEditor(QWidget *parent = nullptr);

    void resizeEvent(QResizeEvent *event) override;

private:
    QGridLayout *m_layout_ptr;
    QComboBox *m_routeComboBox_ptr;
    QTableView *_nodeListView_ptr;
    QStandardItemModel *_nodeListModel_ptr;
    QPushButton *m_addRouteButton_ptr;
    QPushButton *m_addNodeButton_ptr;

private slots:
    void onAddRouteButtonClicked();

    void onAddNodeButtonClicked();

private slots:
    void onFileSavableChanged(bool savable);
    void onPathInfoMapChanged(const QMap<std::string, std::string>& pathInfoMap);
    void onCurrentNodeListChanged(const QList<Node> &nodeList);
};


#endif //NODE_EDITOR_ROUTE_EDITOR_H

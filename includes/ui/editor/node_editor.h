//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_NODE_EDITOR_H
#define NODE_EDITOR_NODE_EDITOR_H


#include <QWidget>
#include <QTabWidget>

class NodeEditor : public QWidget {
Q_OBJECT

public:
    explicit NodeEditor(QWidget *parent = nullptr);

    void resizeEvent(QResizeEvent *event) override;


protected:
    void init();

    void initTabWidget();

    void initRouteInfoWidget();

    void initNodeInfoWidget();

    void initTaskInfoWidget();

private:
    QTabWidget *m_tab_ptr;
    QWidget *m_route_info_ptr;
    QWidget *m_node_info_ptr;
    QWidget *m_task_info_ptr;
};


#endif //NODE_EDITOR_NODE_EDITOR_H

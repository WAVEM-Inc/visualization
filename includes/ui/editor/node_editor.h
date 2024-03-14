//
// Created by antique on 24. 2. 27.
//

#ifndef NODE_EDITOR_NODE_EDITOR_H
#define NODE_EDITOR_NODE_EDITOR_H


#include <QWidget>
#include <QTabWidget>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include "struct/Node.h"
#include "detection_range_list_view.h"

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

    QLineEdit *_pathId_ptr;
    QLineEdit *_pathName_ptr;

    QLineEdit *_nodeId_ptr;
    QLineEdit *_nodeType_ptr;
    QLineEdit *_preNode_ptr;
    QLineEdit *_nextNode_ptr;

    QComboBox *_nodeKind_ptr;
    QComboBox *_nodeDirection_ptr;
    QLineEdit *_nodeHeading_ptr;
    DetectionRangeListView *_dtrListView_ptr;
    QPushButton *_addRangeBtn_ptr;
    QLineEdit *_nodeLat_ptr;
    QLineEdit *_nodeLng_ptr;

    QPushButton *_mapPoseBtn_ptr;
    QPushButton *_vehiclePoseBtn_ptr;
    QPushButton *_cancelBtn_ptr;
    QPushButton *_okBtn_ptr;

    Node _node;
};


#endif //NODE_EDITOR_NODE_EDITOR_H

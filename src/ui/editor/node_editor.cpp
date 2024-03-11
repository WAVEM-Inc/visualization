//
// Created by antique on 24. 2. 27.
//

#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QScrollArea>
#include <QPushButton>
#include "ui/editor/node_editor.h"
#include "model/file_info_model.h"
#include "model/path_info_model.h"
#include "model/node_info_model.h"
#include "enum/NodeKind.h"
#include "enum/Direction.h"
#include "ui/editor/detection_range_view.h"

NodeEditor::NodeEditor(QWidget *parent) :
        QWidget(parent),
        m_tab_ptr(new QTabWidget(this)),
        m_route_info_ptr(new QWidget()),
        m_node_info_ptr(new QWidget()),
        m_task_info_ptr(new QWidget()) {

    init();
}

void NodeEditor::init() {
    initRouteInfoWidget();
    initNodeInfoWidget();
    initTaskInfoWidget();

    initTabWidget();

    connect(&NodeInfoModel::getInstance(), &NodeInfoModel::currentNodeChanged, this, [this](const Node &node) {
        if (node.nodeId.empty()) {
            this->setVisible(false);
        } else {
            this->setVisible(true);
        }
        _node = node;

        _nodeId_ptr->setText(node.nodeId.c_str());
        _nodeType_ptr->setText(node.type.c_str());
        _preNode_ptr->setText(NodeInfoModel::getInstance().getPreNodeId().c_str());
        _nextNode_ptr->setText(NodeInfoModel::getInstance().getNextNodeId().c_str());
        _nodeKind_ptr->setCurrentText(node.kind.c_str());
        _nodeDirection_ptr->setCurrentText(node.direction.c_str());
        _nodeHeading_ptr->setText(QString::number(node.heading));
        _nodeLat_ptr->setText(QString::number(node.position.latitude));
        _nodeLng_ptr->setText(QString::number(node.position.longitude));
        _dtrListView_ptr->setDetectionRanges(node.detectionRange);
    });

    // setup widget events

}

void NodeEditor::initTabWidget() {
    // Node tab - add info widgets
    QWidget *nodeTab = new QWidget();

    QVBoxLayout *nodeLayout = new QVBoxLayout();
    nodeLayout->addWidget(m_route_info_ptr);
    nodeLayout->addWidget(m_node_info_ptr);
    nodeLayout->addWidget(m_task_info_ptr);

    // Node tab - add latlng buttons
    QWidget *latlngBtnWidget = new QWidget();
    QHBoxLayout *latlngBtnLayout = new QHBoxLayout();

    _mapPoseBtn_ptr = new QPushButton("지도 위치로 지정");
    _vehiclePoseBtn_ptr = new QPushButton("차량 위치로 지정");
    latlngBtnLayout->addWidget(_mapPoseBtn_ptr);
    latlngBtnLayout->addWidget(_vehiclePoseBtn_ptr);

    latlngBtnWidget->setLayout(latlngBtnLayout);
    nodeLayout->addWidget(latlngBtnWidget);


    // Node tab - add ok button
    _okBtn_ptr = new QPushButton("완료");
    _okBtn_ptr->setFixedWidth(150);
    nodeLayout->addStretch(1);
    nodeLayout->addWidget(_okBtn_ptr, 0, Qt::AlignRight);

    // Node tab - set scroll area
    QScrollArea *nodeScrollArea = new QScrollArea();
    nodeScrollArea->setWidget(nodeTab);
    nodeScrollArea->setWidgetResizable(true);
    nodeTab->setLayout(nodeLayout);

    // Initialize Link tab
    QWidget *linkTab = new QWidget();


    m_tab_ptr->addTab(nodeScrollArea, "Node");
    m_tab_ptr->addTab(linkTab, "Link");
}

void NodeEditor::initRouteInfoWidget() {
    QGridLayout *layout = new QGridLayout(m_route_info_ptr);

    QLabel *title = new QLabel("경로 정보");
    title->setStyleSheet("QLabel { font-size: 14pt; }");
    layout->addWidget(title, 0, 0);

    QLabel *pathIdLb = new QLabel("경로 ID");
    _pathId_ptr = new QLineEdit();
    layout->addWidget(pathIdLb, 1, 0);
    layout->addWidget(_pathId_ptr, 1, 1);

    QLabel *pathNameLb = new QLabel("경로 이름");
    _pathName_ptr = new QLineEdit();
    layout->addWidget(pathNameLb, 2, 0);
    layout->addWidget(_pathName_ptr, 2, 1);

    m_route_info_ptr->setLayout(layout);
}

void NodeEditor::initNodeInfoWidget() {
    QGridLayout *layout = new QGridLayout(m_node_info_ptr);

    QLabel *title = new QLabel("노드 정보");
    title->setStyleSheet("QLabel { font-size: 14pt; }");
    layout->addWidget(title, 0, 0);

    QLabel *nodeIdLb = new QLabel("노드 ID");
    _nodeId_ptr = new QLineEdit();
    _nodeId_ptr->setReadOnly(true);
    layout->addWidget(nodeIdLb, 1, 0);
    layout->addWidget(_nodeId_ptr, 1, 1);

    QLabel *nodeTypeLb = new QLabel("노드 타입");
    _nodeType_ptr = new QLineEdit();
    _nodeType_ptr->setReadOnly(true);
    layout->addWidget(nodeTypeLb, 2, 0);
    layout->addWidget(_nodeType_ptr, 2, 1);

    QLabel *preNodeLb = new QLabel("이전 노드");
    _preNode_ptr = new QLineEdit();
    _preNode_ptr->setReadOnly(true);
    layout->addWidget(preNodeLb, 3, 0);
    layout->addWidget(_preNode_ptr, 3, 1);

    QLabel *nextNodeLb = new QLabel("다음 노드");
    _nextNode_ptr = new QLineEdit();
    _nextNode_ptr->setReadOnly(true);
    layout->addWidget(nextNodeLb, 4, 0);
    layout->addWidget(_nextNode_ptr, 4, 1);

    m_node_info_ptr->setLayout(layout);
}

void NodeEditor::initTaskInfoWidget() {
    QGridLayout *layout = new QGridLayout(m_task_info_ptr);

    QLabel *title = new QLabel("작업 정보");
    title->setStyleSheet("QLabel { font-size: 14pt; }");
    layout->addWidget(title, 0, 0);

    QLabel *kindLb = new QLabel("작업 구분");
    _nodeKind_ptr = new QComboBox();
    std::vector<NodeKind> kinds = {NodeKind::INTERSECTION, NodeKind::CONNECTING, NodeKind::ENDPOINT, NodeKind::WAITING};
    for (const NodeKind &kind: kinds) {
        _nodeKind_ptr->addItem(QString(getKindKorName(kind).c_str()));
    }
    layout->addWidget(kindLb, 1, 0);
    layout->addWidget(_nodeKind_ptr, 1, 1);

    QLabel *directionLb = new QLabel("진출 방향");
    _nodeDirection_ptr = new QComboBox();
    _nodeDirection_ptr->addItem(QString(getDirectionKorName(Direction::FORWARD).c_str()));
    _nodeDirection_ptr->addItem(QString(getDirectionKorName(Direction::BACKWARD).c_str()));
    layout->addWidget(directionLb, 2, 0);
    layout->addWidget(_nodeDirection_ptr, 2, 1);

    QLabel *headingLb = new QLabel("차량 방향");
    _nodeHeading_ptr = new QLineEdit();
    layout->addWidget(headingLb, 3, 0);
    layout->addWidget(_nodeHeading_ptr, 3, 1);

    QLabel *detectionRangeLb = new QLabel("감지 범위");
    _addRangeBtn_ptr = new QPushButton("+");
    _addRangeBtn_ptr->setFixedSize(30, 30);
    _dtrListView_ptr =  new DetectionRangeListView();
    layout->addWidget(detectionRangeLb, 4, 0);
    layout->addWidget(_addRangeBtn_ptr, 4, 1, Qt::AlignLeft);
    layout->addWidget(_dtrListView_ptr, 5, 0, 1, 2);

    QLabel *latitudeLb = new QLabel("위도");
    _nodeLat_ptr = new QLineEdit();
    layout->addWidget(latitudeLb, 6, 0);
    layout->addWidget(_nodeLat_ptr, 6, 1);

    QLabel *longitudeLb = new QLabel("경도");
    _nodeLng_ptr = new QLineEdit();
    layout->addWidget(longitudeLb, 7, 0);
    layout->addWidget(_nodeLng_ptr, 7, 1);

    m_task_info_ptr->setLayout(layout);
}

void NodeEditor::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);

    m_tab_ptr->resize(this->width(), this->height());
}


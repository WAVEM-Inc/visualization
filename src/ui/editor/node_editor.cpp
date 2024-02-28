//
// Created by antique on 24. 2. 27.
//

#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QScrollArea>
#include <QPushButton>
#include "ui/editor/node_editor.h"

NodeEditor::NodeEditor(QWidget *parent) :
QWidget(parent),
m_tab_ptr(new QTabWidget(this)),
m_route_info_ptr(new QWidget()),
m_node_info_ptr(new QWidget()),
m_task_info_ptr(new QWidget()){

    init();
}

void NodeEditor::init() {
    initRouteInfoWidget();
    initNodeInfoWidget();
    initTaskInfoWidget();

    initTabWidget();
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

    QPushButton *mapLatLngBtn = new QPushButton("지도에서 선택");
    QPushButton *vehicleLatLngBtn = new QPushButton("차량 위치 선택");
    latlngBtnLayout->addWidget(mapLatLngBtn);
    latlngBtnLayout->addWidget(vehicleLatLngBtn);

    latlngBtnWidget->setLayout(latlngBtnLayout);
    nodeLayout->addWidget(latlngBtnWidget);


    // Node tab - add ok button
    QPushButton *okBtn = new QPushButton("완료");
    okBtn->setFixedWidth(150);
    nodeLayout->addStretch(1);
    nodeLayout->addWidget(okBtn, 0, Qt::AlignRight);

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

    QLabel *versionLb = new QLabel("버전");
    QLineEdit *versionEd = new QLineEdit();
    layout->addWidget(versionLb, 1, 0);
    layout->addWidget(versionEd, 1, 1);

    QLabel *separationLb = new QLabel("경로 구분");
    QLineEdit *separationEd = new QLineEdit();
    layout->addWidget(separationLb, 2, 0);
    layout->addWidget(separationEd, 2, 1);

    QLabel *codeLb = new QLabel("경로 코드");
    QLineEdit *codeEd = new QLineEdit();
    layout->addWidget(codeLb, 3, 0);
    layout->addWidget(codeEd, 3, 1);

    m_route_info_ptr->setLayout(layout);
}

void NodeEditor::initNodeInfoWidget() {
    QGridLayout *layout = new QGridLayout(m_node_info_ptr);

    QLabel *title = new QLabel("노드 정보");
    title->setStyleSheet("QLabel { font-size: 14pt; }");
    layout->addWidget(title, 0, 0);

    QLabel *nodeIdLb = new QLabel("노드 ID");
    QLineEdit *nodeIdEd = new QLineEdit();
    layout->addWidget(nodeIdLb, 1, 0);
    layout->addWidget(nodeIdEd, 1, 1);

    QLabel *nodeNameLb = new QLabel("노드 이름");
    QLineEdit *nodeNameEd = new QLineEdit();
    layout->addWidget(nodeNameLb, 2, 0);
    layout->addWidget(nodeNameEd, 2, 1);

    QLabel * nodeTypeLb = new QLabel("노드 타입");
    QLineEdit *nodeTypeEd = new QLineEdit();
    layout->addWidget(nodeTypeLb, 3, 0);
    layout->addWidget(nodeTypeEd, 3, 1);

    QLabel *preNodeLb = new QLabel("이전 노드");
    QLineEdit  *preNodeEd = new QLineEdit();
    layout->addWidget(preNodeLb, 4, 0);
    layout->addWidget(preNodeEd, 4, 1);

    QLabel *nextNodeLb = new QLabel("다음 노드");
    QLineEdit *nextNodeEd = new QLineEdit();
    layout->addWidget(nextNodeLb, 5, 0);
    layout->addWidget(nextNodeEd, 5, 1);

    m_node_info_ptr->setLayout(layout);
}

void NodeEditor::initTaskInfoWidget() {
    QGridLayout *layout = new QGridLayout(m_task_info_ptr);

    QLabel *title = new QLabel("작업 정보");
    title->setStyleSheet("QLabel { font-size: 14pt; }");
    layout->addWidget(title, 0, 0);

    QLabel *taskClassificationLb = new QLabel("작업 구분");
    QLineEdit *taskClassificationEd = new QLineEdit();
    layout->addWidget(taskClassificationLb, 1, 0);
    layout->addWidget(taskClassificationEd, 1, 1);

    QLabel *moveDirectionLb = new QLabel("진출 방향");
    QLineEdit *moveDirectionEd = new QLineEdit();
    layout->addWidget(moveDirectionLb, 2, 0);
    layout->addWidget(moveDirectionEd, 2, 1);

    QLabel *vehicleDirectionLb = new QLabel("차량 방향");
    QLineEdit *vehicleDirectionEd = new QLineEdit();
    layout->addWidget(vehicleDirectionLb, 3, 0);
    layout->addWidget(vehicleDirectionEd, 3, 1);

    QLabel *detectionRangeLb = new QLabel("감지 범위");
    QLineEdit *detectionRangeEd = new QLineEdit();
    layout->addWidget(detectionRangeLb, 4, 0);
    layout->addWidget(detectionRangeEd, 4, 1);

    QLabel *latitudeLb = new QLabel("경도");
    QLineEdit *latitudeEd = new QLineEdit();
    layout->addWidget(latitudeLb, 5, 0);
    layout->addWidget(latitudeEd, 5, 1);

    QLabel *longitudeLb = new QLabel("위도");
    QLineEdit *longitudeEd = new QLineEdit();
    layout->addWidget(longitudeLb, 6, 0);
    layout->addWidget(longitudeEd, 6, 1);

    m_task_info_ptr->setLayout(layout);
}

void NodeEditor::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);

    m_tab_ptr->resize(this->width(), this->height());
}


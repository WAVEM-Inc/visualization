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
#include "model/node_info_model.h"
#include "ui/editor/detection_range_view.h"
#include "model/path_info_model.h"
#include "model/code_info_model.h"
#include "enum/CodeType.h"
#include "utils/GeoPositionUtil.h"

NodeEditor::NodeEditor(QWidget *parent) :
        QWidget(parent),
        m_tab_ptr(new QTabWidget(this)),
        m_route_info_ptr(new QWidget()),
        m_node_info_ptr(new QWidget()),
        m_task_info_ptr(new QWidget()),
        _addRangeBtn_ptr(new QPushButton("+")),
        _dtrListView_ptr(new DetectionRangeListView()) {

    init();
}

void NodeEditor::init() {
    initRouteInfoWidget();
    initNodeInfoWidget();
    initTaskInfoWidget();

    initTabWidget();

    this->setVisible(false);
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
        _nodeKind_ptr->setCurrentText(
                CodeInfoModel::getInstance().getNameByCode(CodeType::NODE_KIND, node.kind).c_str());
        _nodeDirection_ptr->setCurrentText(
                CodeInfoModel::getInstance().getNameByCode(CodeType::DIRECTION, node.direction).c_str());
        _nodeHeading_ptr->setText(QString::number(node.heading));
        _nodeLat_ptr->setText(QString::number(node.position.latitude, 'f', 7));
        _nodeLng_ptr->setText(QString::number(node.position.longitude, 'f', 7));
        _dtrListView_ptr->setDetectionRanges(node.detectionRange);

        std::cout << "Data Kind: " << node.kind << "\n";
        std::cout << "View Kind: " << _nodeKind_ptr->currentText().toStdString() << "\n";
    });

    connect(&PathInfoModel::getInstance(), &PathInfoModel::currentPathIdChanged, this,
            [this](const std::string &pathId) {
                _pathId_ptr->setText(pathId.c_str());
                _pathName_ptr->setText(PathInfoModel::getInstance().getCurrentPathName().c_str());
                this->setVisible(false);
            }
    );

    connect(&CodeInfoModel::getInstance(), &CodeInfoModel::codeMapChanged, this,
            [this](const std::map<std::string, CodeGroup> &codeMap) {

                _nodeKind_ptr->clear();
                _nodeDirection_ptr->clear();

                std::vector<Code> nodeKindCodes = codeMap.at(CodeType::NODE_KIND).codes;
                for (const Code &code: nodeKindCodes) {
                    _nodeKind_ptr->addItem(
                            QString::fromStdString(code.name),
                            QVariant::fromValue(QString::fromStdString(code.code))
                    );
                }

                std::vector<Code> directionCodes = codeMap.at(CodeType::DIRECTION).codes;
                for (const Code &code: directionCodes) {
                    _nodeDirection_ptr->addItem(
                            QString::fromStdString(code.name),
                            QVariant::fromValue(QString::fromStdString(code.code))
                    );
                }
            }
    );

    // setup widget events
    connect(_cancelBtn_ptr, &QPushButton::clicked, this, [this]() {
        this->setVisible(false);
    });

    connect(_okBtn_ptr, &QPushButton::clicked, this, [this]() {
        Node node;
        node.position.latitude = _nodeLat_ptr->text().toDouble();
        node.position.longitude = _nodeLng_ptr->text().toDouble();
        node.nodeId = _nodeId_ptr->text().toStdString();
        node.type = _nodeType_ptr->text().toStdString();
        node.direction = _nodeType_ptr->text().toStdString();
        node.kind = _nodeKind_ptr->currentData().toString().toStdString();
        node.direction = _nodeDirection_ptr->currentData().toString().toStdString();
        node.heading = _nodeHeading_ptr->text().toInt();
        node.detectionRange = _dtrListView_ptr->getDetectionRanges();

        if (node.heading < 0) {
            try {
                Position curPos = node.position;
                Position nextPos = NodeInfoModel::getInstance().getNextNode().position;
                double bearing = 360 - getBearingBetweenPoints(curPos.latitude, curPos.longitude, nextPos.latitude, nextPos.longitude);
                node.heading = bearing;
            } catch (std::out_of_range &e) {
                std::cout << e.what() << "\n";
            }
        }

        NodeInfoModel::getInstance().updateCurrentNode(node);
    });

    connect(_addRangeBtn_ptr, &QPushButton::clicked, this, [this]() {
        DetectionRange range;
        _dtrListView_ptr->addDetectionRange(range);
    });
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
    QWidget *buttonsWidget = new QWidget();
    QHBoxLayout *buttonsLayout = new QHBoxLayout(buttonsWidget);

    _cancelBtn_ptr = new QPushButton("닫기");
    _okBtn_ptr = new QPushButton("저장");

    buttonsLayout->addWidget(_cancelBtn_ptr);
    buttonsLayout->addWidget(_okBtn_ptr);

    nodeLayout->addStretch(1);
    nodeLayout->addWidget(buttonsWidget);

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
    _pathId_ptr->setReadOnly(true);
    layout->addWidget(pathIdLb, 1, 0);
    layout->addWidget(_pathId_ptr, 1, 1);

    QLabel *pathNameLb = new QLabel("경로 이름");
    _pathName_ptr = new QLineEdit();
    _pathName_ptr->setReadOnly(true);
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

    layout->addWidget(kindLb, 1, 0);
    layout->addWidget(_nodeKind_ptr, 1, 1);

    QLabel *directionLb = new QLabel("진출 방향");
    _nodeDirection_ptr = new QComboBox();

    layout->addWidget(directionLb, 2, 0);
    layout->addWidget(_nodeDirection_ptr, 2, 1);

    QLabel *headingLb = new QLabel("차량 방향");
    _nodeHeading_ptr = new QLineEdit();
    _nodeHeading_ptr->setValidator(new QIntValidator(_nodeHeading_ptr));
    layout->addWidget(headingLb, 3, 0);
    layout->addWidget(_nodeHeading_ptr, 3, 1);

    QLabel *detectionRangeLb = new QLabel("감지 범위");
    _addRangeBtn_ptr->setFixedSize(30, 30);
    layout->addWidget(detectionRangeLb, 4, 0);
    layout->addWidget(_addRangeBtn_ptr, 4, 1, Qt::AlignLeft);
    layout->addWidget(_dtrListView_ptr, 5, 0, 1, 2);

    QLabel *latitudeLb = new QLabel("위도");
    _nodeLat_ptr = new QLineEdit();
    _nodeLat_ptr->setValidator(new QDoubleValidator(_nodeLat_ptr));
    layout->addWidget(latitudeLb, 6, 0);
    layout->addWidget(_nodeLat_ptr, 6, 1);

    QLabel *longitudeLb = new QLabel("경도");
    _nodeLng_ptr = new QLineEdit();
    _nodeLng_ptr->setValidator(new QDoubleValidator(_nodeLng_ptr));
    layout->addWidget(longitudeLb, 7, 0);
    layout->addWidget(_nodeLng_ptr, 7, 1);

    m_task_info_ptr->setLayout(layout);
}

void NodeEditor::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);

    m_tab_ptr->resize(this->width(), this->height());
}


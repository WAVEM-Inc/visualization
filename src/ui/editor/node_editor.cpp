//
// Created by antique on 24. 2. 27.
//

#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QScrollArea>
#include <QPushButton>
#include "ui/editor/node_editor.h"

#include <qpainter.h>

#include "model/file_info_model.h"
#include "model/node_info_model.h"
#include "ui/editor/detection_range_view.h"
#include "model/path_info_model.h"
#include "model/code_info_model.h"
#include "enum/CodeType.h"
#include "utils/GeoPositionUtil.h"
#include "model/ros_2_data_model.h"
#include "model/map_node_model.h"

NodeEditor::NodeEditor(QWidget* parent) :
    QWidget(parent),
    m_route_info_ptr(new QWidget()),
    m_node_info_ptr(new QWidget()),
    m_task_info_ptr(new QWidget()),
    _addRangeBtn_ptr(new QPushButton("+")),
    _dtrListView_ptr(new DetectionRangeListView())
{
    init();
}

void NodeEditor::init()
{
    initRouteInfoWidget();
    initNodeInfoWidget();
    initTaskInfoWidget();

    initLayout();

    this->setVisible(false);
    this->layout()->setContentsMargins(0, 0, 0, 0);
    connect(&NodeInfoModel::getInstance(), &NodeInfoModel::currentNodeChanged, this, [this](const Node& node)
    {
        if (node.nodeId.empty())
        {
            this->setVisible(false);
        }
        else
        {
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
        _drivingOption_ptr->setCurrentText(
            CodeInfoModel::getInstance().getNameByCode(CodeType::DRIVING_OPTION, node.drivingOption).c_str());
        _nodeHeading_ptr->setText(QString::number(node.heading));
        _nodeLat_ptr->setText(QString::number(node.position.latitude, 'f', 7));
        _nodeLng_ptr->setText(QString::number(node.position.longitude, 'f', 7));
        _dtrListView_ptr->setDetectionRanges(node.detectionRange);

        std::cout << "Data Kind: " << node.kind << "\n";
        std::cout << "View Kind: " << _nodeKind_ptr->currentText().toStdString() << "\n";
    });

    connect(&PathInfoModel::getInstance(), &PathInfoModel::currentPathIdChanged, this,
            [this](const std::string& pathId)
            {
                _pathId_ptr->setText(pathId.c_str());
                _pathName_ptr->setText(PathInfoModel::getInstance().getCurrentPathName().c_str());
                this->setVisible(false);
            }
    );

    connect(&CodeInfoModel::getInstance(), &CodeInfoModel::codeMapChanged, this,
            [this](const std::map<std::string, CodeGroup>& codeMap)
            {
                _nodeKind_ptr->clear();
                _nodeDirection_ptr->clear();
                _drivingOption_ptr->clear();

                std::vector<Code> nodeKindCodes = codeMap.at(CodeType::NODE_KIND).codes;
                for (const Code& code : nodeKindCodes)
                {
                    _nodeKind_ptr->addItem(
                        QString::fromStdString(code.name),
                        QVariant::fromValue(QString::fromStdString(code.code))
                    );
                }

                std::vector<Code> directionCodes = codeMap.at(CodeType::DIRECTION).codes;
                for (const Code& code : directionCodes)
                {
                    _nodeDirection_ptr->addItem(
                        QString::fromStdString(code.name),
                        QVariant::fromValue(QString::fromStdString(code.code))
                    );
                }

                std::vector<Code> drivingOptionCodes = codeMap.at(CodeType::DRIVING_OPTION).codes;
                for (const Code& code : drivingOptionCodes)
                {
                    _drivingOption_ptr->addItem(
                        QString::fromStdString(code.name),
                        QVariant::fromValue(QString::fromStdString(code.code))
                    );
                }
            }
    );

    // setup widget events
    connect(_cancelBtn_ptr, &QPushButton::clicked, this, [this]()
    {
        this->setVisible(false);
    });

    connect(_okBtn_ptr, &QPushButton::clicked, this, [this]()
    {
        Node node;
        node.position.latitude = _nodeLat_ptr->text().toDouble();
        node.position.longitude = _nodeLng_ptr->text().toDouble();
        node.nodeId = _nodeId_ptr->text().toStdString();
        node.type = _nodeType_ptr->text().toStdString();
        node.direction = _nodeType_ptr->text().toStdString();
        node.kind = _nodeKind_ptr->currentData().toString().toStdString();
        node.direction = _nodeDirection_ptr->currentData().toString().toStdString();
        node.drivingOption = _drivingOption_ptr->currentData().toString().toStdString();
        node.heading = _nodeHeading_ptr->text().toInt();
        node.detectionRange = _dtrListView_ptr->getDetectionRanges();

        // if (node.heading < 0) {
        //     try {
        //         Position curPos = node.position;
        //         Position nextPos = NodeInfoModel::getInstance().getNextNode().position;
        //         double bearing = 360 - getBearingBetweenPoints(curPos.latitude, curPos.longitude, nextPos.latitude,
        //                                                        nextPos.longitude);
        //         node.heading = bearing;
        //     } catch (std::out_of_range &e) {
        //         std::cout << e.what() << "\n";
        //     }
        // }

        NodeInfoModel::getInstance().updateCurrentNode(node);
    });

    connect(_addRangeBtn_ptr, &QPushButton::clicked, this, [this]()
    {
        DetectionRange range;
        _dtrListView_ptr->addDetectionRange(range);
    });

    connect(_vehiclePoseBtn_ptr, &QPushButton::clicked, this, [this]()
    {
        sensor_msgs::msg::NavSatFix nsf = ROS2DataModel::getInstance().getNavSatFixData();
        _nodeLat_ptr->setText(QString::number(nsf.latitude, 'f', 7));
        _nodeLng_ptr->setText(QString::number(nsf.longitude, 'f', 7));

        // try {
        //     Position curPos;
        //     curPos.latitude = nsf.latitude;
        //     curPos.longitude = nsf.longitude;
        //     Position nextPos = NodeInfoModel::getInstance().getNextNode().position;
        //     double bearing = 360 - getBearingBetweenPoints(curPos.latitude, curPos.longitude, nextPos.latitude,
        //                                                    nextPos.longitude);
        //     _nodeHeading_ptr->setText(QString::number(int(bearing)));
        // } catch (std::out_of_range &e) {
        //     std::cout << e.what() << "\n";
        // }
    });

    connect(_mapPoseBtn_ptr, &QPushButton::clicked, this, [this]()
    {
        std::string nodeId = NodeInfoModel::getInstance().getSelectedNode().nodeId;
        Position mapPosition = MapNodeModel::getInstance().getMapNodeById(nodeId).position;

        _nodeLat_ptr->setText(QString::number(mapPosition.latitude, 'f', 7));
        _nodeLng_ptr->setText(QString::number(mapPosition.longitude, 'f', 7));
    });

    connect(_calcHeadingBtn_ptr, &QPushButton::clicked, this, [this]()
    {
        try
        {
            double lat = _nodeLat_ptr->text().toDouble();
            double lng = _nodeLng_ptr->text().toDouble();

            Position curPos;
            curPos.latitude = lat;
            curPos.longitude = lng;
            Position nextPos = NodeInfoModel::getInstance().getNextNode().position;
            double bearing = 360 - getBearingBetweenPoints(curPos.latitude, curPos.longitude, nextPos.latitude,
                                                           nextPos.longitude);
            _nodeHeading_ptr->setText(QString::number(int(bearing)));
        }
        catch (std::out_of_range& e)
        {
            std::cout << e.what() << "\n";
        }
    });
}

void NodeEditor::initLayout()
{
    // 메인 레이아웃 설정
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 정보 위젯 추가
    mainLayout->addWidget(m_route_info_ptr);
    mainLayout->addWidget(m_node_info_ptr);
    mainLayout->addWidget(m_task_info_ptr);

    // 위도/경도 버튼 위젯
    QWidget* latlngBtnWidget = new QWidget();
    QHBoxLayout* latlngBtnLayout = new QHBoxLayout();

    _mapPoseBtn_ptr = new QPushButton("지도 위치로 지정");
    _vehiclePoseBtn_ptr = new QPushButton("차량 위치로 지정");
    latlngBtnLayout->addWidget(_mapPoseBtn_ptr);
    latlngBtnLayout->addWidget(_vehiclePoseBtn_ptr);

    latlngBtnWidget->setLayout(latlngBtnLayout);
    mainLayout->addWidget(latlngBtnWidget);

    // 확인/취소 버튼 위젯
    QWidget* buttonsWidget = new QWidget();
    QHBoxLayout* buttonsLayout = new QHBoxLayout(buttonsWidget);

    _cancelBtn_ptr = new QPushButton("닫기");
    _okBtn_ptr = new QPushButton("저장");

    buttonsLayout->addWidget(_cancelBtn_ptr);
    buttonsLayout->addWidget(_okBtn_ptr);

    mainLayout->addStretch(1);
    mainLayout->addWidget(buttonsWidget);

    // 스크롤 영역 설정 (필요한 경우)
    QScrollArea* scrollArea = new QScrollArea();
    QWidget* contentWidget = new QWidget(); // 스크롤 영역에 들어갈 콘텐츠 위젯
    contentWidget->setLayout(mainLayout); // 메인 레이아웃을 콘텐츠 위젯에 설정
    scrollArea->setWidget(contentWidget); // 스크롤 영역에 콘텐츠 위젯 설정
    scrollArea->setWidgetResizable(true);

    // 최종적으로 스크롤 영역을 this의 레이아웃으로 설정
    QVBoxLayout* thisLayout = new QVBoxLayout(this); // this의 새로운 메인 레이아웃
    thisLayout->addWidget(scrollArea); // 스크롤 영역 추가
}

/*void NodeEditor::initTabWidget() {
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
    m_tab_ptr->addTab(nodeScrollArea, "Node");
    m_tab_ptr->addTab(linkTab, "Link");
}*/

void NodeEditor::initRouteInfoWidget()
{
    QGridLayout* layout = new QGridLayout(m_route_info_ptr);

    QLabel* title = new QLabel("경로 정보");
    title->setStyleSheet("QLabel { font-size: 26pt; }");
    layout->addWidget(title, 0, 0);

    QLabel* pathIdLb = new QLabel("경로 ID");
    _pathId_ptr = new QLineEdit();
    _pathId_ptr->setReadOnly(true);
    layout->addWidget(pathIdLb, 1, 0);
    layout->addWidget(_pathId_ptr, 1, 1);

    QLabel* pathNameLb = new QLabel("경로 이름");
    _pathName_ptr = new QLineEdit();
    _pathName_ptr->setReadOnly(true);
    layout->addWidget(pathNameLb, 2, 0);
    layout->addWidget(_pathName_ptr, 2, 1);

    m_route_info_ptr->setLayout(layout);
}

void NodeEditor::initNodeInfoWidget()
{
    QGridLayout* layout = new QGridLayout(m_node_info_ptr);

    QLabel* title = new QLabel("노드 정보");
    title->setStyleSheet("QLabel { font-size: 26pt; }");
    layout->addWidget(title, 0, 0);

    QLabel* nodeIdLb = new QLabel("노드 ID");
    _nodeId_ptr = new QLineEdit();
    _nodeId_ptr->setReadOnly(true);
    layout->addWidget(nodeIdLb, 1, 0);
    layout->addWidget(_nodeId_ptr, 1, 1);

    QLabel* nodeTypeLb = new QLabel("노드 타입");
    _nodeType_ptr = new QLineEdit();
    _nodeType_ptr->setReadOnly(true);
    layout->addWidget(nodeTypeLb, 2, 0);
    layout->addWidget(_nodeType_ptr, 2, 1);

    QLabel* preNodeLb = new QLabel("이전 노드");
    _preNode_ptr = new QLineEdit();
    _preNode_ptr->setReadOnly(true);
    layout->addWidget(preNodeLb, 3, 0);
    layout->addWidget(_preNode_ptr, 3, 1);

    QLabel* nextNodeLb = new QLabel("다음 노드");
    _nextNode_ptr = new QLineEdit();
    _nextNode_ptr->setReadOnly(true);
    layout->addWidget(nextNodeLb, 4, 0);
    layout->addWidget(_nextNode_ptr, 4, 1);

    m_node_info_ptr->setLayout(layout);
}

void NodeEditor::initTaskInfoWidget()
{
    QGridLayout* layout = new QGridLayout(m_task_info_ptr);

    QLabel* title = new QLabel("작업 정보");
    title->setStyleSheet("QLabel { font-size: 26pt; }");
    layout->addWidget(title, 0, 0);

    QLabel* kindLb = new QLabel("작업 구분");
    _nodeKind_ptr = new QComboBox();
    layout->addWidget(kindLb, 1, 0);
    layout->addWidget(_nodeKind_ptr, 1, 1);

    QLabel* directionLb = new QLabel("진출 방향");
    _nodeDirection_ptr = new QComboBox();
    layout->addWidget(directionLb, 2, 0);
    layout->addWidget(_nodeDirection_ptr, 2, 1);


    QLabel* drivingOptionLb = new QLabel("위치 추정 방법");
    _drivingOption_ptr = new QComboBox();
    layout->addWidget(drivingOptionLb, 3, 0);
    layout->addWidget(_drivingOption_ptr, 3, 1);

    QLabel* headingLb = new QLabel("차량 방향");
    _nodeHeading_ptr = new QLineEdit();
    _nodeHeading_ptr->setValidator(new QIntValidator(_nodeHeading_ptr));
    layout->addWidget(headingLb, 4, 0);
    layout->addWidget(_nodeHeading_ptr, 4, 1);

    QLabel* detectionRangeLb = new QLabel("감지 범위");
    _addRangeBtn_ptr->setFixedSize(30, 30);
    layout->addWidget(detectionRangeLb, 5, 0);
    layout->addWidget(_addRangeBtn_ptr, 5, 1, Qt::AlignLeft);
    layout->addWidget(_dtrListView_ptr, 6, 0, 1, 2);

    QLabel* latitudeLb = new QLabel("위도");
    _nodeLat_ptr = new QLineEdit();
    _nodeLat_ptr->setValidator(new QDoubleValidator(_nodeLat_ptr));
    layout->addWidget(latitudeLb, 7, 0);
    layout->addWidget(_nodeLat_ptr, 7, 1);

    QLabel* longitudeLb = new QLabel("경도");
    _nodeLng_ptr = new QLineEdit();
    _nodeLng_ptr->setValidator(new QDoubleValidator(_nodeLng_ptr));
    layout->addWidget(longitudeLb, 8, 0);
    layout->addWidget(_nodeLng_ptr, 8, 1);

    m_task_info_ptr->setLayout(layout);
}

void NodeEditor::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    //    m_tab_ptr->resize(this->width(), this->height());
}

void NodeEditor::paintEvent(QPaintEvent*)
{
    QStyleOption opt;
    opt.init(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
}

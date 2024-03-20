//
// Created by antique on 24. 2. 26.
//

#include "ui/editor/route_editor.h"

#include <QGridLayout>
#include <QInputDialog>
#include <QStandardItemModel>
#include <QMessageBox>
#include <QMenu>
#include <QHeaderView>

#include "model/path_info_model.h"
#include "model/file_info_model.h"
#include "model/node_info_model.h"
#include "model/map_node_model.h"


RouteEditor::RouteEditor(QWidget *parent) :
        QWidget(parent),
        m_layout_ptr(new QGridLayout(this)),
        m_routeComboBox_ptr(new QComboBox()),
        _nodeListView_ptr(new QTableView()),
        _nodeListModel_ptr(new NodeListModel()),
        m_addRouteButton_ptr(new QPushButton("+")),
        m_addNodeButton_ptr(new QPushButton("+")) {

    this->setVisible(false);

    // Initialize UI Options
    m_layout_ptr->setVerticalSpacing(0);
    m_layout_ptr->setHorizontalSpacing(2);
    m_layout_ptr->setMargin(1);
    m_layout_ptr->addWidget(m_routeComboBox_ptr, 0, 0);
    m_layout_ptr->addWidget(m_addRouteButton_ptr, 0, 1);
    m_layout_ptr->addWidget(_nodeListView_ptr, 1, 0, 1, 2);
    m_layout_ptr->addWidget(m_addNodeButton_ptr, 2, 0, 1, 2);


    _nodeListModel_ptr->setHorizontalHeaderItem(0, new QStandardItem(QString("ID")));
    _nodeListModel_ptr->setHorizontalHeaderItem(1, new QStandardItem(QString("Type")));

    _nodeListView_ptr->setModel(_nodeListModel_ptr);
    _nodeListView_ptr->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    _nodeListView_ptr->setDragEnabled(true);
    _nodeListView_ptr->setAcceptDrops(true);
    _nodeListView_ptr->setDragDropOverwriteMode(false);
    _nodeListView_ptr->setDropIndicatorShown(true);
    _nodeListView_ptr->setSelectionMode(QAbstractItemView::SingleSelection);
    _nodeListView_ptr->setSelectionBehavior(QAbstractItemView::SelectRows);
    _nodeListView_ptr->setContextMenuPolicy(Qt::CustomContextMenu);

    m_routeComboBox_ptr->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_addRouteButton_ptr->setFixedSize(30, 30);
    m_addNodeButton_ptr->setFixedHeight(30);

    // Initialize UI Events
    connect(m_addRouteButton_ptr, &QPushButton::clicked, this, &RouteEditor::onAddRouteButtonClicked);
    connect(m_addNodeButton_ptr, &QPushButton::clicked, this, &RouteEditor::onAddNodeButtonClicked);
    connect(m_routeComboBox_ptr, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (index >= 0) {
            QVariant data = this->m_routeComboBox_ptr->itemData(index);
            QString pathId = data.toString();
            PathInfoModel::getInstance().selectCurrentPathId(pathId.toStdString());
        }
    });

    connect(_nodeListView_ptr, &QTableView::clicked, this, [this](const QModelIndex &index) {
        if (index.isValid()) {
            NodeInfoModel::getInstance().selectCurrentNode(index.row());
        }
    });

    connect(_nodeListView_ptr, &QTableView::customContextMenuRequested, [this](const QPoint &pos) {
        QModelIndex index = _nodeListView_ptr->indexAt(pos);
        if (index.isValid()) {
            QMenu contextMenu;
            QAction *action1 = contextMenu.addAction("삭제");
            QObject::connect(action1, &QAction::triggered, [index]() {
                NodeInfoModel::getInstance().removeNodeFromCurrentPath(index.row());
            });
            // 추가 액션 구현
            contextMenu.exec(QCursor::pos());
        }
    });

    // connect to models events
    connect(&FileInfoModel::getInstance(), &FileInfoModel::fileSavableChanged, this,
            &RouteEditor::onFileSavableChanged);
    connect(&PathInfoModel::getInstance(), &PathInfoModel::pathInfoMapChanged, this,
            &RouteEditor::onPathInfoMapChanged);
    connect(&NodeInfoModel::getInstance(), &NodeInfoModel::currentNodeListChanged, this,
            &RouteEditor::onCurrentNodeListChanged);
}

void RouteEditor::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
}

void RouteEditor::onAddRouteButtonClicked() {
    bool ok;
    QString pathId = QInputDialog::getText(this, tr("경로 ID 입력"), tr("경로 ID를 입력해주세요."),
                                           QLineEdit::Normal, "", &ok);


    if (!ok || pathId.isEmpty()) { return; }

    QString pathName = QInputDialog::getText(this, tr("경로 이름 입력"), tr("경로 이름을 입력해주세요."),
                                             QLineEdit::Normal, "", &ok);
    if (!ok || pathName.isEmpty()) { return; }

    PathInfoModel::getInstance().addPathInfo(pathId.toStdString(), pathName.toStdString());
}

void RouteEditor::onAddNodeButtonClicked() {
    GraphNode mapNode = MapNodeModel::getInstance().getSelectedMapNode();
    if (mapNode.nodeId.empty()) {
        QMessageBox::warning(nullptr, "선택된 노드 없음", "선택된 노드가 없습니다. 지도에서 노드를 선택 후 다시 시도해주세요.");
        return;
    }

    std::string pathId = PathInfoModel::getInstance().getCurrentPathId();
    if (pathId.empty()) {
        QMessageBox::warning(nullptr, "선택된 경로 없음", "선택된 경로가 없습니다. 경로 추가 및 선택 후 다시 시도해주세요.");
    }

    Node node;
    node.nodeId = mapNode.nodeId;
    node.position = mapNode.position;
    node.type = mapNode.type;

    NodeInfoModel::getInstance().addNodeToCurrentPath(node);
}

void RouteEditor::onFileSavableChanged(const bool savable) {
    this->setVisible(true);
}

void RouteEditor::onPathInfoMapChanged(const QMap<std::string, std::string> &pathInfoMap) {
    // QComboBox의 현재 항목을 std::map에 저장합니다.
    std::map<std::string, std::string> currentItems;
    for (int i = 0; i < m_routeComboBox_ptr->count(); ++i) {
        QVariant data = m_routeComboBox_ptr->itemData(i);
        QString text = m_routeComboBox_ptr->itemText(i);
        // key와 value를 추출하는 방식은 실제 데이터 구조에 따라 달라질 수 있습니다.
        currentItems[data.toString().toStdString()] = text.toStdString();
    }

    // pathInfoMap에 있는 항목만 유지하고, 없는 항목은 QComboBox에서 제거합니다.
    for (auto it = currentItems.begin(); it != currentItems.end();) {
        if (pathInfoMap.find(it->first) == pathInfoMap.end()) {
            // pathInfoMap에 없는 항목을 QComboBox에서 찾아 제거합니다.
            int index = m_routeComboBox_ptr->findData(QString::fromStdString(it->first));
            if (index != -1) m_routeComboBox_ptr->removeItem(index);
            it = currentItems.erase(it);
        } else {
            ++it;
        }
    }

    // pathInfoMap의 항목을 QComboBox에 추가합니다.
    // 이미 존재하는 항목은 추가하지 않습니다.
    for (auto &[key, value]: pathInfoMap.toStdMap()) {
        if (currentItems.find(key) == currentItems.end()) {
            // 새 항목을 추가합니다.
            std::string text = value + " [" + key + "]";
            m_routeComboBox_ptr->addItem(QString::fromStdString(text), QVariant(QString::fromStdString(key)));
        }
    }
}

void RouteEditor::onCurrentNodeListChanged(const QList<Node> &nodeList) {
    // Step 1: 현재 모델에서 제거해야 할 노드 식별
    QSet<QString> newIds;
    for (const Node& node : nodeList) {
        newIds.insert(QString::fromStdString(node.nodeId));
    }

    QVector<int> rowsToDelete;
    for (int i = 0; i < _nodeListModel_ptr->rowCount(); ++i) {
        QStandardItem* item = _nodeListModel_ptr->item(i, 0);
        if (item && !newIds.contains(item->text())) {
            rowsToDelete.append(i);
        }
    }
    // 역순으로 삭제하여 인덱스 변동을 방지
    std::sort(rowsToDelete.rbegin(), rowsToDelete.rend());
    for (int row : rowsToDelete) {
        _nodeListModel_ptr->removeRow(row);
    }

    // Step 2: 새 목록에 따라 노드 추가 및 순서 조정
    for (int i = 0; i < nodeList.size(); ++i) {
        const Node& node = nodeList[i];
        QString nodeId = QString::fromStdString(node.nodeId);

        // 현재 모델에서 해당 노드 찾기
        bool found = false;
        for (int j = 0; j < _nodeListModel_ptr->rowCount(); ++j) {
            QStandardItem* item = _nodeListModel_ptr->item(j, 0);
            if (item && item->text() == nodeId) {
                found = true;
                // 노드가 이미 존재하지만 잘못된 위치에 있는 경우 이동
                if (i != j) {
                    QList<QStandardItem*> row = _nodeListModel_ptr->takeRow(j);
                    _nodeListModel_ptr->insertRow(i, row);
                }
                break;
            }
        }

        // 노드가 모델에 없는 경우 추가
        if (!found) {
            QList<QStandardItem*> newRow;
            newRow.append(new QStandardItem(nodeId));
            newRow.append(new QStandardItem(QString::fromStdString(node.type)));
            _nodeListModel_ptr->insertRow(i, newRow);
        }
    }
}
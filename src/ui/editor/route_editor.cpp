//
// Created by antique on 24. 2. 26.
//

#include "ui/editor/route_editor.h"

#include <utility>
#include <QGridLayout>
#include <QInputDialog>
#include <QDir>
#include "utils/file/file_manager.h"
#include "viewmodel/path_view_model.h"
#include "viewmodel/route_file_view_model.h"

RouteEditor::RouteEditor(QWidget *parent) :
        QWidget(parent),
        m_layout_ptr(new QGridLayout(this)),
        m_routeComboBox_ptr(new QComboBox()),
        m_nodeTable_ptr(new QTableWidget(10, 2)),
        m_addRouteButton_ptr(new QPushButton("+")),
        m_addNodeButton_ptr(new QPushButton("+")),
        m_pathListener_ptr(std::make_shared<PathListener>(this)),
        m_savableStateListener_ptr(std::make_shared<SavableStateListener>(this)) {

    PathViewModel::Instance().attachToPathMap(m_pathListener_ptr);
    RouteFileViewModel::Instance().attachToSavable(m_savableStateListener_ptr);
    this->setVisible(false);

    // Initialize UI Options
    m_layout_ptr->setVerticalSpacing(0);
    m_layout_ptr->setHorizontalSpacing(2);
    m_layout_ptr->setMargin(1);
    m_layout_ptr->addWidget(m_routeComboBox_ptr, 0, 0);
    m_layout_ptr->addWidget(m_addRouteButton_ptr, 0, 1);
    m_layout_ptr->addWidget(m_nodeTable_ptr, 1, 0, 1, 2);
    m_layout_ptr->addWidget(m_addNodeButton_ptr, 2, 0, 1, 2);

    m_nodeTable_ptr->setHorizontalHeaderLabels({"ID", "Type"});
    m_nodeTable_ptr->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_nodeTable_ptr->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_nodeTable_ptr->resize(300, 400);

    m_routeComboBox_ptr->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_addRouteButton_ptr->setFixedSize(30, 30);
    m_addNodeButton_ptr->setFixedHeight(30);

    // Initialize UI Events
    connect(m_addRouteButton_ptr, &QPushButton::clicked, this, &RouteEditor::onAddRouteButtonClicked);
    connect(m_routeComboBox_ptr, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (index >= 0) {
            QVariant data = this->m_routeComboBox_ptr->itemData(index);
            QString pathId = data.toString();
            std::cout << "Selected path name:" << pathId.toStdString() << "\n";
        }
    });
}

void RouteEditor::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
}

void RouteEditor::onAddRouteButtonClicked() {
    Path path;

    bool ok;
    QString pathId = QInputDialog::getText(this, tr("경로 ID 입력"), tr("경로 ID를 입력해주세요."),
                                           QLineEdit::Normal, "", &ok);


    if (!ok || pathId.isEmpty()) { return; }

    path.id = pathId.toStdString();

    QString pathName = QInputDialog::getText(this, tr("경로 이름 입력"), tr("경로 이름을 입력해주세요."),
                                             QLineEdit::Normal, "", &ok);
    if (!ok || pathName.isEmpty()) { return; }

    path.name = pathName.toStdString();

    PathViewModel::Instance().addPath(path);
}

void RouteEditor::onAddNodeButtonClicked() {
    Node node;
    node.type = "타입을 지정해주세요.";
}

RouteEditor::PathListener::PathListener(RouteEditor *editor) : m_editor_ptr(editor) {

}

void RouteEditor::PathListener::update(std::map<std::string, Path> data) {
    m_editor_ptr->m_routeComboBox_ptr->clear();

    for (std::pair<const std::string, Path> item : data) {
        const std::string &id = item.second.id;
        const std::string &name = item.second.name;

        std::string text = std::string(name +  " [" + id + "]");
        m_editor_ptr->m_routeComboBox_ptr->addItem(QString::fromStdString(text),
                                                   QVariant(QString::fromStdString(id)));
    }
}

RouteEditor::SavableStateListener::SavableStateListener(RouteEditor *editor) : m_editor_ptr(editor) {

}

void RouteEditor::SavableStateListener::update(bool data) {
    m_editor_ptr->setVisible(data);
}

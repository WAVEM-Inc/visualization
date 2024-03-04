//
// Created by antique on 24. 2. 19.
//

#include <QMessageBox>
#include <QWebChannel>
#include <QWebEngineSettings>
#include <QEventLoop>
#include <utility>

#include "map_view.h"
#include "utils/coordinate_handler.h"
#include "utils/file/file_manager.h"

MapView::MapView(QWidget *parent) :
        QWidget(parent),
        m_webview_ptr(new QWebEngineView(this)),
        m_webpage_ptr(new QWebEnginePage()),
        m_web_channel_ptr(new QWebChannel(m_webpage_ptr)) {

    m_webpage_ptr->setWebChannel(m_web_channel_ptr);

    // Register handler to channel
    CoordinateHandler *handler = new CoordinateHandler();
    m_web_channel_ptr->registerObject(QStringLiteral("coordinateHandler"), handler);

    // Register page to view
    m_webview_ptr->setPage(m_webpage_ptr);

    // Set options to view
    QWebEngineSettings *settings = m_webpage_ptr->settings();
    settings->setAttribute(QWebEngineSettings::LocalContentCanAccessRemoteUrls, true);
    settings->setAttribute(QWebEngineSettings::LocalContentCanAccessFileUrls, true);

    // Load html file
    std::string filePath = std::string(RESOURCE_DIR) + "/map/map.html";
    QUrl url = QUrl::fromLocalFile(filePath.c_str());

    m_webview_ptr->load(url);

    connect(&MapNodeModel::getInstance(), &MapNodeModel::mapNodesChanged, this, &MapView::onMapNodesChanged);

//    MapNodeViewModel::Instance().mapNodes()->attach(m_mapNodesListener_ptr);
}

void MapView::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    m_webview_ptr->resize(this->size());
}

void MapView::onMapNodesChanged(const QMap<std::string, MapNode> &nodeMap) {
    nlohmann::json json = nodeMap.toStdMap();
    QString qString = QString(json.dump().data());
    m_webpage_ptr->runJavaScript(QString(
            "mapNodeJsonData = '" + qString + "';"
            "updateMarkers(mapNodeJsonData);"
    ));
}
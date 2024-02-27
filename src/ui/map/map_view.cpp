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
        m_web_channel_ptr(new QWebChannel(m_webpage_ptr)),
        m_mapNodesListener_ptr(std::make_shared<MapNodeVectorListener>(m_webpage_ptr)) {

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

    MapNodeViewModel::Instance().mapNodes()->attach(m_mapNodesListener_ptr);
}

void MapView::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    m_webview_ptr->resize(this->size());
}

MapNodeVectorListener::MapNodeVectorListener(QPointer<QWebEnginePage> webPage) : m_webpage_ptr(std::move(webPage)) {
    std::cout << m_webpage_ptr->url().toString().toStdString() << "\n";
    std::cout << "Wait for connect to page..." << "\n";
    connect(m_webpage_ptr, &QWebEnginePage::loadFinished, [this](bool success) {
        if (success) {
            std::cout << "Success to connect page!" << "\n";
            pageLoaded = true;
        } else {
            std::cout << "Faied to connect page..." << "\n";
        }
    });
}

void MapNodeVectorListener::update(std::map<std::string, Position> data) {
    if (pageLoaded) {
        nlohmann::json json = data;
        QString qString = QString(json.dump().data());
        m_webpage_ptr->runJavaScript(QString(
                "mapNodeJsonData = '" + qString + "';"
                "updateMarkers(mapNodeJsonData);"
        ));
    } else {
        std::cout << "Page is not loaded...!" << "\n";
    }

}
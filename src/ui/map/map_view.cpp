//
// Created by antique on 24. 2. 19.
//

#include <QMessageBox>
#include <QWebChannel>
#include <QWebEngineSettings>

#include "map_view.h"
#include "utils/coordinate_handler.h"

MapView::MapView(QWidget *parent) :
        QWidget(parent),
        m_webview_ptr(new QWebEngineView(this)),
        m_webpage_ptr(new QWebEnginePage()) {

    // Register channel to page
    QWebChannel *channel = new QWebChannel(m_webpage_ptr);
    m_webpage_ptr->setWebChannel(channel);

    // Register handler to channel
    CoordinateHandler *handler = new CoordinateHandler();
    channel->registerObject(QStringLiteral("coordinateHandler"), handler);

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
}

void MapView::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    m_webview_ptr->resize(this->size());
}

//
// Created by antique on 24. 2. 19.
//

#include <QMessageBox>
#include <QWebChannel>
#include <QWebEngineSettings>

#include "map_view.h"
#include "utils/coordinate_handler.h"
#include "utils/file/config_file_reader.h"
#include "model/code_info_model.h"
#include "utils/file/code_file_reader.h"
#include "model/node_info_model.h"

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
    connect(&MapNodeModel::getInstance(), &MapNodeModel::selectedMapNodeChanged, this, [this](const GraphNode &node) {
        std::cout << MapNodeModel::getInstance().getChangeCenterToSelectedNode() << "\n";
        m_webpage_ptr->runJavaScript(QString(
                "updateSelectedNode(\"%1\", %2);"
        ).arg(node.nodeId.c_str()).arg(MapNodeModel::getInstance().getChangeCenterToSelectedNode()));
    });
    connect(&NodeInfoModel::getInstance(), &NodeInfoModel::currentNodeChanged, this, [this](const Node &node) {
        std::cout << node.nodeId << "\n";
        m_webpage_ptr->runJavaScript(QString(
                "updateSelectedNode(\"%1\");"
        ).arg(node.nodeId.c_str()));
    });

    // Initialize config data
    ConfigFileReader cfgReader;
    ConfigFile cfgData = cfgReader.loadFile();

    CodeFileReader codeReader;
    std::vector<CodeGroup> codeData = codeReader.loadFile();

    connect(m_webpage_ptr, &QWebEnginePage::loadFinished, this, [this, cfgData, codeData]() {
        m_webpage_ptr->runJavaScript(QString(
                "changeCenter(%1, %2);\n"
                "changeZoomLevel(%3);"
        ).arg(cfgData.center.latitude).arg(cfgData.center.longitude).arg(cfgData.zoomLevel));

        FileInfoModel::getInstance().updateLatestFilePath(cfgData.latestFilePath);
        CodeInfoModel::getInstance().setCodeMap(codeData);
    });
}

void MapView::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    m_webview_ptr->resize(this->size());
}

void MapView::onMapNodesChanged(const QMap<std::string, GraphNode> &nodeMap) {
    nlohmann::json json = nodeMap.toStdMap();
    QString qString = QString(json.dump().data());
    m_webpage_ptr->runJavaScript(QString(
            "mapNodeJsonData = '" + qString + "';\n" +
            "updateMarkers(mapNodeJsonData);"
    ));

    std::cout << json.dump(4) << "\n";
}
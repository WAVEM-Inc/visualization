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
#include "utils/GeoPositionUtil.h"
#include "GeographicLib/UTMUPS.hpp"
#include "model/ros_2_data_model.h"

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

    connect(&MapNodeModel::getInstance(), &MapNodeModel::showingNodesChanged, this,
            [this](const std::vector<GraphNode> &nodes) {
                nlohmann::json json = nodes;
                QString qString = QString(json.dump().data());
                m_webpage_ptr->runJavaScript(QString(
                        "mapNodeJsonData = '" + qString + "';\n" +
                        "updateMarkers(mapNodeJsonData, %1);"
                ).arg(MapNodeModel::getInstance().getShowAllNodes()));
            });

    connect(&MapNodeModel::getInstance(), &MapNodeModel::selectedMapNodeChanged, this, [this](const GraphNode &node) {
        std::cout << MapNodeModel::getInstance().getChangeCenterToSelectedNode() << "\n";
        m_webpage_ptr->runJavaScript(QString(
                "updateSelectedNode(\"%1\", %2);"
        ).arg(node.nodeId.c_str()).arg(MapNodeModel::getInstance().getChangeCenterToSelectedNode()));
    });

    connect(&NodeInfoModel::getInstance(), &NodeInfoModel::currentNodeChanged, this, [this](const Node &node) {
        std::cout << node.nodeId << "\n";
        m_webpage_ptr->runJavaScript(QString(
                "updateSelectedNode(\"%1\", %2);"
        ).arg(node.nodeId.c_str()).arg(MapNodeModel::getInstance().getChangeCenterToSelectedNode()));

        // if (MapNodeModel::getInstance().getChangeCenterToSelectedNode()) {
        //     m_webpage_ptr->runJavaScript(QString(
        //             "changeCenter(%1, %2);"
        //     ).arg(node.position.latitude).arg(node.position.longitude));
        // }
    });

    connect(&ROS2DataModel::getInstance(), &ROS2DataModel::onNavSatFixChanged, this,
            [this](const sensor_msgs::msg::NavSatFix &navSatFix) {

                double lat = navSatFix.latitude;
                double lng = navSatFix.longitude;

                std::cout << navSatFix.latitude << "\n";

                m_webpage_ptr->runJavaScript(QString(
                        "updateVehicleLocation(%1, %2);").arg(lat, 0, 'f', 6).arg(lng, 0, 'f', 6));
            });

    connect(&NodeInfoModel::getInstance(), &NodeInfoModel::currentNodeListChanged, this, &MapView::showDetectionRanges);

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

    connect(&MapNodeModel::getInstance(), &MapNodeModel::refreshEventOccured, this, [this]() {

        nlohmann::json json = MapNodeModel::getInstance().getShowingNodes();
        QString qString = QString(json.dump().data());
        m_webpage_ptr->runJavaScript(QString(
                "mapNodeJsonData = '" + qString + "';\n" +
                "updateMarkers(mapNodeJsonData, %1);"
        ).arg(MapNodeModel::getInstance().getShowAllNodes()));

        showDetectionRanges(NodeInfoModel::getInstance().getNodesFromCurrentPath());

        std::cout << "Refresh Event Occured!" << "\n";
    });
}

void MapView::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    m_webview_ptr->resize(this->size());
}

void MapView::showDetectionRanges(const QList<Node> &nodeList) {
    std::vector<std::array<Position, 4>> rectangles;

    for (int i = 0; i < nodeList.size(); i++) {
        const Node &node = nodeList[i];

        for (const DetectionRange &range: node.detectionRange) {
            std::array<Position, 4> rectanglesVertex;

            // Convert Nodes Lat/Lng to UTM Coordinates
            int zone;
            bool northp;
            double nodeX, nodeY;
            GeographicLib::UTMUPS::Forward(node.position.latitude, node.position.longitude, zone, northp, nodeX, nodeY);

            // Calculate DetectionRanges rectangle vertex coordinates.
            double x1 = nodeX - range.widthLeft, y1 = nodeY + range.offset;
            double x2 = nodeX + range.widthRight, y2 = y1;
            double x3 = x2, y3 = y1 + range.height;
            double x4 = x1, y4 = y3;


            // Calculate real rectangles vertex coordinates from node heading.
            double radian = toRadians(node.heading);
            double x1p = cos(radian) * (x1 - nodeX) - sin(radian) * (y1 - nodeY) + nodeX;
            double y1p = sin(radian) * (x1 - nodeX) + cos(radian) * (y1 - nodeY) + nodeY;
            double x2p = cos(radian) * (x2 - nodeX) - sin(radian) * (y2 - nodeY) + nodeX;
            double y2p = sin(radian) * (x2 - nodeX) + cos(radian) * (y2 - nodeY) + nodeY;
            double x3p = cos(radian) * (x3 - nodeX) - sin(radian) * (y3 - nodeY) + nodeX;
            double y3p = sin(radian) * (x3 - nodeX) + cos(radian) * (y3 - nodeY) + nodeY;
            double x4p = cos(radian) * (x4 - nodeX) - sin(radian) * (y4 - nodeY) + nodeX;
            double y4p = sin(radian) * (x4 - nodeX) + cos(radian) * (y4 - nodeY) + nodeY;

            // Reconvert UTM Coordinates to WGS84 Lat/Lng.
            double lat1, lng1, lat2, lng2, lat3, lng3, lat4, lng4;
            GeographicLib::UTMUPS::Reverse(zone, northp, x1p, y1p, lat1, lng1);
            GeographicLib::UTMUPS::Reverse(zone, northp, x2p, y2p, lat2, lng2);
            GeographicLib::UTMUPS::Reverse(zone, northp, x3p, y3p, lat3, lng3);
            GeographicLib::UTMUPS::Reverse(zone, northp, x4p, y4p, lat4, lng4);

            rectanglesVertex[0] = Position(lat1, lng1);
            rectanglesVertex[1] = Position(lat2, lng2);
            rectanglesVertex[2] = Position(lat3, lng3);
            rectanglesVertex[3] = Position(lat4, lng4);

            rectangles.push_back(rectanglesVertex);
        }
    }

    nlohmann::json json = rectangles;
    QString qString = QString(json.dump().data());

    m_webpage_ptr->runJavaScript(QString(
            "detectionRangesJsonData = '" + qString + "';\n" +
            "updateDetectionRanges(detectionRangesJsonData);"
    ));
}
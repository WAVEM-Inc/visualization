//
// Created by antique on 24. 2. 19.
//

#include <QInputDialog>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include "ui/menu/menu_bar.h"
#include "utils/events/FileEvents.h"
#include "utils/file/route_file_reader.h"
#include "utils/file/route_file_writer.h"
#include "model/map_node_model.h"
#include "ui/dialog/node_type_dialog.h"
#include "model/file_info_model.h"
#include "model/path_info_model.h"
#include "model/node_info_model.h"

MenuBar::MenuBar(QWidget *parent) : QMenuBar(parent) {
    initializeStyleSheet();

    // Menu-File
    QMenu *fileMenu = this->addMenu(tr("&파일"));

    // Menu-File-Edit
    fileMenu->addSection(tr("Edit"));

    QAction *newFile = fileMenu->addAction(tr("&새 파일"));
    newFile->setShortcut(QKeySequence("Ctrl+N"));
    connect(newFile, &QAction::triggered, this, [this]() {
        this->onNewFile();
    });

    QAction *openFile = fileMenu->addAction(tr("&불러오기"));
    openFile->setShortcut(QKeySequence("Ctrl+O"));
    connect(openFile, &QAction::triggered, this, [this]() {
        this->onOpenFile();
    });

    //Menu-File-Save
    fileMenu->addSection(tr("Save"));

    QAction *saveFile = fileMenu->addAction(tr("&저장"));
    saveFile->setShortcut(QKeySequence("Ctrl+S"));
    saveFile->setEnabled(false);
    connect(saveFile, &QAction::triggered, this, [this]() {
        this->onSaveFile();
    });

    QAction *saveFileAs = fileMenu->addAction(tr("&다른 이름으로 저장"));
    saveFileAs->setShortcut(QKeySequence("Ctrl+Shift+S"));
    saveFileAs->setEnabled(false);
    connect(saveFileAs, &QAction::triggered, this, [this]() {
       this->onSaveFileAs();
    });

    // Menu-Edit
    QMenu *editMenu = this->addMenu(tr("&보기"));

    QAction *showAllNode = editMenu->addAction(tr("&전체 노드 보기"));
    showAllNode->setShortcut(QKeySequence("Ctrl+A"));
    showAllNode->setCheckable(true);
    showAllNode->setChecked(true);
    MapNodeModel::getInstance().setShowAllNodes(true);
    connect(showAllNode, &QAction::triggered, this, [showAllNode]() {
        MapNodeModel::getInstance().setShowAllNodes(showAllNode->isChecked());
    });

    QAction *fixMapCenter = editMenu->addAction(tr("%선택된 노드로 시점 이동"));
    fixMapCenter->setCheckable(true);
    fixMapCenter->setChecked(true);
    MapNodeModel::getInstance().setChangeCenterToSelectedNode(true);
    connect(fixMapCenter, &QAction::triggered, this, [fixMapCenter]() {
        bool useAutoCenter = fixMapCenter->isChecked();
        MapNodeModel::getInstance().setChangeCenterToSelectedNode(useAutoCenter);
    });

    // connect to models
    connect(&FileInfoModel::getInstance(), &FileInfoModel::fileSavableChanged, this,
            [saveFile, saveFileAs](bool savable) {

        saveFileAs->setEnabled(savable);
        saveFile->setEnabled(savable);
    });
}

void MenuBar::initializeStyleSheet() {
    this->setStyleSheet(
            "QMenuBar { "
            "   background-color: #222222;"
            "   padding: 0px;"
            "   margin: 0px;"
            "}"
            ""
            "QMenuBar::item { "
            "   color: white;"
            "   font-size: 18px;"
            "   background-color: transparent;"
            "   padding: 12px;"
            "   margin: 0px;"
            "}"
            ""
            "QMenuBar::item:selected { "
            "   background-color: #555555;"
            "}"
            ""
            "QMenu { "
            "   background-color: #333333;"
            "   color: white;"
            "   font-size: 12px;"
            "}"
            ""
            "QMenu::item:selected {"
            "   background-color: #555555;"
            "}"
            ""
    );
}

void MenuBar::onNewFile() {
    bool ok;
    QString fileVersion = QInputDialog::getText(nullptr, tr("파일 버전명 입력"), tr("파일 버전"),
                                                QLineEdit::Normal, "", &ok);

    QString mapId;
    if (ok && !fileVersion.isEmpty()) {
        mapId = QInputDialog::getText(nullptr, tr("맵 아이디 입력"), tr("맵 아이디"),
                                      QLineEdit::Normal, "", &ok);
    }

    QString latestFilePath = QString(FileInfoModel::getInstance().getLatestFilePath().c_str());
    QString filePath;
    if (latestFilePath.isEmpty()) {
        filePath = QFileDialog::getSaveFileName(this, tr("Save File Path"), QDir::home().absolutePath(),
                                                        tr("Data files (*.dat)"));
    } else {
        filePath = QFileDialog::getSaveFileName(this, tr("Save File Path"), latestFilePath,
                                                        tr("Data files (*.dat)"));
    }

    if (!filePath.isEmpty()) {
        RouteFile routeFileData;
        routeFileData.version = fileVersion.toStdString();
        routeFileData.mapId = mapId.toStdString();

        RouteFileWriter writer;
        writer.saveFile(filePath, routeFileData);
    }
}

void MenuBar::onOpenFile() {
    bool ok;
    QString latestFilePath = QString(FileInfoModel::getInstance().getLatestFilePath().c_str());
    QString defaultFilePath;
    if (latestFilePath.isEmpty()) {
        defaultFilePath = QDir::home().absolutePath();
    } else {
        defaultFilePath = latestFilePath;
    }
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open File Path"), defaultFilePath,
                                                    tr("Data files (*.dat)"));

    if (!filePath.isEmpty()) {
        RouteFileReader reader;
        reader.loadFile(filePath);
    }
}

void MenuBar::onSaveFile() {
    FileInfo info = FileInfoModel::getInstance().getFileInfo();
    RouteFileWriter writer;

    RouteFile fileData;
    fileData.version = info.fileVersion;
    fileData.mapId = info.mapId;

    std::vector<Path> paths;
    for (const auto &entry : NodeInfoModel::getInstance().getAllNodes().toStdMap()) {
        Path path;
        path.id = entry.first;
        path.name = PathInfoModel::getInstance().getPathInfoMap()[entry.first];
        path.nodeList = entry.second.toVector().toStdVector();
        paths.push_back(path);
    }
    fileData.path = paths;

    std::vector<GraphNode> mapNodes;
    for (const GraphNode &mapNode : MapNodeModel::getInstance().getMapNodes()) {
        mapNodes.push_back(mapNode);
    }
    fileData.node = mapNodes;

    writer.saveFile(QString(info.filePath.c_str()), fileData);
}

void MenuBar::onSaveFileAs() {
    bool ok;

    QString latestFilePath = QString(FileInfoModel::getInstance().getLatestFilePath().c_str());
    QString filePath;
    if (latestFilePath.isEmpty()) {
        filePath = QFileDialog::getSaveFileName(this, tr("Save File Path"), QDir::home().absolutePath(),
                                                tr("Data files (*.dat)"));
    } else {
        filePath = QFileDialog::getSaveFileName(this, tr("Save File Path"), latestFilePath,
                                                tr("Data files (*.dat)"));
    }

    if (!filePath.isEmpty()) {
        FileInfo info = FileInfoModel::getInstance().getFileInfo();
        RouteFileWriter writer;

        RouteFile fileData;
        fileData.version = info.fileVersion;
        fileData.mapId = info.mapId;

        std::vector<Path> paths;
        for (const auto &entry : NodeInfoModel::getInstance().getAllNodes().toStdMap()) {
            Path path;
            path.id = entry.first;
            path.name = PathInfoModel::getInstance().getPathInfoMap()[entry.first];
            path.nodeList = entry.second.toVector().toStdVector();
            paths.push_back(path);
        }
        fileData.path = paths;

        std::vector<GraphNode> mapNodes;
        for (const GraphNode &mapNode : MapNodeModel::getInstance().getMapNodes()) {
            mapNodes.push_back(mapNode);
        }
        fileData.node = mapNodes;

        writer.saveFile(filePath, fileData);
    }
}

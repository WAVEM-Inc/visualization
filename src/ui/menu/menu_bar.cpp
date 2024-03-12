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

    // Menu-Edit
    QMenu *editMenu = this->addMenu(tr("&편집"));

    QAction *addNewNode = editMenu->addAction(tr("&지도에 노드 추가"));
    addNewNode->setShortcut(QKeySequence("Ctrl+Alt+N"));
    addNewNode->setEnabled(false);
    connect(addNewNode, &QAction::triggered, this, [this]() {
       this->onAddNewNode();
    });

    // connect to models
    connect(&FileInfoModel::getInstance(), &FileInfoModel::fileSavableChanged, this,
            [saveFile, saveFileAs, addNewNode](bool savable) {

        saveFileAs->setEnabled(savable);
        saveFile->setEnabled(savable);
        addNewNode->setEnabled(savable);
    });

/*    // Listen savable state
    m_listener.listen([&saveFile, &saveFileAs](const event::FILE_SAVABLE &event) {
        std::cout << "Savable state: " << event.isSavable << "\n";
        saveFile->setEnabled(event.isSavable);
        saveFileAs->setEnabled(event.isSavable);
    });*/
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

/*    QString filePath = QFileDialog::getSaveFileName(this, tr("Save File Path"), QDir::home().absolutePath(),
                                                    tr("Data files (*.dat)"));*/

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

void MenuBar::onAddNewNode() {
    MapNodeModel::getInstance().updateUseAddMode(true);
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
        path.nodelist = entry.second.toVector().toStdVector();
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

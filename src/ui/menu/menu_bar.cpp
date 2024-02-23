//
// Created by antique on 24. 2. 19.
//

#include <QInputDialog>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include "ui/menu/menu_bar.h"
#include "nlohmann/json.hpp"
#include "utils/file_manager.h"
#include "utils/events/FileEvents.h"

MenuBar::MenuBar(QWidget *parent) : QMenuBar(parent) {
    initializeStyleSheet();

    // Menu-File
    QMenu *fileMenu = this->addMenu(tr("&File"));

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

    QAction *saveFileAs = fileMenu->addAction(tr("&다른 이름으로 저장"));
    saveFileAs->setShortcut(QKeySequence("Ctrl+Shift+S"));
    saveFileAs->setEnabled(false);

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

    QString filePath = QFileDialog::getSaveFileName(this, tr("Save File Path"), QDir::home().absolutePath(),
                                                    tr("Data files (*.dat)"));

    if (!filePath.isEmpty()) {
        RouteFile routeFileData;
        routeFileData.fileVersion = fileVersion.toStdString();
        routeFileData.mapId = mapId.toStdString();

        saveFile(filePath, routeFileData);
    }
}

void MenuBar::onOpenFile() {
    bool ok;
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open File Path"), QDir::home().absolutePath(),
                                                    tr("Data files (*.dat)"));

    if (!filePath.isEmpty()) {
        loadFile(filePath);
    }
}

bool MenuBar::saveFile(const QString &filePath, const RouteFile &routeFileData) {
    QString finalPath = filePath;
    QFileInfo fileInfo(finalPath);

    // 확장자 확인 및 추가
    if (fileInfo.suffix().compare("dat", Qt::CaseInsensitive) != 0) {
        finalPath += ".dat";
        fileInfo.setFile(finalPath); // 확장자가 추가된 최종 경로로 QFileInfo 업데이트
    }


    // 파일이 이미 존재하는 경우 덮어쓰기 여부 확인
    if (fileInfo.exists()) {
        auto response = QMessageBox::question(
                nullptr,
                tr("덮어쓰기"),
                tr("이미 존재하는 파일입니다. 덮어 쓰시겠습니까?"),
                QMessageBox::Yes | QMessageBox::No
        );

        if (response == QMessageBox::No) {
            // 사용자가 'No'를 선택한 경우, 저장 작업을 취소합니다.
            return false;
        }
    }

    // 파일 저장
    QFile file(finalPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Failed to save file: %1").arg(finalPath));
        return false;
    }

    nlohmann::json dataJson = routeFileData;
    QString textData = QString(dataJson.dump(4).c_str());

    QTextStream out(&file);
    out << textData;
    file.close();

    if (out.status() == QTextStream::Ok) {
        FileManager::Instance().updateOriginFileData(routeFileData);
        FileManager::Instance().updateCacheFileData(routeFileData);
    } else {
        QMessageBox::warning(nullptr, "실패", "파일 저장에 실패하였습니다.");

        return false;
    }

    std::cout << "Saved path: " << finalPath.toStdString() << "\n";

    return true;
}

bool MenuBar::loadFile(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(nullptr, "실패", "파일을 불러오지 못했습니다. 다시 시도해 주세요");

        return false;
    }

    QTextStream in(&file);
    QString fileContent = in.readAll();

    nlohmann::json json = nlohmann::json::parse(fileContent.toStdString());
    RouteFile fileData = json.get<RouteFile>();

    FileManager::Instance().updateOriginFileData(fileData);
    FileManager::Instance().updateCacheFileData(fileData);

    nlohmann::json originData = FileManager::Instance().getOriginFileData();
    nlohmann::json cacheData = FileManager::Instance().getCacheFileData();

    std::cout << "Origin Data: \n" << originData.dump(4) << "\n";
    std::cout << "Cache Data: \n" << cacheData.dump(4) << "\n";

    return true;
}
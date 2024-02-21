//
// Created by antique on 24. 2. 19.
//

#include "ui/menu/menu_bar.h"

MenuBar::MenuBar(QWidget *parent) : QMenuBar(parent) {
    initializeStyleSheet();

    // Menu-File
    QMenu *fileMenu = this->addMenu(tr("&File"));

    // Menu-File-Edit
    fileMenu->addSection(tr("Edit"));

    QAction *newFile = fileMenu->addAction(tr("&새 파일"));
    newFile->setShortcut(QKeySequence("Ctrl+N"));

    QAction *openFile = fileMenu->addAction(tr("&불러오기"));
    openFile->setShortcut(QKeySequence("Ctrl+O"));

    //Menu-File-Save
    fileMenu->addSection(tr("Save"));

    QAction *saveFile = fileMenu->addAction(tr("&저장"));
    saveFile->setShortcut(QKeySequence("Ctrl+S"));

    QAction *saveFileAs = fileMenu->addAction(tr("&다른 이름으로 저장"));
    saveFileAs->setShortcut(QKeySequence("Ctrl+Shift+S"));
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
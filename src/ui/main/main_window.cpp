//
// Created by antique on 24. 2. 19.
//

#include <iostream>
#include <QVBoxLayout>
#include "ui/main/main_window.h"

MainWindow::MainWindow(QWidget *parent) :
QMainWindow(parent),
m_mapView_ptr(new MapView(this)),
m_menubar_ptr(new MenuBar(this)),
m_routeEditor_ptr(new RouteEditor(this)) {

    this->setMenuBar(m_menubar_ptr);
    this->setCentralWidget(m_mapView_ptr); // m_mapView_ptr를 centralWidget으로 설정
//    m_routeEditor_ptr->resize(300, 400);
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);

    m_routeEditor_ptr->adjustSize();
    m_routeEditor_ptr->move(0, this->height() - m_routeEditor_ptr->height());
}

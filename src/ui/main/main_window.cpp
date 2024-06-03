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
m_routeEditor_ptr(new RouteEditor(this)),
m_nodeEditor_ptr(new NodeEditor(this)) {

    this->setMenuBar(m_menubar_ptr);
    this->setCentralWidget(m_mapView_ptr); // m_mapView_ptr를 centralWidget으로 설정
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);

    m_routeEditor_ptr->resize(310, this->height() * 0.7);
    m_routeEditor_ptr->move(0, this->height() - m_routeEditor_ptr->height());

    m_nodeEditor_ptr->resize(500, this->height() - m_menubar_ptr->height());
    m_nodeEditor_ptr->move(this->width() - 500, m_menubar_ptr->height());
}

MainWindow::NodeListener::NodeListener(MainWindow *window) : m_window_ptr(window) {}

void MainWindow::NodeListener::update(Node data) {
    if (data.type != "null") {
        m_window_ptr->m_nodeEditor_ptr->setVisible(true);
    } else {
        m_window_ptr->m_nodeEditor_ptr->setVisible(false);
    }
}

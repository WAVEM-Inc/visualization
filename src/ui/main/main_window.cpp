//
// Created by antique on 24. 2. 19.
//

#include <iostream>
#include <QVBoxLayout>
#include "ui/main/main_window.h"

MainWindow::MainWindow(QWidget *parent) :
QMainWindow(parent),
m_mapView_ptr(new MapView(this)),
m_menubar_ptr(new MenuBar(this)) {

    this->setMenuBar(m_menubar_ptr);
    this->setCentralWidget(m_mapView_ptr);
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    m_mapView_ptr->resize(this->size());
}

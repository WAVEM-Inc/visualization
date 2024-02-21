//
// Created by antique on 24. 2. 19.
//

#ifndef NODE_EDITOR_MAIN_WINDOW_H
#define NODE_EDITOR_MAIN_WINDOW_H


#include <QWidget>
#include <QMenuBar>
#include <QMainWindow>
#include "map_view.h"
#include "ui/menu/menu_bar.h"

class MainWindow : public QMainWindow {
Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    MapView *m_mapView_ptr;
    MenuBar *m_menubar_ptr;
};


#endif //NODE_EDITOR_MAIN_WINDOW_H

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
#include "ui/editor/route_editor.h"
#include "ui/editor/node_editor.h"

class MainWindow : public QMainWindow {
Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    class NodeListener : public Observer<Node>{
    public:
        NodeListener(MainWindow *window);
        void update(Node data) override;

    private:
        MainWindow *m_window_ptr;
    };

    void resizeEvent(QResizeEvent *event) override;

private:
    MapView *m_mapView_ptr;
    MenuBar *m_menubar_ptr;
    RouteEditor *m_routeEditor_ptr;
    NodeEditor *m_nodeEditor_ptr;
};


#endif //NODE_EDITOR_MAIN_WINDOW_H

//
// Created by antique on 24. 2. 19.
//

#ifndef NODE_EDITOR_MENU_BAR_H
#define NODE_EDITOR_MENU_BAR_H


#include <QMenuBar>
#include "struct/RouteFile.h"
#include "utils/patterns/observer/subject.h"

class MenuBar : public QMenuBar {
Q_OBJECT
public:
    explicit MenuBar(QWidget *parent = nullptr);

protected:
    void initializeStyleSheet();

private:
    void onNewFile();

    void onOpenFile();

    void onSaveFile();

    void onSaveFileAs();

};


#endif //NODE_EDITOR_MENU_BAR_H

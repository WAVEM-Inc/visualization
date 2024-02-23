//
// Created by antique on 24. 2. 19.
//

#ifndef NODE_EDITOR_MENU_BAR_H
#define NODE_EDITOR_MENU_BAR_H


#include <QMenuBar>

class MenuBar : public QMenuBar {
Q_OBJECT
public:
    explicit MenuBar(QWidget *parent = nullptr);

protected:
    void initializeStyleSheet();

    static bool saveFile(const QString &filePath, const QString &textData);

private:
    void onNewFile();

    void onOpenFile();

    void onSaveFile();

    void onSaveFileAs();

};


#endif //NODE_EDITOR_MENU_BAR_H

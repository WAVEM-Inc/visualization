//
// Created by antique on 24. 2. 19.
//

#ifndef NODE_EDITOR_MAP_VIEW_H
#define NODE_EDITOR_MAP_VIEW_H


#include <QWidget>
#include <QWebEngineView>

class MapView : public QWidget {
    Q_OBJECT

public:
    explicit MapView(QWidget *parent = nullptr);

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    QWebEngineView *m_webview_ptr;
    QWebEnginePage *m_webpage_ptr;
};


#endif //NODE_EDITOR_MAP_VIEW_H

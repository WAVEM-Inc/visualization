//
// Created by antique on 24. 2. 19.
//

#ifndef NODE_EDITOR_MAP_VIEW_H
#define NODE_EDITOR_MAP_VIEW_H


#include <QWidget>
#include <QWebEngineView>
#include <QPointer>
#include "utils/patterns/observer/observer.h"
#include "struct/MapNode.h"
#include "struct/RouteFile.h"

class MapNodeVectorListener : public QObject, public Observer<std::map<std::string, Position>>{
    Q_OBJECT
public:
    explicit MapNodeVectorListener(QPointer<QWebEnginePage> webView);

    void update(std::map<std::string, Position> data) override;

private:
    QPointer<QWebEnginePage> m_webpage_ptr;
    bool pageLoaded = false;
};

class MapView : public QWidget {
Q_OBJECT

public:
    explicit MapView(QWidget *parent = nullptr);

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    QPointer<QWebEngineView> m_webview_ptr;
    QPointer<QWebEnginePage> m_webpage_ptr;
    QPointer<QWebChannel> m_web_channel_ptr;
    std::shared_ptr<MapNodeVectorListener> m_mapNodesListener_ptr;
};


#endif //NODE_EDITOR_MAP_VIEW_H
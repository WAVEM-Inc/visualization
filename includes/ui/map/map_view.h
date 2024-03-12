//
// Created by antique on 24. 2. 19.
//

#ifndef NODE_EDITOR_MAP_VIEW_H
#define NODE_EDITOR_MAP_VIEW_H


#include <QWidget>
#include <QWebEngineView>
#include <QPointer>
#include "utils/patterns/observer/observer.h"
#include "struct/GraphNode.h"
#include "struct/RouteFile.h"


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

private slots:
    void onMapNodesChanged(const QMap<std::string, GraphNode> &nodeMap);
};


#endif //NODE_EDITOR_MAP_VIEW_H
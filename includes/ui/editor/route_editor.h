//
// Created by antique on 24. 2. 26.
//

#ifndef NODE_EDITOR_ROUTE_EDITOR_H
#define NODE_EDITOR_ROUTE_EDITOR_H


#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QTableWidget>
#include <QGridLayout>
#include "model/RouteFile.h"
#include "utils/patterns/observer/observer.h"

class RouteEditor : public QWidget {
Q_OBJECT

public:
    explicit RouteEditor(QWidget *parent = nullptr);

    void resizeEvent(QResizeEvent *event) override;

protected:
    class PathListener : public Observer<std::map<std::string, Path>> {
    public:
        explicit PathListener(RouteEditor *editor);

    private:
        RouteEditor *m_editor_ptr;

        void update(std::map<std::string, Path> data) override;
    };

    class SavableStateListener : public Observer<bool> {
    public:
        explicit SavableStateListener(RouteEditor *editor);

    private:
        RouteEditor *m_editor_ptr;

        void update(bool data) override;
    };

private:
    QGridLayout *m_layout_ptr;
    QComboBox *m_routeComboBox_ptr;
    QTableWidget *m_nodeTable_ptr;
    QPushButton *m_addRouteButton_ptr;
    QPushButton *m_addNodeButton_ptr;
    std::shared_ptr<PathListener> m_pathListener_ptr;
    std::shared_ptr<SavableStateListener> m_savableStateListener_ptr;

private slots:
    void onAddRouteButtonClicked();

    void onAddNodeButtonClicked();
};


#endif //NODE_EDITOR_ROUTE_EDITOR_H

#include <QApplication>
#include "ui/main/main_window.h"
#include "includes/model/MapNode.h"
#include "patterns/observer/ConcreteObserver.h"
#include "patterns/observer/subject.h"
#include "viewmodel/map_node_view_model.h"

int main(int argc, char *argv[]) {
    std::shared_ptr<ConcreteObserver> observer1 = std::make_shared<ConcreteObserver>();
    MapNodeViewModel::Instance().attach(observer1);

    QApplication app(argc, argv);

    MainWindow *window = new MainWindow();
    window->resize(800, 600);
    window->show();

    return app.exec();
}

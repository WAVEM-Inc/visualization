#include <QApplication>
#include "ui/main/main_window.h"
#include "utils/patterns/observer/subject.h"
#include "model/MapNode.h"
#include "viewmodel/map_node_view_model.h"

class TestObserver : public Observer<std::map<std::string, Position>> {
    void update(std::map<std::string, Position> data) override {
        nlohmann::json json = data;
        std::cout << json.dump(4) << "\n";
    }
};

int main(int argc, char *argv[]) {
    std::shared_ptr<TestObserver> observer = std::make_shared<TestObserver>();
    MapNodeViewModel::Instance().mapNodes()->attach(observer);

    QApplication app(argc, argv);

    MainWindow *window = new MainWindow();
    window->resize(800, 600);
    window->show();

    return app.exec();
}

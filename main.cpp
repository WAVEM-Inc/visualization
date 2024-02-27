#include <QApplication>
#include "ui/main/main_window.h"
#include "utils/patterns/observer/subject.h"
#include "model/MapNode.h"
#include "viewmodel/map_node_view_model.h"
#include "utils/file/file_manager.h"

class TestObserver : public Observer<std::map<std::string, Position>> {
    void update(std::map<std::string, Position> data) override {
        nlohmann::json json = data;
        std::cout << json.dump(4) << "\n";
    }
};

class FileObserver : public Observer<RouteFile> {
    void update(RouteFile data) override {
        nlohmann::json json = data;
        std::cout << json.dump(4) << "\n";
    }
};

int main(int argc, char *argv[]) {
/*    std::shared_ptr<FileObserver> observer = std::make_shared<FileObserver>();
    FileManager::Instance().getCacheData()->attach(observer);*/

    QApplication app(argc, argv);

    MainWindow *window = new MainWindow();
    window->resize(800, 600);
    window->show();

    return app.exec();
}

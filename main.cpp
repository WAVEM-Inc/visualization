#include <QApplication>
#include <QListView>
#include "ui/main/main_window.h"
#include "utils/patterns/observer/subject.h"
#include "struct/MapNode.h"
#include "utils/file/file_manager.h"
#include "utils/file/code_file_reader.h"
#include "ui/editor/node_list_model.h"
#include "ui/editor/node_editor.h"

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
    QApplication app(argc, argv);

    MainWindow *window = new MainWindow();
    window->resize(800, 600);
    window->show();

    return app.exec();
}

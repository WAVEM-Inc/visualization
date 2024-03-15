#include <QApplication>
#include "ui/main/main_window.h"
#include "utils/file/code_file_reader.h"
#include "utils/file/config_file_writer.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);



    MainWindow *window = new MainWindow();
    window->resize(800, 600);
    window->show();

    QObject::connect(&app, &QApplication::aboutToQuit, []() {
        ConfigFileWriter writer;
        writer.saveFile();
    });

    return app.exec();
}

#include <QApplication>
#include "ui/main/main_window.h"
#include "utils/file/code_file_reader.h"
#include "utils/file/config_file_writer.h"
#include "ros2/nav_sat_fix_subscriber.h"
#include "utils/GeoPositionUtil.h"

int main(int argc, char *argv[]) {
    std::cout << convert_wgs84_to_utm(36.1128827, 128.3703862)[0] << ", " << convert_wgs84_to_utm(36.1128827, 128.3703862)[1] << "\n";
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<NavSatFixSubscriber>();
    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    MainWindow *window = new MainWindow();
    window->resize(800, 600);
    window->show();

    QObject::connect(&app, &QApplication::aboutToQuit, []() {
        ConfigFileWriter writer;
        writer.saveFile();
        
        rclcpp::shutdown();
    });

    int result = app.exec();

    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    return result;
}

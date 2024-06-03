#include <QApplication>
#include "ui/main/main_window.h"
#include "utils/file/code_file_reader.h"
#include "utils/file/config_file_writer.h"
#include "ros2/nav_sat_fix_subscriber.h"
#include "utils/GeoPositionUtil.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<NavSatFixSubscriber>();
    std::thread ros_thread([&]()
    {
        rclcpp::spin(node);
    });

    qRegisterMetaType<sensor_msgs::msg::NavSatFix>("sensor_msgs::msg::NavSatFix");

    MainWindow* window = new MainWindow();
    window->resize(800, 600);
    window->show();

    window->setStyleSheet(
        "QLineEdit {"
        "font: bold 22px;"
        " }"
        ""
        "QLabel {"
        "font: bold 22px;"
        "}"
        ""
        "QComboBox {"
        "font: bold 22px;"
        "}"
        ""
        "QTableView {"
        "font: bold 22px;"
        "}"
        ""
        "QStandardItemModel {"
        "font: bold 22px;"
        "}"
        "QPushButton {"
        "font: bold 22px;"
        "}");

    QPixmap cursor_img = QPixmap("resources/image/cursor.png");
    cursor_img = cursor_img.scaled(32, 32);
    QCursor cursor = QCursor(cursor_img, 0, 0);
    window->setCursor(cursor);

    QObject::connect(&app, &QApplication::aboutToQuit, []()
    {
        ConfigFileWriter writer;
        writer.saveFile();

        rclcpp::shutdown();
    });

    int result = app.exec();

    if (ros_thread.joinable())
    {
        ros_thread.join();
    }

    return result;
}

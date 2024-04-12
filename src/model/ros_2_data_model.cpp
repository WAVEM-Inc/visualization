//
// Created by antique on 24. 3. 15.
//

#include "model/ros_2_data_model.h"

ROS2DataModel::ROS2DataModel(QObject *parent) : QObject(parent), _navSatFix(std::make_shared<sensor_msgs::msg::NavSatFix>()) {

}

void ROS2DataModel::updateNaxSatFixData(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    _navSatFix = msg;
    emit onNavSatFixChanged(*msg);
}

sensor_msgs::msg::NavSatFix ROS2DataModel::getNavSatFixData() const {
    return *_navSatFix;
}

void ROS2DataModel::emergency_button_clicked() {
    emit onEmergencyButtonClicked();
}

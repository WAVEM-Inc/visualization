//
// Created by antique on 24. 3. 15.
//

#include "model/ros_2_data_model.h"

ROS2DataModel::ROS2DataModel(QObject *parent) : QObject(parent) {

}

void ROS2DataModel::updateNaxSatFixData(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    _navSatFix = msg;
}

sensor_msgs::msg::NavSatFix::SharedPtr ROS2DataModel::getNavSatFixData() const {
    return _navSatFix;
}

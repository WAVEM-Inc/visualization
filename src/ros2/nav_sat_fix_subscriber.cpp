//
// Created by antique on 24. 3. 15.
//

#include "ros2/nav_sat_fix_subscriber.h"
#include "model/ros_2_data_model.h"

NavSatFixSubscriber::NavSatFixSubscriber() : rclcpp::Node("route_editor_navsatfix_subscriber") {
    std::cout << "create node" << "\n";
    _subscription_ptr = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "sensor/ublox/fix", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&NavSatFixSubscriber::topicCallback, this, _1));
}

void NavSatFixSubscriber::topicCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const {
    ROS2DataModel::getInstance().updateNaxSatFixData(msg);
}

NavSatFixSubscriber::~NavSatFixSubscriber() = default;

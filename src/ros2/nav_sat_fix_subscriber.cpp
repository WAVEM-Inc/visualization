//
// Created by antique on 24. 3. 15.
//

#include "ros2/nav_sat_fix_subscriber.h"
#include "model/ros_2_data_model.h"

NavSatFixSubscriber::NavSatFixSubscriber() : rclcpp::Node("route_editor_navsatfix_subscriber") {
    std::cout << "create node" << "\n";

    _publisher_ptr = this->create_publisher<can_msgs::msg::Emergency>("/drive/can/emergency", 10);

    _subscription_ptr = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensor/ublox/fix", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&NavSatFixSubscriber::topicCallback, this, _1));

    QObject::connect(&ROS2DataModel::getInstance(), &ROS2DataModel::onEmergencyButtonClicked, [this]() {
        publish();
    });
}

NavSatFixSubscriber::~NavSatFixSubscriber() = default;

void NavSatFixSubscriber::topicCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const {
    ROS2DataModel::getInstance().updateNaxSatFixData(msg);
}

void NavSatFixSubscriber::publish() {
    emergency_state = !emergency_state;

    can_msgs::msg::Emergency msg = can_msgs::msg::Emergency();
    msg.stop = emergency_state;

    std::cout << "Publish to /drive/can/emergency" << "\n";
    std::cout << "Emergency value: " << emergency_state << "\n";

    _publisher_ptr->publish(msg);
}

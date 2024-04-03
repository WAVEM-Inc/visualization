//
// Created by antique on 24. 3. 15.
//

#ifndef ROUTE_EDITOR_NAV_SAT_FIX_SUBSCRIBER_H
#define ROUTE_EDITOR_NAV_SAT_FIX_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

class NavSatFixSubscriber : public rclcpp::Node{
public:
    explicit NavSatFixSubscriber();

    ~NavSatFixSubscriber() override;

private:
    void topicCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const;

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _subscription_ptr;
};


#endif //ROUTE_EDITOR_NAV_SAT_FIX_SUBSCRIBER_H

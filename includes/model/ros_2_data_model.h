//
// Created by antique on 24. 3. 15.
//

#ifndef ROUTE_EDITOR_ROS_2_DATA_MODEL_H
#define ROUTE_EDITOR_ROS_2_DATA_MODEL_H


#include <QObject>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class ROS2DataModel : public QObject {
Q_OBJECT

public:
    static ROS2DataModel &getInstance() {
        static ROS2DataModel instance;
        return instance;
    }

    ROS2DataModel(const ROS2DataModel &) = delete;

    ROS2DataModel &operator=(const ROS2DataModel &) = delete;

    void updateNaxSatFixData(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    sensor_msgs::msg::NavSatFix getNavSatFixData() const;

signals:
    void onNavSatFixChanged(const sensor_msgs::msg::NavSatFix &navSatFix);

private:
    explicit ROS2DataModel(QObject *parent = nullptr);

private:
    sensor_msgs::msg::NavSatFix::SharedPtr _navSatFix;
};


#endif //ROUTE_EDITOR_ROS_2_DATA_MODEL_H

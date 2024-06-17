import $ from "jquery";
import mqtt from "mqtt/*";
import React, { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import Rqtt from "../../api/application/rqtt";
import { RTM_TOPIC_FORMAT } from "../../api/domain/common.constants";
import "./TopComponent.css";
import TopDropDownComponent from "./TopDropdownComponent";

interface TopComponentProps {
    rqtt: Rqtt;
    rqttC: mqtt.MqttClient;
}

const TopComponent: React.FC<TopComponentProps> = ({
    rqtt,
    rqttC
}: TopComponentProps): React.ReactElement<any, any> | null => {
    const [heartBeat, setHeartBeat] = useState<any>();
    const [battery, setBattery] = useState<any>();
    const [_batteryD, setBatteryD] = useState<any>({
        battery: 0,
        status: "red"
    });
    const [isDropdownView, setDropdownView] = useState<boolean>(false);
    const [menuIconRotation, setMenuIconRotation] = useState<number>(0);
    const [ping, setPing] = useState<number>(0.0);
    const [pingStatus, setPingStatus] = useState<string>("#ccc");

    const handleClickContainer: React.MouseEventHandler<HTMLLabelElement> = (): void => {
        setDropdownView(!isDropdownView);
        setMenuIconRotation((prevRotation) => prevRotation === 0 ? 90 : 0);
    }

    const heartBeatCallback: (message: any) => void = (message: any): void => {
        setHeartBeat(message);
    }

    const batteryCallback: (message: any) => void = (message: any): void => {
        setBattery(message);
    }

    useEffect(() => {
        if (battery) {
            let status: string = "";

            if (battery.voltage >= 80) {
                status = "green";
            } else if (battery.voltage >= 50) {
                status = "yellow";
            } else if (battery.voltage >= 20) {
                status = "orange";
            } else {
                status = "red";
            }

            const btry: any = {
                battery: battery.voltage,
                status: status
            }

            setBatteryD(btry);

            $(".top_battery_status").css("background-color", `${btry.status}`);
            $(".top_battery_status").css("width", `${btry.battery}%`);
        }
    }, [battery]);

    useEffect(() => {
        if (heartBeat) {
            setPing(heartBeat.ping_differ);

            if (heartBeat.ping_differ) {
                if (heartBeat.ping_differ === 0.0) {
                    setPingStatus("red");
                } else if (heartBeat.ping_differ <= 300.0) {
                    setPingStatus("green");
                } else if (heartBeat.ping_differ >= 300.0) {
                    setPingStatus("orange");
                } else {
                    setPingStatus("red");
                }
            }
        } else {
            setPingStatus("red");
        }
    }, [heartBeat]);

    useEffect(() => {
        if (rqtt) {
            if (rqttC) {
                const heartBeatTopic: string = `${RTM_TOPIC_FORMAT}/heartbeat`;
                rqtt.subscribe(rqttC, heartBeatTopic);
                rqtt.addSubscriptionCallback(rqttC, heartBeatTopic, heartBeatCallback);

                const batteryTopic: string = `${RTM_TOPIC_FORMAT}/sensor/battery/state`;
                rqtt.subscribe(rqttC, batteryTopic);
                rqtt.addSubscriptionCallback(rqttC, batteryTopic, batteryCallback);
            }
        }
    }, [rqtt, rqttC]);

    return (
        <div className="top_components">
            <label className="top_dropdown_toggle_label" onClick={handleClickContainer} style={{ transform: `rotate(${menuIconRotation}deg)` }}>
                ☰
            </label>
            {isDropdownView && <TopDropDownComponent />}
            <div className="top_logo">
                <p className="top_logo_title">
                    <Link className="top_logo_title_link" to={"/kec/dashboard"}>
                        RMViZ
                    </Link>
                </p>
            </div>
            <div className="top_tools_container">
                <div className="top_battery">
                    <div className="top_battery_status_container">
                        <div className="top_battery_status" />
                        <span className="top_battery_voltage">{_batteryD.battery}%</span>
                    </div>
                </div>
                <div className="top_heartbeat">
                    <div className="top_ping_container">
                        <div className="top_ping_status" style={{ backgroundColor: pingStatus }}></div>
                        <div className="top_ping_data">
                            {ping}ms
                        </div>
                    </div>
                </div>
            </div>
        </div>
    )
}

export default TopComponent;

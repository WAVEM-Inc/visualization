import $ from "jquery";
import React, { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import "./TopComponent.css";
import TopDropDownComponent from "./TopDropdownComponent";

interface TopComponentProps {
    heartBeatData: any;
    batteryData: any;
}

const TopComponents: React.FC<TopComponentProps> = ({
    heartBeatData,
    batteryData
}: TopComponentProps) => {
    const [isDropdownView, setDropdownView] = useState(false);
    const [menuIconRotation, setMenuIconRotation] = useState(0);
    const [ping, setPing] = useState<number>(0.0);
    const [pingStatus, setPingStatus] = useState<string>("#ccc");
    const [battery, setBattery] = useState<any>({
        battery: 0.0,
        status: "red"
    });

    const handleClickContainer = () => {
        setDropdownView(!isDropdownView);
        setMenuIconRotation((prevRotation) => prevRotation === 0 ? 90 : 0);
    }

    useEffect(() => {
        if (batteryData) {
            // console.info(`battery : ${JSON.stringify(battery)}`);
            let status: string = "";

            if (batteryData.voltage >= 80) {
                status = "green";
            } else if (batteryData.voltage >= 50) {
                status = "yellow";
            } else if (batteryData.voltage >= 20) {
                status = "orange";
            } else {
                status = "red";
            }

            const btry: any = {
                battery: batteryData.voltage,
                status: status
            }

            setBattery(btry);

            $(".top_battery_status").css("background-color", `${btry.status}`);
            $(".top_battery_status").css("width", `${battery.battery}%`);
        }
    }, [batteryData]);

    useEffect(() => {
        if (heartBeatData) {
            // console.info(`heartBeat : ${JSON.stringify(heartBeatData)}`);
            setPing(heartBeatData.ping_differ);

            if (ping) {
                if (ping === 0.0) {
                    setPingStatus("red");
                } else if (ping <= 300.0) {
                    setPingStatus("green");
                } else if (ping >= 300.0) {
                    setPingStatus("orange");
                } else {
                    setPingStatus("red");
                }
            }
        } else {
            setPingStatus("red");
        }
    }, [heartBeatData]);

    return (
        <div className="top_components">
            <label className="top_dropdown_toggle_label" onClick={handleClickContainer} style={{ transform: `rotate(${menuIconRotation}deg)` }}>
                ☰
            </label>
            {isDropdownView && <TopDropDownComponent />}
            <div className="top_logo">
                <p className="top_logo_title">
                    <Link className="top_logo_title_link" to={"/bluespace"}>
                        RMViZ
                    </Link>
                </p>
            </div>
            <div className="top_battery">
                <div className="top_battery_status_container">
                    <div className="top_battery_status"/>
                    <span className="top_battery_voltage">{battery.battery}%</span>
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
    )
}

export default TopComponents;

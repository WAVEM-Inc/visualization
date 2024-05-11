import $ from "jquery";
import React, { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { TopState } from "../../domain/top/TopDomain";
import "./TopComponent.css";
import TopDropDownComponent from "./TopDropdownComponent";

interface TopComponentProps {
    state: TopState;
}

const TopComponent: React.FC<TopComponentProps> = ({
    state
}: TopComponentProps): React.ReactElement<any, any> | null => {
    const [isDropdownView, setDropdownView] = useState<boolean>(false);
    const [menuIconRotation, setMenuIconRotation] = useState<number>(0);
    const [ping, setPing] = useState<number>(0.0);
    const [pingStatus, setPingStatus] = useState<string>("#ccc");
    const [battery, setBattery] = useState<any>({
        battery: 0.0,
        status: "red"
    });

    const handleClickContainer: React.MouseEventHandler<HTMLLabelElement> = (): void => {
        setDropdownView(!isDropdownView);
        setMenuIconRotation((prevRotation) => prevRotation === 0 ? 90 : 0);
    }

    useEffect(() => {
        if (state.battery) {
            let status: string = "";

            if (state.battery.voltage >= 80) {
                status = "green";
            } else if (state.battery.voltage >= 50) {
                status = "yellow";
            } else if (state.battery.voltage >= 20) {
                status = "orange";
            } else {
                status = "red";
            }

            const btry: any = {
                battery: state.battery.voltage,
                status: status
            }

            setBattery(btry);

            $(".top_battery_status").css("background-color", `${btry.status}`);
            $(".top_battery_status").css("width", `${battery.battery}%`);
        }
    }, [state.battery]);

    useEffect(() => {
        if (state.heartBeat) {
            setPing(state.heartBeat.ping_differ);

            if (state.heartBeat.ping_differ) {
                if (state.heartBeat.ping_differ === 0.0) {
                    setPingStatus("red");
                } else if (state.heartBeat.ping_differ <= 300.0) {
                    setPingStatus("green");
                } else if (state.heartBeat.ping_differ >= 300.0) {
                    setPingStatus("orange");
                } else {
                    setPingStatus("red");
                }
            }
        } else {
            setPingStatus("red");
        }
    }, [state.heartBeat]);

    return (
        <div className="top_components">
            <label className="top_dropdown_toggle_label" onClick={handleClickContainer} style={{ transform: `rotate(${menuIconRotation}deg)` }}>
                ☰
            </label>
            {isDropdownView && <TopDropDownComponent />}
            <div className="top_logo">
                <p className="top_logo_title">
                    <Link className="top_logo_title_link" to={"/bluespace/dashboard"}>
                        RMViZ
                    </Link>
                </p>
            </div>
            <div className="top_battery">
                <div className="top_battery_status_container">
                    <div className="top_battery_status" />
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

export default TopComponent;

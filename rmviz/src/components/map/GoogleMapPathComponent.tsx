import { useEffect, useState } from "react";
import ReactModal from "react-modal";
import MqttClient from "../../api/mqttClient";
import { MapState } from "../../domain/map/MapDomain";
import { initializeMap } from "../../service/map/MapService";
import "./GoogleMapPathComponent.css";

interface GoogleMapPathComponentProps {
    mqttClient: MqttClient;
    state: MapState;
}

const GoogleMapPathComponent: React.FC<GoogleMapPathComponentProps> = ({
    mqttClient,
    state
}: GoogleMapPathComponentProps) => {
    const [googleMap, setGoogleMap] = useState<google.maps.Map>();
    const kecCoord: google.maps.LatLng = new google.maps.LatLng(36.11434, 128.3690);
    const [isModalOpen, setIsModalOpen] = useState<boolean>(false);
    const [pathIdArray, setPathIdArray] = useState<Array<string>>([]);

    const openModal = () => {
        setIsModalOpen(true);
    }

    const closeModal = () => {
        setIsModalOpen(false);
    }

    useEffect(() => {
        if (state.path) {
            console.info(`Path File : ${JSON.stringify(state.path)}`);
            if (state.path.path) {
                const pathList: Array<any> = Array.from(state.path.path);
                const pathModalElement: HTMLElement | null = document.getElementById("pathModal");

                for (const path of pathList) {
                    console.info(`path : ${JSON.stringify(path)}`);
                    if (pathModalElement) {
                        const button: HTMLButtonElement = document.createElement("button");
                        button.textContent = `${path.id} - ${path.name}`;

                        pathModalElement.appendChild(button);
                    }
                }
            }
        }
    }, [state.path]);

    useEffect(() => {
        const mapElement: HTMLElement | null = document.getElementById("path_map");

        if (mapElement) {
            const mapInstance: google.maps.Map = initializeMap(mapElement, kecCoord);

            setGoogleMap(mapInstance);
        }

        return (() => {
            if (googleMap) {
                googleMap.unbindAll();
            }
            if (state.path) {
                state.path = null;
            }
        });
    }, []);

    return (
        <div className={"path_map_components"}>
            <div className={"path_map_container"}>
                <div id="path_map" />
                <div id="path_info_container" className="path_info_container" />
                <div className="path_data_container">
                    <div className={"path_select_container"}>
                        <button onClick={() => { mqttClient.publish("/rmviz/request/path/select", JSON.stringify({})); openModal(); }}>
                            경로 불러오기
                        </button>
                    </div>
                    <div className={"gps_data_container"}>
                        GPS
                    </div>
                    <div className="node_request_btn_container">
                        <div className="node_request_title">
                            <p>노드 추가</p>
                        </div>
                        <button className={"node_btn_request"}>
                            <img src={process.env.PUBLIC_URL + "../marker_landmark.png"} />
                            <p>연결</p>
                        </button>
                        <button className={"node_btn_request"}>
                            <img src={process.env.PUBLIC_URL + "../marker_stop.png"} />
                            <p>일시 정지</p>
                        </button>
                        <button className={"node_btn_request"}>
                            <img src={process.env.PUBLIC_URL + "../marker_intersection.png"} />
                            <p>교차로</p>
                        </button>
                        <button className={"node_btn_request"}>
                            <img src={process.env.PUBLIC_URL + "../marker_arrive.png"} />
                            <p>종점</p>
                        </button>
                    </div>
                </div>
                <ReactModal
                    id="pathModal"
                    className={"pathModal"}
                    isOpen={isModalOpen}
                    onRequestClose={closeModal}
                >
                    <button onClick={() => { closeModal() }}>닫기</button>

                </ReactModal>
            </div>
        </div>
    );
}

export default GoogleMapPathComponent;
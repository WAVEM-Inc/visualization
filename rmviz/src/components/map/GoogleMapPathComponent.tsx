import { useEffect, useState } from "react";
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

    useEffect(() => {
        const mapElement: HTMLElement | null = document.getElementById("map");

        if (mapElement) {
            const mapInstance: google.maps.Map = initializeMap(mapElement, kecCoord);

            setGoogleMap(mapInstance);
        }

        return (() => {
            if (googleMap) {
                googleMap.unbindAll();
            }
        });
    }, []);

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div id="map"></div>
            </div>
        </div>
    );
}

export default GoogleMapPathComponent;
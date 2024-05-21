import MqttClient from "../../api/mqttClient";
import { MapState } from "../../domain/map/MapDomain";

interface GoogleMapPathComponentProps {
    mqttClient: MqttClient;
    state: MapState;
}

const GoogleMapPathComponent: React.FC<GoogleMapPathComponentProps> = ({
    mqttClient,
    state
}: GoogleMapPathComponentProps) => {

    return (
        <div className={"map_components"}>
            <div className={"map_container"}>
                <div id="map"></div>
            </div>
        </div>
    );
}

export default GoogleMapPathComponent;
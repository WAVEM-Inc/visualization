import { IPublishPacket } from 'mqtt/*';
import React, { useEffect, useState } from 'react';
import JSONInput from 'react-json-editor-ajrm';
import MqttClient from '../../api/mqttClient';
import TopComponent from '../../components/top/TopComponent';
import { TopState } from '../../domain/top/TopDomain';
import "./PathEditPage.css";

const lo_en: any = require("react-json-editor-ajrm/locale/en");

interface PathEditPageProps {
    topState: TopState,
    mqttClient: MqttClient;
}

const PathEditPage: React.FC<PathEditPageProps> = ({
    topState,
    mqttClient
}: PathEditPageProps): React.ReactElement<any, any> | null => {
    const [path, setPath] = useState<any>({});

    useEffect(() => {
        if (mqttClient) {
            const pathEditRequestTopic: string = "/rms/ktp/dummy/request/path/edit";
            mqttClient.publish(pathEditRequestTopic, "");

            const pathEditResponseTopic: string = "/rms/ktp/dummy/response/path/edit";
            mqttClient.subscribe(pathEditResponseTopic);
            mqttClient.client.on("message", (topic: string, payload: Buffer, packet: IPublishPacket) => {
                const message: any = JSON.parse(payload.toString());
                if (topic === pathEditResponseTopic) {
                    console.info(`pathEditResponse : ${JSON.stringify(message)}`);
                    setPath(message);
                } else return;
            });
        }
    }, []);

    return (
        <div className="path_edit_container">
            <div className="top_component_container">
                <TopComponent
                    state={topState}
                />
            </div>
            <div className="path_container">
                <div className="json_edit_conatiner">
                    <JSONInput
                        id="json_text_area"
                        placeholder={path}
                        theme="dark"
                        locale={lo_en}
                        colors={{
                            string: "#DAA520"
                        }}
                    />
                </div>
                <div className="path_map_container">
                    
                </div>
            </div>
        </div>
    );
}

export default PathEditPage;
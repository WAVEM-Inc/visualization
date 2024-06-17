import React, { useEffect } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import "./ROSComponent.css";
import Rqtt from "../../api/application/rqtt";
import mqtt from "mqtt/*";

interface ROSComponentProps {
    rqtt: Rqtt;
    rqttC: mqtt.MqttClient
}

const ROSComponent: React.FC<ROSComponentProps> = ({
    rqtt,
    rqttC
}: ROSComponentProps): React.ReactElement<any, any> | null => {

    let renderer: THREE.WebGLRenderer;
    let scene: THREE.Scene;
    let camera: THREE.PerspectiveCamera;
    let controls: OrbitControls;

    const setUpScene = (container: HTMLElement): void => {
        renderer = new THREE.WebGLRenderer();
        renderer.setSize(container.offsetWidth, container.offsetHeight);
        container.appendChild(renderer.domElement);

        scene = new THREE.Scene();
        camera = new THREE.PerspectiveCamera(70, container.offsetWidth / container.offsetHeight, 1, 10000);
        controls = new OrbitControls(camera, renderer.domElement);

        camera.position.set(0, 20, 100);
        camera.position.z = 5;

        controls.update();

        container.addEventListener('resize', () => {
            renderer.setSize(container.offsetWidth, container.offsetHeight);
            camera.aspect = container.offsetWidth / container.offsetHeight;
            camera.updateProjectionMatrix();
        });

        setAxesHelper();
        setGridHelper();
    }

    const animate = (): void => {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }

    const setAxesHelper = (): void => {
        const axesHelper: THREE.AxesHelper = new THREE.AxesHelper(3);
        axesHelper.position.set(0, 0, 0);
        scene.add(axesHelper);
    }

    const setGridHelper = (): void => {
        const size: number = 10;
        const divisions: number = 10;

        const gridHelper: THREE.GridHelper = new THREE.GridHelper(size, divisions);
        scene.add(gridHelper);
    }

    useEffect(() => {
        const container: HTMLElement | null = document.getElementById('universe_container');

        if (container) {
            setUpScene(container);
        }
    }, []);

    return (
        <div>
            <div id='universe_container' className='universe_container'></div>
        </div>
    );
}

export default ROSComponent;
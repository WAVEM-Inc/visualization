import * as rclnodejs from "rclnodejs";
import Rqtt from "./common/application/rqtt";
import SensorController from "./sensor/presentation/sensor.controller";
import mqtt from "mqtt/*";
import RouteController from "./route/presentation/route.controller";
import RtMController from "./common/presentation/rtm.controller";
import HeartBeatController from "./heartbeat/presentation/heartbeat.controller";

const NODE_NAME: string = "rmserver_rqtt";

class RMserverRqttNode extends rclnodejs.Node {
  constructor() {
    super(NODE_NAME);
    this.getLogger().info(`${this.name()} created`);
  }
}

async function initializeNode(): Promise<void> {
  await rclnodejs.init();
  const node: rclnodejs.Node = new RMserverRqttNode();
  const rqtt: Rqtt = new Rqtt(node);
  const rqttC: mqtt.MqttClient = await rqtt.initialize();

  const rtmController: RtMController = new RtMController(rqttC, node);
  const sensorController: SensorController = new SensorController(node);
  const routeController: RouteController = new RouteController(rqttC, node);
  const heartBeatController: HeartBeatController = new HeartBeatController(rqttC, node);
  node.spin();
}

(async function main(): Promise<void> {
  await initializeNode();
})().catch((): void => {
  rclnodejs.shutdown();
  process.exit();
});

process.on("SIGINT", (): void => {
  console.log("Terminated by CTRL-C");
  rclnodejs.shutdown();
  process.exit();
});
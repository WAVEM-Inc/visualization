import * as rclnodejs from "rclnodejs";
import Rqtt from "./common/application/rqtt";
import SensorController from "./sensor/presentation/sensor.controller";
import mqtt from "mqtt/*";

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

  new SensorController(rqttC, node);
  node.spin();
}

(async function main(): Promise<void> {
  await initializeNode();
})().catch((): void => {
  rclnodejs.shutdown();
  process.exitCode = 1;
});

process.on("SIGINT", () => {
  console.log("Terminated by CTRL-C");
  rclnodejs.shutdown();
  process.exit();
});
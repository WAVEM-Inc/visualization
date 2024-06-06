import * as rclnodejs from "rclnodejs";
import Rqtt from "./common/application/rqtt";
import mqtt from "mqtt/*";
import RtMController from "./common/presentation/rtm.controller";
import MtRController from "./common/presentation/mtr.controller";

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
  const mtrController: MtRController = new MtRController(rqttC, node);

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
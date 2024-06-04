import * as rclnodejs from "rclnodejs";

const NODE_NAME: string = "rmserver_rqtt";

class RMserverRqttNode extends rclnodejs.Node {
  constructor() {
    super(NODE_NAME);
    this.getLogger().info(`${this.name()} created`);
  }
}

async function initialize_node(): Promise<void> {
  await rclnodejs.init();
  const node: rclnodejs.Node = new RMserverRqttNode();
  node.spin();
}

(async function main(): Promise<void> {
  initialize_node();
})().catch((): void => {
  process.exitCode = 1;
});
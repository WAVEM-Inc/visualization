import threading;
from rclpy.node import Node;

from rmserver_rqtt.application.mqtt import Client;
from rmserver_rqtt.presentation.request.bridge import RequestBridge;
from rmserver_rqtt.presentation.response.bridge import ResponseBridge;

NODE_NAME: str = "rmserver_kec";
DEFAULT_STRING: str = "";
DEFAULT_INT: int = 0;
DEFAULT_FLOAT: float = 0.0;


class RmServerKEC(Node):

    def __init__(self) -> None:
        super().__init__(node_name=NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        self.declare_mqtt_parameters();
        __mqtt_client: Client = Client(node=self);
        __mqtt_client.connect();

        def mqtt_thread_cb() -> None:
            __mqtt_client.run();

        mqtt_thread: threading.Thread = threading.Thread(target=mqtt_thread_cb);
        mqtt_thread.start();

        request_bridge: RequestBridge = RequestBridge(node=self, mqtt_client=__mqtt_client);
        response_bridge: ResponseBridge = ResponseBridge(node=self, mqtt_client=__mqtt_client);

    def declare_mqtt_parameters(self) -> None:
        parameters_dict: dict = {
            "current_mqtt_file": DEFAULT_STRING,
            "current_map_config_file": DEFAULT_STRING,
            "current_urdf_file": DEFAULT_STRING
        };

        for key, value in parameters_dict.items():
            self.get_logger().info(f"{self.get_name()} Declaring key : [{key}], value : [{value}]");
            self.declare_parameter(name=key, value=value);


__all__ = ["RmServerKEC"];

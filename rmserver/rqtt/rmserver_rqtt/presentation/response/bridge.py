from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rmserver_rqtt.application.mqtt import Client;
from rmserver_rqtt.application.response.graph import GraphProcessor;
from rmserver_rqtt.application.response.obstacle import ObstacleProcessor;
from rmserver_rqtt.application.response.report import ReportProcessor;
from rmserver_rqtt.application.response.sensor import SensorProcessor;
from rmserver_rqtt.application.response.status import StatusProcessor;


class ResponseBridge:

    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__mqtt_client: Client = mqtt_client;
        
        graph_processor: GraphProcessor = GraphProcessor(node=self.__node, mqtt_client=self.__mqtt_client);
        obstacle_processor: ObstacleProcessor = ObstacleProcessor(node=self.__node, mqtt_client=self.__mqtt_client);
        report_processor: ReportProcessor = ReportProcessor(node=self.__node, mqtt_client=self.__mqtt_client);
        sensor_processor: SensorProcessor = SensorProcessor(node=self.__node, mqtt_client=self.__mqtt_client);
        status_processor: StatusProcessor = StatusProcessor(node=self.__node, mqtt_client=self.__mqtt_client);


__all__: list[str] = ["ResponseBridge"];
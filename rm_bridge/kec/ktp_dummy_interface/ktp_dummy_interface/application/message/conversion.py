import json;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from ktp_dummy_interface.application.message.filter import filter_empty_values;
from rosbridge_library.internal import message_conversion;
from typing import Any;

def ros_message_to_json(log: RcutilsLogger, ros_message: Any) -> str:
    try:
        extracted_json: Any = message_conversion.extract_values(inst=ros_message);
        filtered_json: Any = filter_empty_values(extracted_json);
        dumped_json: str = json.dumps({k: v for k, v in filtered_json.items() if v is not None});
        
        return dumped_json;
    except message_conversion.NonexistentFieldException as nefe:
        log.error(f"ros_to_json : {nefe}");
        return None;

def json_to_ros_message(log: RcutilsLogger, json_payload: Any, target_ros_class: Any) -> Any:
    try:
        ros_message: Any = message_conversion.populate_instance(msg=json_payload, inst=target_ros_class());
        log.info(f"\n{json.dumps(obj=message_conversion.extract_values(inst=ros_message), indent=4)}");
        return ros_message;
    except ValueError as ve:
        log.error(f"json_to_ros_message : {ve}");
        return None;
    except KeyError as ke:
        log.error(f"json_to_ros_message : {ke}");
        return None;
    except message_conversion.NonexistentFieldException as nefe:
        log.error(f"json_to_ros_message : {nefe}");
        return None;
    except Exception as e:
        log.error(f"json_to_ros_message : {e}");
        return None;
    
__all__: list[str] = ["conversion"];
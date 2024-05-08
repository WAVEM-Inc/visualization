from typing import Any;

_control: Any = {};
_control_callback_flag: bool = False;

_mission: Any = {};
_mission_callback_flag: bool = False;

_detected_object: Any = {};
_detected_object_callback_flag: bool = False;

@staticmethod
def get_control() -> Any:
    return _control;

@staticmethod
def set_control(control: Any) -> None:
    global _control;
    _control = control;

@staticmethod
def get_control_callback_flag() -> bool:
    return _control_callback_flag;

@staticmethod
def set_control_callback_flag(control_callback_flag: bool) -> None:
    global _control_callback_flag;
    _control_callback_flag = control_callback_flag;

@staticmethod
def get_mission() -> Any:
    return _mission;

@staticmethod
def set_mission(mission: Any) -> None:
    global _mission;
    _mission = mission;

@staticmethod
def get_mission_callback_flag() -> bool:
    return _mission_callback_flag;

@staticmethod
def set_mission_callback_flag(mission_callback_flag: bool) -> None:
    global _mission_callback_flag;
    _mission_callback_flag = mission_callback_flag;

@staticmethod
def get_detected_object() -> Any:
    return _detected_object;

@staticmethod
def set_detected_object(detected_object: Any) -> None:
    global _detected_object;
    _detected_object = detected_object;

@staticmethod
def get_detected_object_flag() -> bool:
    return _detected_object_callback_flag;

@staticmethod
def set_detected_object_flag(detected_object_flag: bool) -> None:
    global _detected_object_callback_flag;
    _detected_object_callback_flag = detected_object_flag;
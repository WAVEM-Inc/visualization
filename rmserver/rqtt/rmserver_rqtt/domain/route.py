from typing import Tuple;


class RouteStatus:
    
    def __init__(self) -> None:
        self._is_driving: bool = False
        """
        0 : 출발
        1 : 경유지 도착
        2 : 경로 주행 완료
        3 : 주행 서버 미작동
        4 : 주행 진행 중
        5 : 주행 취소
        """
        self._status_code: int = 0
        self._node_index: int = 0
        self._node_info: Tuple[str, str] = ("", "")
    
    @property
    def is_driving(self) -> bool:
        return self._is_driving
    
    @is_driving.setter
    def is_driving(self, is_driving: bool) -> None:
        self._is_driving = is_driving
        
    @property
    def status_code(self) -> int:
        return self._status_code
    
    @status_code.setter
    def status_code(self, status_code: int) -> None:
        self._status_code = status_code
        
    @property
    def node_index(self) -> int:
        return self._node_index
    
    @node_index.setter
    def node_index(self, node_index: int) -> None:
        self._node_index = node_index
        
    @property
    def node_info(self) -> Tuple[str, str]:
        return self._node_info
    
    @node_info.setter
    def node_info(self, node_info: Tuple[str, str]) -> None:
        self._node_info = node_info


__all__: list[str] = ["RouteStatus"];
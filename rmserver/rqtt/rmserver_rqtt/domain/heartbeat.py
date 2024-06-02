
class HeartBeat:
    def __init__(self) -> None:
        self._request_time: str = "";
        self._response_time: str = "";
        self._ping: float = 0.0;
        
    @property
    def request_time(self) -> str:
        return self._request_time;
    
    @request_time.setter
    def request_time(self, request_time: str) -> None:
        self._request_time = request_time;
        
    @property
    def response_time(self) -> str:
        return self._response_time;
    
    @response_time.setter
    def response_time(self, response_time: str) -> None:
        self._response_time = response_time;


__all__: list[str] = ["HeartBeat"];
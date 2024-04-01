package application.type.option

data class ConnectOption (
    var ipAddress: String,
    var sendPort: Int,
    var receivePort: Int
)
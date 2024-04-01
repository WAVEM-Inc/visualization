package application.type.data

data class LidarCalibration(
    val translation: Position = Position(),
    val rotation: Quaternion = Quaternion()
)

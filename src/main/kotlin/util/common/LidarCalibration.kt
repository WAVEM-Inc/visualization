package util.common

data class LidarCalibration(
    val translation: Position = Position(),
    val rotation: Quaternion = Quaternion()
)

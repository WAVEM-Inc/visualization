package util.common

import ui.opengl.math.Coordinate3D

data class ObjectInfo(
    var id: Int = 0,
    var type: String = "",
    var heading: Double = 0.0,
    var position: Position = Position(),
    var ttc: String = "",
    var risk: String = "",
    var textDrawPosition: Coordinate3D
)
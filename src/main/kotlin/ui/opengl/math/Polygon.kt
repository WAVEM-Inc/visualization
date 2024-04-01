package ui.opengl.math

data class Polygon(
    var points: MutableList<Coordinate3D> = ArrayList(),
    var width: Double = 0.0,
    var height: Double = 0.0,
    var position: Coordinate3D = Coordinate3D()
) {
}

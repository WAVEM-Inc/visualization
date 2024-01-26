package ui.opengl.math

import kotlin.math.sqrt

data class Coordinate3D(var x: Double = 0.0, var y: Double = 0.0, var z: Double = 0.0) {
    fun distanceFromOther(other: Coordinate3D): Double {
        val dx = x - other.x
        val dy = y - other.y
        val dz = z - other.z

        return sqrt((dx * dx) + (dy * dy) + (dz * dz))
    }

    fun set(x: Double, y: Double, z: Double) {
        this.x = x
        this.y = y
        this.z = z
    }

    fun set(coordinate: Coordinate3D) {
        this.x = coordinate.x
        this.y = coordinate.y
        this.z = coordinate.z
    }
}
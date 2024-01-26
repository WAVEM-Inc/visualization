package ui.opengl.math

import kotlin.math.sqrt

data class Coordinate2D(var x: Double, var y: Double) {
    fun distanceFromOther(other: Coordinate2D): Double {
        val dx = x - other.x
        val dy = y - other.y

        return sqrt((dx * dx) + (dy * dy))
    }

    fun set(x: Double, y: Double) {
        this.x = x
        this.y = y
    }

    fun set(coordinate: Coordinate2D) {
        this.x = coordinate.x
        this.y = coordinate.y
    }
}
package ui.opengl.math

data class Vect3D(var x: Double = 0.0, var y: Double = 0.0, var z: Double = 0.0) {
    fun set(x: Double, y: Double, z: Double) {
        this.x = x
        this.y = y
        this.z = z
    }

    fun set(vector: Vect3D) {
        this.x = vector.x
        this.y = vector.y
        this.z = vector.z
    }
}

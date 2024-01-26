package ui.opengl.math

data class GLColor(var red: Float = 0f, var green: Float = 0f, var blue: Float = 0f, var alpha: Float = 1f) {
    constructor(htmlColor: String) : this() {
        val color = htmlColor.substring(1) // Remove the '#' character
        this.red= Integer.parseInt(color.substring(0, 2), 16) / 255.0f
        this.green = Integer.parseInt(color.substring(2, 4), 16) / 255.0f
        this.blue = Integer.parseInt(color.substring(4, 6), 16) / 255.0f
    }
    fun htmlToRgb(htmlColor: String): FloatArray {
        val color = htmlColor.substring(1) // Remove the '#' character
        val r = Integer.parseInt(color.substring(0, 2), 16) / 255.0f
        val g = Integer.parseInt(color.substring(2, 4), 16) / 255.0f
        val b = Integer.parseInt(color.substring(4, 6), 16) / 255.0f
        return floatArrayOf(r, g, b)
    }
}

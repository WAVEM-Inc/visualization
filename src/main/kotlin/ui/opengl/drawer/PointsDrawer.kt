package ui.opengl.drawer

import androidx.compose.ui.graphics.Color
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic
import com.jogamp.opengl.GL2
import ui.opengl.math.Coordinate2D
import ui.opengl.math.Coordinate3D
import ui.opengl.math.GLColor

class PointsDrawer(
    private val pointSize: Float = 1f,
    private val farColor: Color = Color.Green,
    private val nearColor: Color = Color.Red,
    private val useDynamicColor: Boolean = true,
    private val maxDistance: Int = 30,
    private val points: List<Coordinate3D>
) : GLDrawer {
    private val originCoordinate = Coordinate3D(0.0, 0.0, 0.0)
    private var nearColorR = nearColor.red
    private var nearColorG = nearColor.green
    private var nearColorB = nearColor.blue

    private var farColorR = farColor.red
    private var farColorG = farColor.green
    private var farColorB = farColor.blue

    override fun draw(gl: GL2) {
        // Initialize GL point size
        gl.glPointSize(pointSize)
        gl.glBegin(GL2.GL_POINTS)

        if (useDynamicColor) {
            for (i in points.indices) {
                val pos = points[i]
                val color = getLinearInterpolationColor(pos.distanceFromOther(originCoordinate))
                gl.glColor3f(color.red, color.green, color.blue)
                gl.glVertex3d(pos.x, pos.y, pos.z)
            }
        } else {
            val color = farColor
            gl.glColor3f(color.red, color.green, color.blue)

            for (i in points.indices) {
                val pos = points[i]
                gl.glVertex3d(pos.x, pos.y, pos.z)
            }
        }


        gl.glEnd()

        // Clear GL point size
        gl.glPointSize(1f)
    }

    fun getLinearInterpolationColor(distance: Double): GLColor {
        val interpolationFactor: Float = distance.toFloat() / maxDistance
        val r = (nearColorR + interpolationFactor * (farColorR - nearColorR))
        val g = (nearColorG + interpolationFactor * (farColorG - nearColorG))
        val b = (nearColorB + interpolationFactor * (farColorB - nearColorB))

        return GLColor(r, g, b)
    }
}
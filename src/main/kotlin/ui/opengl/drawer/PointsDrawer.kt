package ui.opengl.drawer

import androidx.compose.ui.graphics.Color
import com.jogamp.opengl.GL2
import ui.opengl.math.Coordinate3D
import ui.opengl.math.GLColor
import kotlin.math.cos
import kotlin.math.sin

//class PointsDrawer(
//    private val pointSize: Float = 1f,
//    private val farColor: Color = Color.Green,
//    private val nearColor: Color = Color.Red,
//    private val useDynamicColor: Boolean = true,
//    private val maxDistance: Int = 30,
//    private val points: List<Coordinate3D>
//) : GLDrawer {
//    private val originCoordinate = Coordinate3D(0.0, 0.0, 0.0);
//    private var nearColorR = nearColor.red;
//    private var nearColorG = nearColor.green;
//    private var nearColorB = nearColor.blue;
//
//    private var farColorR = farColor.red;
//    private var farColorG = farColor.green;
//    private var farColorB = farColor.blue;
//
//    private val colorCache = Array<GLColor?>(maxDistance + 1) { null }
//
//    override fun draw(gl: GL2) {
//        gl.glPointSize(pointSize);
//        gl.glBegin(GL2.GL_POINTS);
//
//        for (i in points.indices) {
//            val pos = points[i];
//            val color = if (useDynamicColor) getLinearInterpolationColor(pos.distanceFromOther(originCoordinate)) else GLColor(farColor.red, farColor.green, farColor.blue);
//            gl.glColor3f(color.red, color.green, color.blue);
//            gl.glVertex3d(pos.x, pos.y, pos.z);
//        }
//
//        gl.glEnd();
//        gl.glPointSize(1f); // Reset the point size to default
//    }
//
//    fun getLinearInterpolationColor(distance: Double): GLColor {
//        val distanceIndex = (distance / maxDistance * maxDistance).toInt();
//        return colorCache[distanceIndex] ?: interpolateColor(distanceIndex).also { colorCache[distanceIndex] = it };
//    }
//
//    private fun interpolateColor(distanceIndex: Int): GLColor {
//        val interpolationFactor: Float = distanceIndex.toFloat() / maxDistance;
//        val r = (nearColorR + interpolationFactor * (farColorR - nearColorR));
//        val g = (nearColorG + interpolationFactor * (farColorG - nearColorG));
//        val b = (nearColorB + interpolationFactor * (farColorB - nearColorB));
//
//        return GLColor(r, g, b);
//    }
//}

class PointsDrawer(
    private val pointSize: Float = 1f,
    private val farColor: Color = Color.Green,
    private val nearColor: Color = Color.Red,
    private val useDynamicColor: Boolean = true,
    private val maxDistance: Int = 30,
    private val heading: Double = 0.0,
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

                // Rotate 90 degrees
//                val angleRadians = Math.PI / 2 + heading
                val angleRadians = heading
                val cosTheta = cos(angleRadians)
                val sinTheta = sin(angleRadians)

                val x = pos.x * cosTheta - pos.y * sinTheta
                val y = pos.x * sinTheta + pos.y * cosTheta

                gl.glColor3f(color.red, color.green, color.blue)
                gl.glVertex3d(x, y, pos.z)
            }
        } else {
            val color = farColor
            gl.glColor3f(color.red, color.green, color.blue)

            for (i in points.indices) {
                val pos = points[i]

                // Rotate 90 degrees
                val angleRadians = Math.PI / 2 + heading
                val cosTheta = cos(angleRadians)
                val sinTheta = sin(angleRadians)

                val x = pos.x * cosTheta - pos.y * sinTheta
                val y = pos.x * sinTheta + pos.y * cosTheta

                gl.glVertex3d(x, y, pos.z)
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
package ui.opengl.drawer

import com.jogamp.opengl.GL2
import ui.opengl.math.Coordinate3D
import ui.opengl.math.GLColor

class LineDrawer(
    val startPoint: Coordinate3D = Coordinate3D(),
    val endPoint: Coordinate3D = Coordinate3D(),
    val color: GLColor = GLColor(),
    val lineWidth: Float = 1f
) : GLDrawer {

    override fun draw(gl: GL2) {
        // Draw line
        gl.glLineWidth(lineWidth)
        gl.glBegin(GL2.GL_LINES)
        gl.glColor4f(color.red, color.green, color.blue, color.alpha)

        gl.glVertex3d(startPoint.x, startPoint.y, startPoint.z)
        gl.glVertex3d(endPoint.x, endPoint.y, endPoint.z)

        gl.glEnd()
    }

}
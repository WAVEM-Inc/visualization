package ui.opengl.drawer

import com.jogamp.opengl.GL2
import com.jogamp.opengl.GLAutoDrawable
import com.jogamp.opengl.util.awt.TextRenderer
import ui.opengl.math.GLColor
import java.awt.Font

class GridLineDrawer(
    width: Int = 10,
    height: Int = 10,
    var lineWidth: Float = 1f,
    var lineColor: GLColor = GLColor(),
    val step: Int = 5,
    var backgroundColor: GLColor = GLColor()
) : GLDrawer {
    private val left = -(width / 2)
    private val right = (width / 2)
    private val top = (height / 2)
    private val bottom = -(height / 2)

    override fun draw(gl: GL2) {
/*        // Draw background rectangle with backgroundColor
        gl.glBegin(GL2.GL_QUADS)
        gl.glColor4f(backgroundColor.red, backgroundColor.green, backgroundColor.blue, backgroundColor.alpha)

        gl.glVertex2i(left, top)
        gl.glVertex2i(right, top)
        gl.glVertex2i(right, bottom)
        gl.glVertex2i(left, bottom)

        gl.glEnd()*/

        // Draw lines
        gl.glLineWidth(lineWidth)
        gl.glBegin(GL2.GL_LINES)
        gl.glColor4f(lineColor.red, lineColor.green, lineColor.blue, lineColor.alpha)

        for (i in left .. right step step) {
            gl.glVertex2i(i, top)
            gl.glVertex2i(i, bottom)
        }

        for (i in bottom .. top step step) {
            gl.glVertex2i(left, i)
            gl.glVertex2i(right, i)
        }

        gl.glEnd()
    }
}
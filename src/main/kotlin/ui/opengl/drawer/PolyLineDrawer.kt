package ui.opengl.drawer

import application.type.data.ObjectInfo
import application.type.option.ObstacleOption
import com.jogamp.opengl.GL2
import com.jogamp.opengl.awt.GLJPanel
import com.jogamp.opengl.glu.GLU
import com.jogamp.opengl.util.awt.TextRenderer
import ui.opengl.math.Coordinate3D
import ui.opengl.math.GLColor
import ui.opengl.math.Polygon
import viewmodel.UdpDataViewModel

class PolyLineDrawer(
    val objects: Map<Int, Polygon>,
    val objectsInfo: Map<Int, ObjectInfo>,
    val color: GLColor = GLColor("#00FF3C"),
    val textRenderer: TextRenderer,
    val obstacleOption: ObstacleOption,
    val glu: GLU,
    val parentPanel: GLJPanel
) : GLDrawer {
    override fun draw(gl: GL2) {
        for (objectData in objects) {
            gl.glBegin(GL2.GL_LINE_LOOP)
            gl.glColor3f(color.red, color.green, color.blue)

            for (polygon in objectData.value.points) {
                gl.glVertex3d(polygon.x, polygon.y, polygon.z)
            }

            gl.glEnd()
            gl.glBegin(GL2.GL_LINE_LOOP)

            for (polygon in objectData.value.points) {
                gl.glVertex3d(polygon.x, polygon.y, polygon.z + objectData.value.height)
            }

            gl.glEnd()
            gl.glBegin(GL2.GL_LINES)

            for (polygon in objectData.value.points) {
                gl.glVertex3d(polygon.x, polygon.y, polygon.z)
                gl.glVertex3d(polygon.x, polygon.y, polygon.z + objectData.value.height)
            }
            gl.glEnd()

            val value = objectsInfo[objectData.key]
            var index = 0

            if (value != null) {
                if (obstacleOption.useId) {
                    renderText(gl, "ID: ${value.id}", index, objectData.value)
                    index++
                }

                if (obstacleOption.useType) {
                    renderText(gl, "Type: ${value.type}", index, objectData.value)
                    index++
                }

                if (obstacleOption.usePosition) {
                    renderText(gl, "Position", index, objectData.value)
                    index++
                    renderText(gl, "    - X: %.2f".format(value.position.x), index, objectData.value)
                    index++
                    renderText(gl, "    - Y: %.2f".format(value.position.y), index, objectData.value)
                    index++
                    renderText(gl, "    - Z: %.2f".format(value.position.z), index, objectData.value)
                    index++
                }

                if (obstacleOption.useSpeed) {
                    renderText(gl, "Speed: %.2f".format(value.speed), index, objectData.value)
                    index++
                }

                if (obstacleOption.useHeading) {
                    renderText(gl, "Heading: %.2f".format(value.heading), index, objectData.value)
                    index++
                }

                if (obstacleOption.useTtc) {
                    renderText(gl, "TTC: ${value.ttc}", index, objectData.value)
                    index++
                }

                if (obstacleOption.useRisk) {
                    renderText(gl, "Risk: ${value.risk}", index, objectData.value)
                }
            }
        }
    }

    private fun renderText(gl: GL2, text: String, index: Int, objectData: Polygon) {
        drawTextBillboard(
            gl, glu, textRenderer, text,
            objectData.position.x.toFloat() + objectData.width.toFloat(),
            objectData.position.y.toFloat() + objectData.height.toFloat(),
            objectData.position.z.toFloat() + objectData.height.toFloat(),
            index
        )
//        drawTextBillboard(gl, glu, textRenderer, text)
//        textRenderer.draw3D(
//            text,
//            objectData.position.x.toFloat() + objectData.width.toFloat(),
//            objectData.position.y.toFloat() + objectData.height.toFloat() - (textRenderer.font.size * 0.025f * index),
//            objectData.position.z.toFloat()+ objectData.height.toFloat(),
//            0.02f
//        )
    }

    fun projectToScreen(gl: GL2, glu: GLU, x: Float, y: Float, z: Float): FloatArray? {
        val modelview = FloatArray(16)
        val projection = FloatArray(16)
        val viewport = IntArray(4)
        val screenPos = FloatArray(3)

        gl.glGetFloatv(GL2.GL_MODELVIEW_MATRIX, modelview, 0)
        gl.glGetFloatv(GL2.GL_PROJECTION_MATRIX, projection, 0)
        gl.glGetIntegerv(GL2.GL_VIEWPORT, viewport, 0)

        val result = glu.gluProject(x, y, z, modelview, 0, projection, 0, viewport, 0, screenPos, 0)
        return if (result) screenPos else null
    }

    fun drawTextBillboard(
        gl: GL2,
        glu: GLU,
        textRenderer: TextRenderer,
        text: String,
        x: Float,
        y: Float,
        z: Float,
        index: Int
    ) {
        val screenPos = projectToScreen(gl, glu, x, y, z)
        if (screenPos != null) {
            val screenX = screenPos[0]
            val screenY = screenPos[1] // OpenGL's Y axis is inverted

            textRenderer.beginRendering(parentPanel.width, parentPanel.height)
            textRenderer.draw(
                text, screenX.toInt(),
                screenY.toInt() - (textRenderer.font.size * index)
            )
            textRenderer.endRendering()
        }
    }
}

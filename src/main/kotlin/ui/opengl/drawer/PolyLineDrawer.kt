package ui.opengl.drawer

import application.type.data.ObjectInfo
import application.type.option.ObstacleOption
import com.jogamp.opengl.GL2
import com.jogamp.opengl.util.awt.TextRenderer
import ui.opengl.math.GLColor
import ui.opengl.math.Polygon

class PolyLineDrawer(
    val objects: Map<Int, Polygon>,
    val objectsInfo: Map<Int, ObjectInfo>,
    val color: GLColor = GLColor("#00FF3C"),
    val textRenderer: TextRenderer,
    val obstacleOption: ObstacleOption
): GLDrawer {
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
                textRenderer.begin3DRendering()
                textRenderer.setColor(1f, 1f, 1f, 1f)

                if (obstacleOption.useId) {
                    renderText("ID: ${value.id}", index, objectData.value)
                    index++
                }

                if (obstacleOption.useType) {
                    renderText("Type: ${value.type}", index, objectData.value)
                    index++
                }

                if (obstacleOption.usePosition) {
                    renderText("Position", index, objectData.value)
                    index++
                    renderText("    - X: ${value.position.x}", index, objectData.value)
                    index++
                    renderText("    - Y: ${value.position.y}", index, objectData.value)
                    index++
                    renderText("    - Z: ${value.position.z}", index, objectData.value)
                    index++
                }

                if (obstacleOption.useHeading) {
                    renderText("Heading: ${value.heading}", index, objectData.value)
                    index++
                }

                if (obstacleOption.useTtc) {
                    renderText("TTC: ${value.ttc}", index, objectData.value)
                    index++
                }

                if (obstacleOption.useRisk) {
                    renderText("Risk: ${value.risk}", index, objectData.value)
                }

                textRenderer.end3DRendering()
            }
        }
    }

    private fun renderText(text: String, index: Int, objectData: Polygon) {
        textRenderer.draw3D(
            text,
            objectData.position.x.toFloat() + objectData.width.toFloat(),
            objectData.position.y.toFloat() + objectData.height.toFloat() - (textRenderer.font.size * 0.025f * index),
            objectData.position.z.toFloat()+ objectData.height.toFloat(),
            0.02f
        )
    }
}
package ui.opengl.camera

import com.jogamp.opengl.GL2
import com.jogamp.opengl.glu.GLU
import ui.opengl.math.Coordinate3D

fun unProjectScreenCoordinate(gl: GL2, x: Int, y: Int, camPos: Coordinate3D): Coordinate3D? {
    val glu = GLU.createGLU(gl)

    val viewport = IntArray(4)
    val modelView = DoubleArray(16)
    val project = DoubleArray(16)

    gl.glGetIntegerv(GL2.GL_VIEWPORT, viewport, 0)
    gl.glGetDoublev(GL2.GL_MODELVIEW_MATRIX, modelView, 0)
    gl.glGetDoublev(GL2.GL_PROJECTION_MATRIX, project, 0)

    val objPos = DoubleArray(4)

    val winX = x.toDouble()
    val winY = viewport[3] - y.toDouble() - 1

    glu.gluUnProject(winX, winY, 0.0, modelView, 0, project, 0, viewport, 0, objPos, 0)

    return getCrossXYPlanCoordinate(camPos, Coordinate3D(objPos[0], objPos[1], objPos[2]))
}

fun getCrossXYPlanCoordinate(p1: Coordinate3D, p2: Coordinate3D): Coordinate3D? {
    val dz = p2.z - p1.z

    if (dz == 0.0) {
        return null
    }

    val t = -p1.z / dz
    val x = p1.x + t * (p2.x - p1.x)
    val y = p1.y + t * (p2.y - p1.y)

    return Coordinate3D(x, y, 0.0)
}
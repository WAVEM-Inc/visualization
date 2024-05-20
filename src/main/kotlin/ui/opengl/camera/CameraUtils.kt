package ui.opengl.camera

import com.jogamp.opengl.GL2
import com.jogamp.opengl.glu.GLU
import ui.opengl.math.Coordinate3D
import java.nio.FloatBuffer
import java.nio.IntBuffer



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

fun worldToScreen(gl: GL2, x: Float, y: Float, z: Float): FloatArray? {
    val glu = GLU()
    val viewport = IntArray(4)
    val modelview = DoubleArray(16)
    val projection = DoubleArray(16)
    val screenCoords = DoubleArray(3)

    // 뷰포트, 모델뷰 행렬 및 프로젝션 행렬 업데이트
    gl.glGetIntegerv(GL2.GL_VIEWPORT, viewport, 0)
    gl.glGetDoublev(GL2.GL_MODELVIEW_MATRIX, modelview, 0)
    gl.glGetDoublev(GL2.GL_PROJECTION_MATRIX, projection, 0)

    // 3D 월드 좌표를 2D 화면 좌표로 변환
    val result = glu.gluProject(
        x.toDouble(), y.toDouble(), z.toDouble(),
        modelview, 0,
        projection, 0,
        viewport, 0,
        screenCoords, 0
    )

    return if (result) {
        floatArrayOf(screenCoords[0].toFloat(), screenCoords[1].toFloat(), screenCoords[2].toFloat())
    } else {
        null // 좌표 변환 실패 시 null 반환
    }
}

fun getScreenCoords(gl: GL2, glu: GLU, x: Double, y: Double, z: Double): IntArray? {
    val screenCoords: FloatBuffer = FloatBuffer.allocate(4)
    val viewport: IntBuffer = IntBuffer.allocate(16)
    val modelView: FloatBuffer = FloatBuffer.allocate(16)
    val projection: FloatBuffer = FloatBuffer.allocate(16)

    gl.glGetFloatv(GL2.GL_MODELVIEW_MATRIX, modelView)
    gl.glGetFloatv(GL2.GL_PROJECTION_MATRIX, projection)
    gl.glGetIntegerv(GL2.GL_VIEWPORT, viewport)

    val result: Boolean =
        glu.gluProject(x.toFloat(), y.toFloat(), z.toFloat(), modelView, projection, viewport, screenCoords)

    if (result) {
        return intArrayOf(screenCoords[0].toInt(), screenCoords[1].toInt())
    } else {
        return null
    }
}
package ui.opengl.camera

import com.jogamp.opengl.GL2
import com.jogamp.opengl.glu.GLU
import com.jogamp.opengl.math.Vec2i
import ui.opengl.math.Coordinate3D
import ui.opengl.math.Perspective
import ui.opengl.math.Vect3D
import java.awt.event.*
import javax.swing.SwingUtilities
import kotlin.math.cos
import kotlin.math.sin

class SphereCamera (private val gl: GL2, private val glu: GLU = GLU.createGLU(gl)) {
    private val camPos = Coordinate3D(0.0, 0.0, 50.0)
    private val viewPos = Coordinate3D(0.0, 0.0, 0.0)
    private val upVector = Vect3D(0.0, 1.0, 0.0)

    private val perspective = Perspective(45.0, 1.3, 1.0, 1000.0)

    private var theta = 45.0
    private var phi = 0.0

    private val mouseListener: MouseListener
    private val mouseMotionListener: MouseMotionListener
    private val mouseWheelListener: MouseWheelListener

    init {
        // Create GL context thread looper
        gl.context.makeCurrent()

        // Old mouse coordinate and world coordinate of preMouse
        val preMouse = Vec2i(0, 0)
        val preWorld = Coordinate3D()

        mouseListener = object : MouseListener {
            override fun mouseClicked(e: MouseEvent?) {}
            override fun mousePressed(e: MouseEvent?) {
                if (e != null) {
                    preMouse.setX(e.x)
                    preMouse.setY(e.y)
                    unProjectScreenCoordinate(gl, e.x, e.y, camPos)?.let { preWorld.set(it) }
                }
            }

            override fun mouseReleased(e: MouseEvent?) {}
            override fun mouseEntered(e: MouseEvent?) {}
            override fun mouseExited(e: MouseEvent?) {}
        }

        mouseMotionListener = object : MouseMotionListener {
            override fun mouseDragged(e: MouseEvent) {
                if (SwingUtilities.isLeftMouseButton(e)) {
                    moveCameraWithViewFocus(preWorld, e.x, e.y)
                } else if (SwingUtilities.isRightMouseButton(e)) {
                    calculateRotation(preMouse, e.x, e.y)
                }
/*                if (e!!.modifiersEx and InputEvent.BUTTON1_DOWN_MASK != 0 && e.modifiersEx and
                    InputEvent.BUTTON3_DOWN_MASK != 0) {
                    // When mouse's left and right button clicked at the same time.
                    calculateRotation(preMouse, e.x, e.y)

                } else if (SwingUtilities.isLeftMouseButton(e)){
                    // When just mouse's left button clicked.
                    moveCameraWithViewFocus(preWorld, e.x, e.y)
                }*/

                preMouse.set(e.x, e.y)
            }

            override fun mouseMoved(e: MouseEvent?) {}
        }

        mouseWheelListener = MouseWheelListener { e ->
            val pz = camPos.z + e.wheelRotation

            if (pz > 1) {
                camPos.z = pz
            }

            calculateRotation(preMouse, preMouse.x(), preMouse.y())

            update()
        }

        calculateRotation(Vec2i(0, 0), 0, 0)

        update()
    }

    fun update() {
        gl.glMatrixMode(GL2.GL_PROJECTION)
        gl.glLoadIdentity()

        glu.gluPerspective(perspective.fovy, perspective.aspect, perspective.zNear, perspective.zFar)

        glu.gluLookAt(
            camPos.x, camPos.y, camPos.z,
            viewPos.x, viewPos.y, viewPos.z,
            upVector.x, upVector.y, upVector.z
        )

        gl.glMatrixMode(GL2.GL_MODELVIEW)
        gl.glLoadIdentity()
    }

    private fun moveCameraWithViewFocus(preWorldCoord: Coordinate3D, x: Int, y: Int) {
        val worldCoordinate = unProjectScreenCoordinate(gl, x, y, camPos)

        val difX = worldCoordinate?.x?.minus(preWorldCoord.x)
        val difY = worldCoordinate?.y?.minus(preWorldCoord.y)

        if (difX != null) {
            camPos.x -= difX * 0.2
            viewPos.x -= difX * 0.2
        }

        if (difY != null) {
            camPos.y -= difY * 0.2
            viewPos.y -= difY * 0.2
        }

        update()
    }

    private fun calculateRotation(preMouse: Vec2i, mx: Int, my: Int) {
        val difX = mx - preMouse.x()
        val difY = my - preMouse.y()

        val horizontal = phi + difX * 0.2
        val vertical = theta + difY * 0.2

        if (horizontal in -180.0 .. 180.0) {
            phi = horizontal
        }

        if (vertical in 0.0 .. 90.0) {
            theta = vertical
        }

        val distance = camPos.distanceFromOther(viewPos)

        val theta = Math.toRadians(theta)
        val phi = Math.toRadians(phi) - Math.PI * 0.5

        val x = distance * sin(theta) * cos(phi) + viewPos.x
        val y = distance * sin(theta) * sin(phi) + viewPos.y
        val z = distance * cos(theta) + viewPos.z

        camPos.set(x, y, z)

        val vector = Vect3D((viewPos.x - x), (viewPos.y - y), (viewPos.z + z))

        if (vector.x != 0.0 && vector.y != 0.0) {
            upVector.set(vector)
        }

        update()
    }

    fun getMouseListener(): MouseListener {
        return mouseListener
    }

    fun getMouseWheelListener(): MouseWheelListener {
        return mouseWheelListener
    }

    fun getMouseMotionListener(): MouseMotionListener {
        return mouseMotionListener
    }

    fun setSize(width: Int, height: Int) {
        perspective.aspect = width.toDouble() / height.toDouble()
        update()
    }
}
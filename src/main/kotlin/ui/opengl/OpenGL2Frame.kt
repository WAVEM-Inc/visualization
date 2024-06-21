package ui.opengl

import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacle
import application.type.data.ObjectInfo
import application.type.data.Position
import application.type.option.ObstacleOption
import application.type.option.PointCloudOption
import com.jogamp.opengl.GL2
import com.jogamp.opengl.GLAutoDrawable
import com.jogamp.opengl.GLEventListener
import com.jogamp.opengl.awt.GLJPanel
import com.jogamp.opengl.glu.GLU
import com.jogamp.opengl.util.awt.TextRenderer
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import org.jetbrains.skiko.currentNanoTime
import ui.opengl.camera.SphereCamera
import ui.opengl.camera.getScreenCoords
import ui.opengl.camera.worldToScreen
import ui.opengl.drawer.GridLineDrawer
import ui.opengl.drawer.LineDrawer
import ui.opengl.drawer.PointsDrawer
import ui.opengl.drawer.PolyLineDrawer
import ui.opengl.math.Coordinate3D
import ui.opengl.math.GLColor
import ui.opengl.math.Polygon
import ui.theme.colorTohex
import ui.theme.hexToColor
import utils.pointCloudToCoordinate3DList
import viewmodel.ConfigDataViewModel
import viewmodel.UdpDataViewModel
import java.awt.Font
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class OpenGL2Frame(val parentPanel: GLJPanel) : GLEventListener {
    private lateinit var glu: GLU
    private lateinit var camera: SphereCamera

    private var pointCloudData: List<Coordinate3D> = ArrayList()
    private var pointCloudOption = PointCloudOption()

    private var objectsData: MutableMap<Int, Polygon> = HashMap()
    private var objectsInfo: MutableMap<Int, ObjectInfo> = HashMap()
    private var obstacleOption = ObstacleOption()

    private var currentLocale = Coordinate3D()
    private var currentHeading: Double = 0.0

    private val backgroundColor = GLColor("#01305A")
    private lateinit var textRenderer: TextRenderer

    val jobs = ArrayList<Job>()
    var fontSize = 24

    override fun init(drawable: GLAutoDrawable) {
        val gl = drawable.gl.gL2

        glu = GLU.createGLU(gl)
        camera = SphereCamera(gl, glu)
        textRenderer = TextRenderer(Font("SansSerif", Font.PLAIN, fontSize))

        parentPanel.addMouseListener(camera.getMouseListener())
        parentPanel.addMouseWheelListener(camera.getMouseWheelListener())
        parentPanel.addMouseMotionListener(camera.getMouseMotionListener())

        startJobs(CoroutineScope(Dispatchers.Main))
    }

    override fun dispose(drawable: GLAutoDrawable) {
        println("GLFrame is disposed.")
    }

    override fun display(drawable: GLAutoDrawable) {
        val gl: GL2 = drawable.gl.gL2

        gl.glClearColor(backgroundColor.red, backgroundColor.green, backgroundColor.blue, 1f)
        gl.glClear(GL2.GL_COLOR_BUFFER_BIT or GL2.GL_DEPTH_BUFFER_BIT)

        if (textRenderer.font.size != fontSize) {
            if (textRenderer != null) {
                textRenderer.dispose()
            }

            textRenderer = TextRenderer(Font("SansSerif", Font.PLAIN, fontSize))
        }

        // Draw Grid Lines
        val gridLineDrawer =
            GridLineDrawer(width = 100, height = 100, lineColor = GLColor("#FFFFFF"))
        gridLineDrawer.draw(gl)

        textRenderer.setColor(1f, 1f, 1f, 1f)
        for (i in -50..50 step 5) {
            drawTextBillboard(gl, glu, textRenderer, "${abs(i)}m", i.toFloat(), 0f, 0f)
            drawTextBillboard(gl, glu, textRenderer, "${abs(i)}m", 0f, i.toFloat(), 0f)
        }

        // Draw 3-axis lines
        val xLineDrawer = LineDrawer(endPoint = Coordinate3D(x = 1.0), color = GLColor(red = 1f), lineWidth = 2f)
        val yLineDrawer = LineDrawer(endPoint = Coordinate3D(y = 1.0), color = GLColor(green = 1f), lineWidth = 2f)
        val zLineDrawer = LineDrawer(endPoint = Coordinate3D(z = 1.0), color = GLColor(blue = 1f), lineWidth = 2f)

        xLineDrawer.draw(gl)
        yLineDrawer.draw(gl)
        zLineDrawer.draw(gl)

        val heading = Math.toDegrees(currentHeading).toFloat()
        gl.glRotatef(-(heading - 90), 0f, 0f, 1f)
        if (pointCloudOption.draw) {
            val pointsDrawer =
                if (pointCloudOption.useDynamicColor) {
                    PointsDrawer(
                        pointSize = 3f,
                        points = pointCloudData,
                        nearColor = hexToColor(pointCloudOption.nearColor),
                        farColor = hexToColor(pointCloudOption.farColor),
                        maxDistance = pointCloudOption.maxDistance,
                        heading = currentHeading
                    )
                } else {
                    PointsDrawer(
                        pointSize = 3f,
                        points = pointCloudData,
                        nearColor = hexToColor(pointCloudOption.farColor),
                        farColor = hexToColor(pointCloudOption.farColor),
                        maxDistance = pointCloudOption.maxDistance,
                        heading = currentHeading
                    )
                }

            pointsDrawer.draw(gl)
        }

        val color = hexToColor(obstacleOption.textColor)
        textRenderer.setColor(color.red, color.green, color.blue, color.alpha)
        if (obstacleOption.draw) {
            val polyLineDrawer =
                PolyLineDrawer(
                    objects = objectsData,
                    objectsInfo = objectsInfo,
                    textRenderer = textRenderer,
                    obstacleOption = obstacleOption,
                    glu = glu,
                    parentPanel = parentPanel
                )
            polyLineDrawer.draw(gl)
        }
        gl.glRotatef(heading - 90, 0f, 0f, 1f)
    }

    override fun reshape(drawable: GLAutoDrawable, x: Int, y: Int, width: Int, height: Int) {
        camera.setSize(width, height)
    }

    fun updatePointCloudData(pclData: List<Coordinate3D>) {
        pointCloudData = pclData
//        println("pclData = [${pclData}]")
    }

    private fun getPolygonFromObstacle(obstacle: PerceptionObstacle): Polygon {
        val polygon = Polygon()

        polygon.width = obstacle.width
        polygon.height = obstacle.height

//        val cosTheta = cos(currentHeading)
//        val sinTheta = sin(currentHeading)

        var x = obstacle.position.x - currentLocale.x
        var y = obstacle.position.y - currentLocale.y
        var z = obstacle.position.z - currentLocale.z
//
//        x = x * cosTheta - y * sinTheta
//        y = x * sinTheta + y * cosTheta

        polygon.position = Coordinate3D(
            x = x,
            y = y,
            z = z
        )

        obstacle.polygonPointList.forEach { point ->
            var px = point.x - currentLocale.x
            var py = point.y - currentLocale.y
            var pz = point.z - currentLocale.z
//
//            px = px * cosTheta - py * sinTheta
//            py = px * sinTheta + py * cosTheta
//            px = x + (px - x) * cosTheta - (py - y) * sinTheta
//            py = y + (px - x) * sinTheta + (py - y) * cosTheta

            polygon.points.add(
                Coordinate3D(
                    x = px,
                    y = py,
                    z = pz
                )
            )
        }

        return polygon
    }

    private fun updateObjectInfo(obstacle: PerceptionObstacle) {
        val id = obstacle.id

        if (objectsInfo.containsKey(id).not()) {
            val speed = sqrt(obstacle.velocity.x * obstacle.velocity.x +
                        obstacle.velocity.y * obstacle.velocity.x +
                        obstacle.velocity.z * obstacle.velocity.z) / 1000

            val info = ObjectInfo(
                id = obstacle.id,
                type = obstacle.type.name,
                heading = obstacle.theta,
                position = Position(obstacle.position.x, obstacle.position.y, obstacle.position.z),
                speed = speed,
                ttc = obstacle.ttc.toString(),
                risk = obstacle.riskLevel.toString(),
                textDrawPosition = Coordinate3D(
                    x = obstacle.position.x - currentLocale.x + obstacle.width,
                    y = obstacle.position.y - currentLocale.y + obstacle.height,
                    z = obstacle.height + obstacle.position.z
                )
            )

            objectsInfo[id] = info
        } else {
            val value = objectsInfo[id]

            if (value != null) {
                value.id = obstacle.id
                value.type = obstacle.type.name
                value.heading = obstacle.theta
                value.position = Position(obstacle.position.x, obstacle.position.y, obstacle.position.z)
                value.speed = obstacle.velocity.x
                value.ttc = obstacle.ttc.toString()
                value.risk = obstacle.riskLevel.toString()
                value.textDrawPosition = Coordinate3D(
                    x = obstacle.position.x - currentLocale.x + obstacle.width,
                    y = obstacle.position.y - currentLocale.y + obstacle.height,
                    z = obstacle.height + obstacle.position.z
                )
            }
        }
    }

    fun startJobs(coroutineScope: CoroutineScope) {
        jobs.add(coroutineScope.launch(context = Dispatchers.Main) {
            UdpDataViewModel.subscribePointCloud(collector = { pointCloud ->
                pointCloudData = pointCloudToCoordinate3DList(pointCloud)
            })
        })

        jobs.add(coroutineScope.launch(context = Dispatchers.Main) {
            UdpDataViewModel.subscribeLocalization(collector = { localization ->
                val position = localization.pose.position
                currentLocale = Coordinate3D(position.x, position.y, position.z)
                currentHeading = localization.pose.heading
            })
        })

        jobs.add(coroutineScope.launch(context = Dispatchers.Main) {
            UdpDataViewModel.subscribeObstacles(collector = { obstacles ->
                val objects = HashMap<Int, Polygon>()

                for (obs in obstacles.perceptionObstacleList) {
                    val polygon = getPolygonFromObstacle(obs)
                    objects[obs.id] = polygon

                    updateObjectInfo(obs)
                }

                objectsData = objects
                objectsInfo.keys.retainAll(objectsData.keys)
            })
        })

        jobs.add(coroutineScope.launch(context = Dispatchers.Main) {
            ConfigDataViewModel.subscribePointCloudOption { option ->
                pointCloudOption = option
            }
        })

        jobs.add(coroutineScope.launch(context = Dispatchers.Main) {
            ConfigDataViewModel.subscribeObstacleOption { option ->
                obstacleOption = option
                fontSize = option.textSize
            }
        })
    }

    fun stopJobs() {
        for (job in jobs) {
            job.cancel()
        }
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

    fun drawTextBillboard(gl: GL2, glu: GLU, textRenderer: TextRenderer, text: String, x: Float, y: Float, z: Float) {
        val screenPos = projectToScreen(gl, glu, x, y, z)
        if (screenPos != null) {
            val screenX = screenPos[0]
            val screenY = screenPos[1] // OpenGL's Y axis is inverted

            textRenderer.beginRendering(parentPanel.width, parentPanel.height)
            textRenderer.setColor(1f, 1f, 1f, 1f)
            textRenderer.draw(text, screenX.toInt(), screenY.toInt())
            textRenderer.endRendering()
        }
    }
}
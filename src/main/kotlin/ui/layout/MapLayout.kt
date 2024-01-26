package ui.layout

import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.awt.SwingPanel
import androidx.compose.ui.graphics.Color
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import org.jxmapviewer.JXMapViewer
import org.jxmapviewer.OSMTileFactoryInfo
import org.jxmapviewer.input.PanMouseInputListener
import org.jxmapviewer.input.ZoomMouseWheelListenerCenter
import org.jxmapviewer.viewer.DefaultTileFactory
import org.jxmapviewer.viewer.GeoPosition
import ui.map.LocationPainter
import ui.map.MyButton
import util.files.loadResource
import util.proj.ProjUtil
import viewmodel.TestViewModel
import java.awt.FlowLayout
import java.awt.Image
import javax.imageio.ImageIO
import javax.swing.ImageIcon

@Composable
fun MapLayout() {
    val viewer = JXMapViewer()
    val locationPainter = LocationPainter()

    val coroutineScope = rememberCoroutineScope()
    var subscribeJob by remember { mutableStateOf<Job?>(null) }


    var draggable by remember { mutableStateOf(false) }

    val btnDraggable = MyButton()

    LaunchedEffect(Unit) {
        subscribeJob = coroutineScope.launch {
            TestViewModel.subscribeLocalization(collector = { location ->
                if (draggable.not()) {
                    val geoPosition = ProjUtil.utmToWgs84(
                        location.pose.position.x,
                        location.pose.position.y,
                        location.pose.position.z
                    )

                    viewer.addressLocation = geoPosition
                    locationPainter.setGeoPosition(geoPosition)
                    locationPainter.setHeading(location.pose.heading)
                }
            })
        }
    }

    DisposableEffect(Unit) {
        onDispose {
            subscribeJob?.cancel()
        }
    }

    LaunchedEffect(Unit) {
        viewer.tileFactory = getTileFactory()
        viewer.addressLocation = GeoPosition(0.0, 0.0)
        viewer.zoom = 3
        viewer.setPanEnabled(draggable)

        locationPainter.setGeoPosition(GeoPosition(0.0, 0.0))
        viewer.setOverlayPainter(locationPainter)

        val mm = PanMouseInputListener(viewer)
        viewer.addMouseListener(mm)
        viewer.addMouseMotionListener(mm)
        viewer.addMouseWheelListener(ZoomMouseWheelListenerCenter(viewer))
        viewer.addMouseWheelListener { e ->
            if (e.wheelRotation == -1 && viewer.zoom != 0) {
                viewer.addressLocation = viewer.convertPointToGeoPosition(e.point)
            }
        }
    }

    SwingPanel(background = Color.White, modifier = Modifier.fillMaxSize(), factory = {
        viewer.apply {
            layout = FlowLayout(FlowLayout.RIGHT)

            val icon = ImageIcon(
                ImageIO.read(loadResource("icon/ic_move.png"))
                    .getScaledInstance(30, 30, Image.SCALE_SMOOTH)
            )

            btnDraggable.layout = null
            btnDraggable.icon = icon
            btnDraggable.addActionListener {
                draggable = draggable.not()
            }

            add(btnDraggable)
        }
    }, update = {
        btnDraggable.isSelected = draggable
        viewer.setPanEnabled(draggable)
    })
}

private fun getTileFactory(): DefaultTileFactory {
    val info = OSMTileFactoryInfo()

    return DefaultTileFactory(info)
}
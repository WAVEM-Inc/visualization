import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.DpSize
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.*
import apollo.dreamview.PointCloudOuterClass.PointCloud
import application.EvizConfigManager
import application.EvizWindow
import application.EvizWindowManager
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import ui.component.title.EvizTitleBar
import ui.layout.MainLayout
import ui.opengl.math.Coordinate3D
import ui.theme.Navy_100
import util.protobuf.ReplayData
import util.protobuf.readFileBuffers
import viewmodel.TestViewModel
import java.awt.Dimension


@Composable
fun App() {
    val replayData = loadFile()

    val pclList = ArrayList<ArrayList<Coordinate3D>>()
    for (pcl in replayData.pointCloud) {
        pclList.add(pointCloudToList(pcl))
    }

    val delay = 1000f / 33f
    CoroutineScope(Dispatchers.Default).launch {
        var count = 0
        while (true) {
            if (count >= pclList.size - 1) {
                count = 0
            }
            delay(delay.toLong())

            TestViewModel.updatePointCloud(pclList[count])
            TestViewModel.updateObstacle(replayData.obstacles[count].perceptionObstacleList)
            TestViewModel.updateLocalization(replayData.localization[count])

            count++
        }
    }

    CoroutineScope(Dispatchers.Default).launch {
        TestViewModel.subscribeObstacle(collector = { obstacle ->
            val dummys = HashMap<Int, List<String>>()

            for (o in obstacle) {
                dummys[o.id] = listOf("Test1", "Test2")
            }

            TestViewModel.updateDummy(dummys)
        })
    }

    EvizConfigManager

    MaterialTheme {
        MainLayout(modifier = Modifier.fillMaxSize().background(Navy_100))
    }
}

fun main() = application {
    val applicationState = remember { EvizWindowManager.windows }

    val mainWindowState = rememberWindowState(
        position = WindowPosition(Alignment.Center),
        size = DpSize(1920.dp, 980.dp)
    )

    Window(
        onCloseRequest = { exitApplication() },
        title = "Eviz - Essys Visualization",
        state = mainWindowState
    ) {
        window.minimumSize = Dimension(1250, 600)
        Column(modifier = Modifier.fillMaxSize()) {
            // Title Bar
            EvizTitleBar(
                true,
                "Eviz - Essys Visualization",
                onExitClick = { exitApplication() },
                onMaximizeClick = {
                    when (mainWindowState.placement) {
                        WindowPlacement.Maximized -> mainWindowState.placement = WindowPlacement.Floating
                        WindowPlacement.Floating -> mainWindowState.placement = WindowPlacement.Maximized
                        WindowPlacement.Fullscreen -> mainWindowState.placement = WindowPlacement.Floating
                    }
                },
                onMinimizeClick = { mainWindowState.isMinimized = true }
            )

            // Main Contents
            App()
        }

        // Create New Windows
        for (window in applicationState.entries) {
            if (window.value) {
                EvizWindow(type = window.key, useMenu = false)
            }
        }
    }
}

fun loadFile(): ReplayData {
    val replayData = readFileBuffers()
    val pclData = replayData.pointCloud

    val result = ArrayList<ArrayList<Coordinate3D>>()
    for (pcl in pclData) {
        result.add(pointCloudToList(pcl))
    }

    return replayData
}

fun pointCloudToList(pcl: PointCloud): ArrayList<Coordinate3D> {
    val result = ArrayList<Coordinate3D>()
    for (i in 0 until pcl.numList.size step 3) {
        val x = pcl.numList[i]
        val y = pcl.numList[i + 1]
        val z = pcl.numList[i + 2]
        result.add(Coordinate3D(x.toDouble(), y.toDouble(), z.toDouble()))
    }

    return result
}
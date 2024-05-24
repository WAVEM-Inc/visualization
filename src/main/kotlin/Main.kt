import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.DpSize
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.*
import apollo.dreamview.PointCloudOuterClass
import apollo.localization.Localization
import apollo.perception.PerceptionObstacleOuterClass
import application.EvizWindow
import application.connect.UdpClient
import application.manager.EvizWindowManager
import application.type.EvizConfigManager
import essys_middle.Dashboard
import essys_middle.Mobileye
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import ui.component.setting.lidar.LidarOption
import ui.component.setting.mobileye.MobileyeOptionCache
import ui.component.setting.obstacle.ObstacleOptionCache
import ui.component.title.EvizTitleBar
import ui.layout.MainLayout
import ui.theme.Navy_100
import utils.getDefaultCachePath
import viewmodel.ConfigDataViewModel
import viewmodel.UdpDataViewModel
import java.awt.Dimension

@Composable
@Preview
fun App() {
    LaunchedEffect(Unit) {
        println(getDefaultCachePath())
        EvizConfigManager
        LidarOption
        ObstacleOptionCache
        MobileyeOptionCache

        val client = UdpClient()
        val cs = CoroutineScope(Dispatchers.Main)

        client.setOnMessageListener(listener = object: UdpClient.OnMessageListener {
            override fun onLocalizationReceive(message: Localization.LocalizationEstimate) {
                cs.launch {
                    UdpDataViewModel.updateLocalization(message)
                }
            }

            override fun onObstacleReceive(message: PerceptionObstacleOuterClass.PerceptionObstacles) {
                cs.launch {
                    UdpDataViewModel.updateObstacles(message)
                }
            }

            override fun onTrafficLightReceive(message: Dashboard.TrafficLight) {
                cs.launch {
                    UdpDataViewModel.updateTrafficLight(message)
                }
            }

            override fun onPointCloudReceive(message: PointCloudOuterClass.PointCloud) {
                cs.launch {
                    UdpDataViewModel.updatePointCloud(message)
                }
            }

            override fun onDashboardReceive(message: Dashboard.VehicleSignal) {
                cs.launch {
                    UdpDataViewModel.updateVehicleSignal(message)
                }
            }

            override fun onMobileyeReceived(message: Mobileye.MobileyeData) {
                cs.launch {
                    UdpDataViewModel.updateMobileye(message)
                }
            }
        })

        ConfigDataViewModel.subscribeConnectOption(collector = { option ->
            println("Connect Info - ipAddr: ${option.ipAddress}, port: ${option.receivePort}")
            client.connect(option.ipAddress, option.receivePort)
        })
    }

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

package ui.layout

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.delay
import ui.component.common.EvizVerticalResizableLayout
import ui.component.monitor.MonitorMode
import ui.component.monitor.MonitorModeButton
import ui.component.monitor.MonitorTextField
import ui.theme.Gray
import ui.theme.White_100
import ui.theme.White_200
import viewmodel.UdpDataViewModel
import java.time.Instant
import java.time.LocalDateTime
import java.time.ZoneId
import java.time.format.DateTimeFormatter

@Preview
@Composable
fun MonitorLayout(
    modifier: Modifier = Modifier.fillMaxSize()
) {
    var monitorMode by remember { mutableStateOf(MonitorMode.VERTICAL) }

    Column(modifier = modifier) {
        MonitorModeButton(
            modifier = Modifier
                .wrapContentSize()
                .align(Alignment.CenterHorizontally)
                .padding(top = 16.dp)
                .clip(RoundedCornerShape(10)),
            mode = monitorMode,
            onModeChange = { monitorMode = it }
        )

        if (monitorMode == MonitorMode.VERTICAL) {
            VerticalMonitorLayout()
        } else {
            HorizontalMonitorLayout()
        }
    }
}

@Composable
private fun VerticalMonitorLayout(
    modifier: Modifier = Modifier.fillMaxSize()
) {
    val scrollState = rememberScrollState()
    val scrollAdapter = rememberScrollbarAdapter(scrollState)

    val gpsContents = remember { mutableStateListOf<String>() }
    val lidarContents = remember { mutableStateListOf<String>() }
    val cameraContents = remember { mutableStateListOf<String>() }

    LaunchedEffect(true) {
        UdpDataViewModel.subscribeLocalization(collector = { localization ->
            val current = LocalDateTime.now()
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd-HH:mm:ss")
            val formatted = current.format(formatter)

            gpsContents.add("${formatted}, Lat: ${localization.pose.position.x}, Lng: ${localization.pose.position.y}")
        })

        UdpDataViewModel.subscribePointCloud(collector = { points ->
            val current = LocalDateTime.now()
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd-HH:mm:ss")
            val formatted = current.format(formatter)

            lidarContents.add("${formatted}, Points Count: ${points.numCount}")
        })

        UdpDataViewModel.subscribeTrafficLight(collector = { lights ->
            val current = LocalDateTime.now()
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd-HH:mm:ss")
            val formatted = current.format(formatter)

            val red = if (lights.red.number == 0) "X" else "O"
            val yellow = if (lights.yellow.number == 0) "X" else "O"
            val left = if (lights.left.number == 0) "X" else "O"
            val green = if (lights.green.number == 0) "X" else "O"

            cameraContents.add("${formatted}, RED: ${red}, YELLOW: ${yellow}, LEFT: ${left}, GREEN: ${green}")
        })
    }

    Box(modifier = modifier) {
        Column(Modifier.fillMaxSize().verticalScroll(scrollState).padding(16.dp).padding(end = 32.dp)) {
            MonitorTextField(
                title = "GPS Data",
                info = "GPS UTC Time: 1641363045",
                infoList = listOf("RTK Status: RTK_FIXED", "INS Status: INS_GOOD", "SOL Status: SOL_GOOD"),
                contents = gpsContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )

            MonitorTextField(
                title = "LiDAR Data",
                info = "LiDAR PointCloud: 4000",
                infoList = listOf(""),
//                infoList = listOf("Total: 4000", "Front: 1000", "Left: 1500", "Right: 1500"),
                contents = lidarContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )

            MonitorTextField(
                title = "Camera Data",
                info = "Camera",
                infoList = listOf(""),
//                infoList = listOf("RED: O", "YELLOW: X", "LEFT: O", "GREEN: X"),
                contents = cameraContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )
        }

        VerticalScrollbar(
            adapter = scrollAdapter,
            modifier = Modifier.align(Alignment.TopEnd),
            style = ScrollbarStyle(
                minimalHeight = 20.dp,
                thickness = 8.dp,
                shape = RoundedCornerShape(50),
                hoverDurationMillis = 200,
                unhoverColor = White_100,
                hoverColor = Gray
            )
        )

    }
}

@Composable
private fun HorizontalMonitorLayout(
    modifier: Modifier = Modifier.fillMaxSize()
) {
    val scrollState = rememberScrollState()
    val scrollAdapter = rememberScrollbarAdapter(scrollState)

    val gpsContents = remember { mutableStateListOf<String>() }
    val lidarContents = remember { mutableStateListOf<String>() }
    val cameraContents = remember { mutableStateListOf<String>() }

    LaunchedEffect(true) {
        UdpDataViewModel.subscribeLocalization(collector = { localization ->
            val current = LocalDateTime.now()
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd-HH:mm:ss")
            val formatted = current.format(formatter)

            gpsContents.add("${formatted}, Lat: ${localization.pose.position.x}, Lng: ${localization.pose.position.y}")
        })

        UdpDataViewModel.subscribePointCloud(collector = { points ->
            val current = LocalDateTime.now()
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd-HH:mm:ss")
            val formatted = current.format(formatter)

            lidarContents.add("${formatted}, Points Count: ${points.numCount}")
        })

        UdpDataViewModel.subscribeTrafficLight(collector = { lights ->
            val current = LocalDateTime.now()
            val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd-HH:mm:ss")
            val formatted = current.format(formatter)

            val red = if (lights.red.number == 0) "X" else "O"
            val yellow = if (lights.yellow.number == 0) "X" else "O"
            val left = if (lights.left.number == 0) "X" else "O"
            val green = if (lights.green.number == 0) "X" else "O"

            cameraContents.add("${formatted}, RED: ${red}, YELLOW: ${yellow}, LEFT: ${left}, GREEN: ${green}")
        })
    }

    Box(modifier = modifier) {
        Row(Modifier.fillMaxSize().horizontalScroll(scrollState).padding(16.dp).padding(end = 32.dp)) {
            MonitorTextField(
                title = "GPS Data",
                info = "GPS UTC Time: 1641363045",
                infoList = listOf("RTK Status: RTK_FIXED", "INS Status: INS_GOOD", "SOL Status: SOL_GOOD"),
                contents = gpsContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )

            MonitorTextField(
                title = "LiDAR Data",
                info = "LiDAR PointCloud: 4000",
                infoList = listOf(""),
//                infoList = listOf("Total: 4000", "Front: 1000", "Left: 1500", "Right: 1500"),
                contents = lidarContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )

            MonitorTextField(
                title = "Camera Data",
                info = "Camera",
                infoList = listOf(""),
//                infoList = listOf("RED: O", "YELLOW: X", "LEFT: O", "GREEN: X"),
                contents = cameraContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )
        }

        VerticalScrollbar(
            adapter = scrollAdapter,
            modifier = Modifier.align(Alignment.TopEnd),
            style = ScrollbarStyle(
                minimalHeight = 20.dp,
                thickness = 8.dp,
                shape = RoundedCornerShape(50),
                hoverDurationMillis = 200,
                unhoverColor = White_100,
                hoverColor = Gray
            )
        )

    }
}
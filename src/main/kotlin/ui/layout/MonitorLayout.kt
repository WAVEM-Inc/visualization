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
        for (i in 1..1000) {
            delay(200)
            gpsContents.add("${i}. 2023-10-25 14:30:00.0, Lat: 37.7749, Lon: -122.4194")
            lidarContents.add("${i}. 2023-10-25 14:30:00.0, X: 1.2, Y: 3.4, Z: 5.6")
            cameraContents.add("${i}. 2023-10-25 14:30:00.0, X: 1.2, Y: 3.4, Z: 5.6")
        }
    }

    Box(modifier = modifier) {
        Column(Modifier.fillMaxSize().verticalScroll(scrollState).padding(16.dp).padding(end = 32.dp)) {
            MonitorTextField(
                title = "GPS Data",
                info = "GPS UTC Time: 1641363045",
                infoList = listOf("RTK Status: RTK_FIXED", "INS Status: INS_GOOD", "SOL Status: SOL_ GOOD"),
                contents = gpsContents,
                modifier = Modifier.fillMaxWidth().heightIn(50.dp, 600.dp).padding(bottom = 30.dp),
            )


            MonitorTextField(
                title = "LiDAR Data",
                info = "LiDAR PointCloud: 4000",
                infoList = listOf("Total: 4000", "Front: 1000", "Left: 1500", "Right: 1500"),
                contents = lidarContents,
                modifier = Modifier.fillMaxWidth().heightIn(50.dp, 600.dp).padding(bottom = 30.dp)
            )

            MonitorTextField(
                title = "Camera Data",
                info = "Camera",
                infoList = listOf("RED: O", "YELLOW: X", "LEFT: O", "GREEN: X"),
                contents = cameraContents,
                modifier = Modifier.fillMaxWidth().heightIn(50.dp, 600.dp).padding(bottom = 30.dp)
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
        for (i in 1..1000) {
            delay(200)
            gpsContents.add("${i}. 2023-10-25 14:30:00.0, Lat: 37.7749, Lon: -122.4194")
            lidarContents.add("${i}. 2023-10-25 14:30:00.0, X: 1.2, Y: 3.4, Z: 5.6")
            cameraContents.add("${i}. 2023-10-25 14:30:00.0, X: 1.2, Y: 3.4, Z: 5.6")
        }
    }

    Box(modifier = modifier) {
        Row(Modifier.fillMaxSize().horizontalScroll(scrollState).padding(16.dp).padding(end = 32.dp)) {
            MonitorTextField(
                title = "GPS Data",
                info = "GPS UTC Time: 1641363045",
                infoList = listOf("RTK Status: RTK_FIXED", "INS Status: INS_GOOD", "SOL Status: SOL_ GOOD"),
                contents = gpsContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )

            MonitorTextField(
                title = "LiDAR Data",
                info = "LiDAR PointCloud: 4000",
                infoList = listOf("Total: 4000", "Front: 1000", "Left: 1500", "Right: 1500"),
                contents = lidarContents,
                modifier = Modifier.fillMaxHeight().weight(1f).padding(horizontal = 16.dp)
            )

            MonitorTextField(
                title = "Camera Data",
                info = "Camera",
                infoList = listOf("RED: O", "YELLOW: X", "LEFT: O", "GREEN: X"),
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
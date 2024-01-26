package ui.layout

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.unit.IntSize
import androidx.compose.ui.unit.dp
import ui.component.gl.PointCloudViewer
import ui.component.vehicle.TrafficLightInfo
import ui.component.vehicle.VehicleSignalInfo
import ui.theme.Navy_100

@Preview
@Composable
fun DashboardLayout(modifier: Modifier = Modifier.fillMaxSize()) {
    val parentSize = remember { mutableStateOf(IntSize(0, 0)) }
    val vehicleInfoHeight = 220

    Column(modifier = modifier.fillMaxSize().background(Navy_100).onSizeChanged { size ->
        parentSize.value = size
    }) {
        PointCloudViewer(
            modifier = Modifier
                .align(Alignment.CenterHorizontally)
                .offset(0.dp, 2.dp)
                .width((parentSize.value.width - 8).dp)
                .height((parentSize.value.height - vehicleInfoHeight).dp)
        )

        Row(modifier = Modifier.height(vehicleInfoHeight.dp)) {
            VehicleSignalInfo(
                modifier = Modifier
                    .fillMaxHeight()
                    .fillMaxWidth(0.7f)
                    .padding(4.dp)
                    .padding(top = 2.dp)
            )
            TrafficLightInfo(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(vertical = 4.dp)
                    .padding(top = 2.dp, end = 4.dp)
            )
        }
    }
}
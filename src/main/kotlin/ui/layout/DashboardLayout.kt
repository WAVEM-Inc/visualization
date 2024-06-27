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
import ui.theme.Navy_200

@Preview
@Composable
fun DashboardLayout(modifier: Modifier = Modifier.fillMaxSize()) {

    Column(modifier = modifier.fillMaxSize().background(Navy_100).padding(4.dp)) {
        PointCloudViewer(
            modifier = Modifier
                .align(Alignment.CenterHorizontally)
                .fillMaxWidth()
                .weight(1f)
                .padding(bottom = 4.dp)
        )

        Row(modifier = Modifier.wrapContentHeight().background(Navy_200)) {
            VehicleSignalInfo(
                modifier = Modifier
                    .fillMaxWidth(0.7f)
            )
            TrafficLightInfo(
                modifier = Modifier
                    .fillMaxWidth()
            )
        }
    }
}
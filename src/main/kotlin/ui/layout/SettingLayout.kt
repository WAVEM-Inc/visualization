package ui.layout

import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import ui.component.common.EvizText
import ui.component.setting.lidar.LidarSettingLayout
import ui.component.setting.mobileye.MobileyeOptionLayout
import ui.component.setting.obstacle.ObstacleOptionLayout
import ui.theme.Gray
import ui.theme.Navy_100
import ui.theme.White_100

@Composable
fun SettingLayout(modifier: Modifier = Modifier.fillMaxSize()) {
    val vsState = rememberScrollState()

    val vsAdapter = rememberScrollbarAdapter(vsState)

    Box(modifier = modifier.background(Navy_100)) {
        Column(
            modifier = Modifier
                .verticalScroll(vsState)
                .padding(horizontal = 16.dp)
                .fillMaxWidth()
        ) {
            Spacer(Modifier.height(50.dp).fillMaxWidth())
            EvizText("LiDAR", fontWeight = FontWeight.Bold, fontSize = 30f)
            LidarSettingLayout()
            Spacer(Modifier.height(50.dp).fillMaxWidth())

            EvizText("Obstacle", fontWeight = FontWeight.Bold, fontSize = 30f)
            ObstacleOptionLayout()
            Spacer(Modifier.height(50.dp))

            EvizText("Mobileye", fontWeight = FontWeight.Bold, fontSize = 30f)
            MobileyeOptionLayout()
            Spacer(Modifier.height(50.dp))
        }

        VerticalScrollbar(
            modifier = Modifier.align(Alignment.TopEnd),
            adapter = vsAdapter,
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
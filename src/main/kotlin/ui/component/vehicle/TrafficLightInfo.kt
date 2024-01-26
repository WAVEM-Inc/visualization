package ui.component.vehicle

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.painter.Painter
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import ui.theme.Navy_100
import ui.theme.Navy_200
import ui.theme.Poppins
import ui.theme.White_100

@Preview
@Composable
fun TrafficLightInfo(modifier: Modifier = Modifier) {
    Column(modifier.background(Navy_200)) {
        Text(
            modifier = Modifier.padding(8.dp),
            text = "Traffic Light Information",
            fontFamily = Poppins,
            fontWeight = FontWeight.Bold,
            color = White_100,
            fontSize = TextUnit(24f, TextUnitType.Sp)
        )

        Box(
            modifier = Modifier.fillMaxSize(),
            contentAlignment = Alignment.Center
        ) {
            Row(
                modifier = Modifier
                    .clip(RoundedCornerShape(24))
                    .fillMaxWidth(0.8f)
                    .aspectRatio(3.6f)
                    .background(Navy_100)
                    .align(Alignment.Center)
            ) {
                val lightModifier = Modifier.fillMaxSize().weight(1f).padding(10.dp)

                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_red.svg"))
                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_yellow.svg"))
                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_arrow.svg"), isOn = true)
                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_green.svg"))
            }
        }
    }
}

@Composable
private fun TrafficLight(painter: Painter, modifier: Modifier = Modifier, isOn: Boolean = false) {
    Image(
        modifier = modifier.alpha(if (isOn) 1f else 0.5f),
        painter = painter,
        contentDescription = null,
        contentScale = ContentScale.Fit
    )
}
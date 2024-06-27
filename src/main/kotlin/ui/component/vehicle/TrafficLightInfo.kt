package ui.component.vehicle

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
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
import viewmodel.UdpDataViewModel

@Preview
@Composable
fun TrafficLightInfo(modifier: Modifier = Modifier) {
    var redOn = remember { mutableStateOf(false) }
    var yellowOn = remember { mutableStateOf(false) }
    var arrowOn = remember { mutableStateOf(false) }
    var greenOn = remember { mutableStateOf(false) }

    LaunchedEffect(Unit) {
        UdpDataViewModel.subscribeTrafficLight(collector = { info ->
            redOn.value = info.red.number == 1
            yellowOn.value = info.yellow.number == 1
            arrowOn.value = info.left.number == 1
            greenOn.value = info.green.number == 1
        })
    }

    Column(modifier = modifier.background(Navy_200)) {
        Text(
            modifier = Modifier.padding(8.dp),
            text = "Traffic Light Information",
            fontFamily = Poppins,
            fontWeight = FontWeight.Bold,
            color = White_100,
            fontSize = TextUnit(24f, TextUnitType.Sp)
        )

        Box(
            modifier = Modifier.fillMaxWidth().align(Alignment.CenterHorizontally),
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

                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_red.svg"), isOn = redOn.value)
                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_yellow.svg"), isOn = yellowOn.value)
                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_arrow.svg"), isOn = arrowOn.value)
                TrafficLight(modifier = lightModifier, painter = painterResource("icon/ic_traffic_light_green.svg"), isOn = greenOn.value)
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
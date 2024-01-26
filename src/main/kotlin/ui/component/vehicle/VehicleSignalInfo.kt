package ui.component.vehicle

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import ui.component.common.TextWithHeaderContent
import ui.theme.Navy_200
import ui.theme.Poppins
import ui.theme.White_100

@Preview
@Composable
fun VehicleSignalInfo(modifier: Modifier = Modifier) {
    Column(modifier = modifier.background(Navy_200)) {
        Text(
            modifier = Modifier.padding(start = 8.dp, top = 8.dp),
            text = "Vehicle Signal Infomation",
            fontFamily = Poppins,
            fontWeight = FontWeight.Bold,
            fontSize = TextUnit(22f, TextUnitType.Sp),
            color = White_100
        )

        Row {
            VehicleSignalInfoItem1(modifier = Modifier.weight(1f).padding(16.dp))
            VehicleSignalInfoItem2(modifier = Modifier.weight(1f).padding(16.dp))
            VehicleSignalInfoItem3(modifier = Modifier.weight(1f).padding(16.dp))
        }
    }
}

@Composable
private fun VehicleSignalInfoItem1(modifier: Modifier = Modifier) {
    val textModifier = Modifier.padding(vertical = 4.dp)

    Column(modifier = modifier) {
        TextWithHeaderContent(text = "Gear Position", modifier = textModifier)
        TextWithHeaderContent(text = "Yaw Rate", modifier = textModifier)
        TextWithHeaderContent(text = "Vehicle Speed", modifier = textModifier)
        TextWithHeaderContent(text = "Vehicle Accelerator", modifier = textModifier)
        TextWithHeaderContent(text = "Steering Angle", modifier = textModifier)
    }
}

@Composable
private fun VehicleSignalInfoItem2(modifier: Modifier = Modifier) {
    val textModifier = Modifier.padding(vertical = 4.dp)

    Column(modifier = modifier) {
        TextWithHeaderContent(text = "Steering Rotation Speed", modifier = textModifier)
        TextWithHeaderContent(text = "Brake Throttle Input", modifier = textModifier)
        TextWithHeaderContent(text = "Brake Throttle Output", modifier = textModifier)
        TextWithHeaderContent(text = "Turn Signal", modifier = textModifier)
        TextWithHeaderContent(text = "Head Light", modifier = textModifier)
    }
}

@Composable
private fun VehicleSignalInfoItem3(modifier: Modifier = Modifier) {
    val textModifier = Modifier.padding(vertical = 4.dp)

    Column(modifier = modifier) {
        TextWithHeaderContent(text = "Wheel Speed", modifier = textModifier)
        TextWithHeaderContent(text = "Hazard Signal", modifier = textModifier)
        TextWithHeaderContent(text = "Door Lock", modifier = textModifier)
        TextWithHeaderContent(text = "Door Open", modifier = textModifier)
        TextWithHeaderContent(text = "AEB Trigger", modifier = textModifier)
    }
}
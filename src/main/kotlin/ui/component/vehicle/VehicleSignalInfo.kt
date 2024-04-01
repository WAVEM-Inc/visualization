package ui.component.vehicle

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import ui.component.common.TextWithHeaderContent
import ui.theme.Navy_200
import ui.theme.Poppins
import ui.theme.White_100
import viewmodel.UdpDataViewModel

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
            VehicleSignalInfoItem1(modifier = Modifier.weight(1f).padding(16.dp, 16.dp, 0.dp, 0.dp))
            VehicleSignalInfoItem2(modifier = Modifier.weight(1f).padding(16.dp, 16.dp, 0.dp, 0.dp))
            VehicleSignalInfoItem3(modifier = Modifier.weight(1f).padding(16.dp, 16.dp, 0.dp, 0.dp))
        }
    }
}

@Composable
private fun VehicleSignalInfoItem1(modifier: Modifier = Modifier) {
    val textModifier = Modifier.padding(vertical = 4.dp)

    var gp = mutableStateOf("NaN")
    var yr = mutableStateOf("NaN")
    var vs = mutableStateOf("NaN")
    var va = mutableStateOf("NaN")
    var sa = mutableStateOf("NaN")

    LaunchedEffect(Unit) {
        UdpDataViewModel.subscribeVehicleSignal(collector = { info ->
            gp.value = info.gearPosition.name
            yr.value = info.yawRate.toString()
            vs.value = info.speed.toString()
            va.value = info.accelerator.toString()
            sa.value = info.steeringAngle.toString()
        })
    }

    Column(modifier = modifier) {
        TextWithHeaderContent(text = "Gear Position", content = gp.value, modifier = textModifier)
        TextWithHeaderContent(text = "Yaw Rate", content = yr.value, modifier = textModifier)
        TextWithHeaderContent(text = "Vehicle Speed", content = vs.value, modifier = textModifier)
        TextWithHeaderContent(text = "Vehicle Accelerator", content = va.value, modifier = textModifier)
        TextWithHeaderContent(text = "Steering Angle", content = sa.value, modifier = textModifier)
    }
}

@Composable
private fun VehicleSignalInfoItem2(modifier: Modifier = Modifier) {
    val textModifier = Modifier.padding(vertical = 4.dp)

    var srs = mutableStateOf("NaN")
    var bti = mutableStateOf("NaN")
    var bto = mutableStateOf("NaN")
    var ts = mutableStateOf("NaN")
    var hl = mutableStateOf("NaN")

    LaunchedEffect(Unit) {
        UdpDataViewModel.subscribeVehicleSignal(collector = { info ->
            srs.value = info.steeringRotationSpeed.toString()
            bti.value = info.brakePressure.toString()
            bto.value = info.brakePressureFeedback.toString()
            ts.value = info.turnSignal.name
            hl.value = info.headLight.name
        })
    }

    Column(modifier = modifier) {
        TextWithHeaderContent(text = "Steering Rotation Speed", content = srs.value, modifier = textModifier)
        TextWithHeaderContent(text = "Brake Throttle Input", content = bti.value, modifier = textModifier)
        TextWithHeaderContent(text = "Brake Throttle Output", content = bto.value, modifier = textModifier)
        TextWithHeaderContent(text = "Turn Signal", content = ts.value, modifier = textModifier)
        TextWithHeaderContent(text = "Head Light", content = hl.value, modifier = textModifier)
    }
}

@Composable
private fun VehicleSignalInfoItem3(modifier: Modifier = Modifier) {
    val textModifier = Modifier.padding(vertical = 4.dp)

    var ws = mutableStateOf("NaN")
    var dl = mutableStateOf("NaN")
    var dO = mutableStateOf("NaN")
    var aeb = mutableStateOf("NaN")

    LaunchedEffect(Unit) {
        UdpDataViewModel.subscribeVehicleSignal(collector = { info ->
            ws.value = info.wheelSpeed.toString()
            dl.value = info.doorLock.name
            dO.value = info.doorOpen.name
            aeb.value = info.aebTrigger
        })
    }

    Column(modifier = modifier) {
        TextWithHeaderContent(text = "Wheel Speed", content = ws.value, modifier = textModifier)
        TextWithHeaderContent(text = "Door Lock", content = dl.value, modifier = textModifier)
        TextWithHeaderContent(text = "Door Open", content = dO.value, modifier = textModifier)
        TextWithHeaderContent(text = "AEB Trigger", content = aeb.value, modifier = textModifier)
    }
}
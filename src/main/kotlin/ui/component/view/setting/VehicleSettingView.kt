package ui.component.view.setting

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material.Checkbox
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp


@Composable
fun VehicleSettingView() {
    Column {
        Text(
            "Vehicle 표시 항목 설정",
            modifier = Modifier.wrapContentSize().padding(10.dp),
            fontSize = TextUnit(20f, TextUnitType.Sp),
            fontWeight = FontWeight.Bold
        )
        LazyColumn(modifier = Modifier.fillMaxSize().weight(1f).padding(start = 20.dp)) {
            item {
                VehicleSettingCheckBox("Gear Position", onCheckedChange = {})
                VehicleSettingCheckBox("Yaw Rate", onCheckedChange = {})
                VehicleSettingCheckBox("Vehicle Speed", onCheckedChange = {})
                VehicleSettingCheckBox("Vehicle Accelerator", onCheckedChange = {})
                VehicleSettingCheckBox("Steering Angle", onCheckedChange = {})
                VehicleSettingCheckBox("Steering rotation speed", onCheckedChange = {})
                VehicleSettingCheckBox("Brake Throttle", onCheckedChange = {})
                VehicleSettingCheckBox("Turn Signal", onCheckedChange = {})
                VehicleSettingCheckBox("Headlight On/Off", onCheckedChange = {})
                VehicleSettingCheckBox("Wheel Speed", onCheckedChange = {})
                VehicleSettingCheckBox("Hazard Signal", onCheckedChange = {})
                VehicleSettingCheckBox("Door Lock/Unlock", onCheckedChange = {})
                VehicleSettingCheckBox("Door Open/Close", onCheckedChange = {})
                VehicleSettingCheckBox("AEB Trigger", onCheckedChange = {})
            }
        }
    }
}

@Composable
fun VehicleSettingCheckBox(
    text: String,
    isChecked: Boolean = true,
    onCheckedChange: (isChecked: Boolean) -> Unit
) {
    val (checked, setChecked) = remember { mutableStateOf(isChecked) }

    Row {
        Text(text = text, modifier = Modifier.weight(1f).padding(10.dp).align(Alignment.CenterVertically))
        Checkbox(checked = checked, onCheckedChange = { bool ->
            setChecked(bool)
            onCheckedChange(isChecked)
        }
        )
    }
}
package ui.component.view.main

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.material.Checkbox
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp

@Composable
fun VehicleSignalView(modifier: Modifier = Modifier.wrapContentSize()) {
    val visibilities = remember { mutableStateListOf(true, true, true, true, true, true) }

    Column(modifier = modifier) {
        Column(
            modifier = Modifier.wrapContentSize().padding(bottom = 10.dp).align(Alignment.CenterHorizontally)
                .background(Color.DarkGray).alpha(0.8f).padding(10.dp)
        ) {
            visibilities.mapIndexed { index, b ->
                if (b) {
                    Text("Signal $index: XXX", modifier = Modifier, color = Color.Yellow)
                }
            }
        }
        Column(
            modifier = Modifier.wrapContentSize().background(Color.White)
                .border(1.dp, Color.LightGray).padding(10.dp).padding(end = 10.dp)
        ) {
            visibilities.mapIndexed { index, b ->
                VehicleSignalCheckView(
                    "Vehicle Signal $index",
                    onCheckedChange = { isChecked -> visibilities[index] = isChecked })
            }
        }
    }
}

@Composable
fun VehicleSignalCheckView(name: String, onCheckedChange: (isChecked: Boolean) -> Unit) {
    val (checked, setChecked) = remember { mutableStateOf(true) }

    Row(modifier = Modifier.wrapContentSize()) {
        Checkbox(checked, onCheckedChange = { bool ->
            setChecked(bool)
            onCheckedChange(bool)
        }, modifier = Modifier.align(Alignment.CenterVertically))
        Text(
            name,
            textAlign = TextAlign.Center,
            modifier = Modifier.wrapContentSize().align(Alignment.CenterVertically)
        )
    }
}
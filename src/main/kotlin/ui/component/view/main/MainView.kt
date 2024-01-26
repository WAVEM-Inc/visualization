package ui.component.view.main

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.Layout
import androidx.compose.ui.layout.MeasurePolicy
import androidx.compose.ui.unit.dp
import androidx.compose.ui.zIndex
import ui.component.view.MenuTabView


@Composable
fun MainView() {
    val menus = listOf("MainView", "Monitor")
    val (selected, setSelected) = remember { mutableStateOf(0) }

    Column(modifier = Modifier.fillMaxSize()) {
        MenuTabView(items = menus, selectedItemIndex = selected, onClick = setSelected)
        Box(modifier = Modifier.fillMaxSize().weight(1f).border(2.dp, Color.Black)) {
            VehicleSignalView(modifier = Modifier.wrapContentSize().align(Alignment.BottomEnd).zIndex(1f))
            when(selected) {
                0 -> Main3DView()
                1 -> MainMonitorView()
            }
        }
    }
}

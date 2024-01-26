package ui.component.layout

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.unit.dp
import ui.component.view.MultiColumnMenuTabView
import ui.component.view.TrafficLightView
import ui.component.view.setting.VehicleSettingView

@Composable
fun SettingViewLayout() {
    val (contentIndex, setContentIndex) = remember { mutableStateOf(0) }

    Column(modifier = Modifier.fillMaxSize().background(Color(0xffF5F7F8))) {
        Box(modifier = Modifier.fillMaxSize().weight(3f).border(2.dp, Color.Black)) {
            Column(modifier = Modifier.fillMaxWidth().wrapContentHeight()) {
                SettingMenuView(
                    modifier = Modifier.fillMaxWidth().wrapContentHeight(),
                    onSelectedMenuChange = { index ->
                        setContentIndex(index)
                    }
                )
                Box(modifier = Modifier.fillMaxSize().weight(1f)) {
                    VehicleSettingView()
                }
            }
        }

        Spacer(modifier = Modifier.fillMaxWidth().height(10.dp))

        Box(modifier = Modifier.fillMaxWidth().height(200.dp).border(2.dp, Color.Black)) {
            TrafficLightView()
        }
    }
}

@Composable
fun SettingMenuView(modifier: Modifier = Modifier, onSelectedMenuChange: (index: Int) -> Unit) {
    val menus = listOf("Vehicle", "GPS", "LiDAR", "Moblieye", "인지", "판단", "로깅", "리플레이")
    val (selected, setSelected) = remember { mutableStateOf(0) }
    var width by remember { mutableStateOf(0f) }


    Box(modifier.onGloballyPositioned { coordinates ->
        width = coordinates.size.width.toFloat() / 4f
    }) {
        MultiColumnMenuTabView(
            items = menus,
            selectedItemIndex = selected,
            columnNum = 2,
            tabWidth = width.dp,
            onClick = { index ->
                setSelected(index)
                onSelectedMenuChange(index)
            }
        )
    }
}
package ui.component.title

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.onClick
import androidx.compose.foundation.window.WindowDraggableArea
import androidx.compose.material.DropdownMenu
import androidx.compose.material.DropdownMenuItem
import androidx.compose.material.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.PointerEventType
import androidx.compose.ui.input.pointer.onPointerEvent
import androidx.compose.ui.text.font.FontStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.DpOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.WindowScope
import application.EvizWindowManager
import application.EvizWindowType
import ui.component.common.ImageButton
import ui.theme.*

@Preview
@Composable
fun WindowScope.EvizTitleBar(
    useMenu: Boolean = false, title: String, onMaximizeClick: () -> Unit,
    onMinimizeClick: () -> Unit,
    onExitClick: () -> Unit
) {
    WindowDraggableArea(
        modifier = Modifier.fillMaxWidth().height(50.dp).background(Black)
    ) {
        Box(
            modifier = Modifier.fillMaxSize()
        ) {
            if (useMenu) {
                EvizNewWindowMenus(
                    modifier = Modifier.align(Alignment.CenterStart).fillMaxHeight()
                )
            }

            Text(
                title,
                modifier = Modifier.wrapContentSize().align(Alignment.Center),
                color = White_100,
                fontFamily = Poppins,
                fontWeight = FontWeight.SemiBold,
                fontStyle = FontStyle.Normal,
                fontSize = 16.sp
            )

            EvizTitleOptions(
                modifier = Modifier.align(Alignment.CenterEnd), onMaximizeClick, onMinimizeClick, onExitClick
            )
        }
    }
}

@Composable
fun EvizTitleOptions(
    modifier: Modifier = Modifier,
    onMaximizeClick: () -> Unit,
    onMinimizeClick: () -> Unit,
    onExitClick: () -> Unit
) {
    Row(modifier = modifier) {
        ImageButton(
            "icon/ic_minimize_button.svg",
            onClick = onMinimizeClick
        )

        ImageButton(
            "icon/ic_maximize_button.svg",
            onClick = onMaximizeClick
        )

        ImageButton(
            "icon/ic_exit_button.svg",
            onClick = onExitClick
        )
    }
}

@OptIn(ExperimentalFoundationApi::class, ExperimentalComposeUiApi::class)
@Composable
fun EvizNewWindowMenus(modifier: Modifier = Modifier) {
    var isExpanded by remember { mutableStateOf(false) }
    val marginHorizontal = 16.dp

    Box(
        modifier = modifier
            .fillMaxHeight()
            .onClick {
                isExpanded = true
            }
            .background(
                if (isExpanded) {
                    LightGray
                } else {
                    Color.Transparent
                }
            )
            .padding(horizontal = marginHorizontal)
    ) {
        Text(
            text = "Window",
            fontFamily = Poppins,
            modifier = Modifier.align(Alignment.Center),
            fontSize = 16.sp,
            fontWeight = FontWeight.Normal,
            fontStyle = FontStyle.Normal,
            color = White_100
        )

        DropdownMenu(
            expanded = isExpanded,
            onDismissRequest = { isExpanded = false },
            offset = DpOffset(-marginHorizontal, 0.dp),
            modifier = Modifier
                .background(Navy_200)
                .border(2.dp, LightGray)
                .onPointerEvent(PointerEventType.Exit) {
                    isExpanded = false
                }
        ) {
            DropdownMenuItem(
                onClick = { EvizWindowManager.openWindow(EvizWindowType.DATA_RECORD) }
            ) {
                Text(text = "Recording", color = White_100)
            }

            DropdownMenuItem(
                onClick = { EvizWindowManager.openWindow(EvizWindowType.DATA_REPLAY) }
            ) {
                Text(text = "Replay", color = White_100)
            }

            DropdownMenuItem(
                onClick = { EvizWindowManager.openWindow(EvizWindowType.MOBILEYE) }
            ) {
                Text(text = "Mobileye", color = White_100)
            }

            DropdownMenuItem(
                onClick = { EvizWindowManager.openWindow(EvizWindowType.MAP) }
            ) {
                Text(text = "Map", color = White_100)
            }
        }
    }
}
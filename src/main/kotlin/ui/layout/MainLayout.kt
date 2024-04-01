package ui.layout

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsHoveredAsState
import androidx.compose.foundation.layout.*
import androidx.compose.material.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.painter.Painter
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.dp
import application.type.ui.EvizLayoutType
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import ui.theme.DarkGray
import ui.theme.LightGray
import ui.theme.White_100
import viewmodel.NotificationViewModel
import viewmodel.WindowViewModel


@Preview
@Composable
fun MainLayout(modifier: Modifier = Modifier.fillMaxSize()) {
    val currentPage = remember { mutableStateOf(EvizLayoutType.DASHBOARD) }
    val snackbarState = remember { SnackbarHostState() }
    val coroutineScope = rememberCoroutineScope()

    LaunchedEffect(Unit) {
        WindowViewModel.subscribeCurrentLayout { id ->
            currentPage.value = id
        }
    }

    LaunchedEffect(Unit) {
        coroutineScope.launch {
            NotificationViewModel.subscribeNotification { info ->
                coroutineScope.launch {
                    snackbarState.currentSnackbarData?.dismiss()
                }

                coroutineScope.launch {
                    snackbarState.showSnackbar(
                        message = info.message,
                        actionLabel = "확인",
                        duration = SnackbarDuration.Long
                    )
                }
            }
        }
    }

    Box(modifier.wrapContentSize()) {
        Row(modifier = modifier) {
            MainMenuTable()
            when (currentPage.value) {
                EvizLayoutType.DASHBOARD -> DashboardLayout()
                EvizLayoutType.MONITOR -> MonitorLayout()
                EvizLayoutType.SETTING -> SettingLayout()
            }
        }

        SnackbarHost(
            hostState = snackbarState,
            modifier = Modifier.align(Alignment.BottomCenter)
        )
    }
}

@Preview
@Composable
fun MainMenuTable() {
    val selectedIndex = remember { mutableStateOf(EvizLayoutType.DASHBOARD) }

    LaunchedEffect(Unit) {
        WindowViewModel.subscribeCurrentLayout { id ->
            selectedIndex.value = id
        }
    }

    Column(modifier = Modifier.width(140.dp).fillMaxHeight().background(DarkGray)) {
        Menu(
            onClick = {
                CoroutineScope(Dispatchers.Main).launch {
                    WindowViewModel.updateCurrentLayout(EvizLayoutType.DASHBOARD)
                }
            },
            isSelected = selectedIndex.value == EvizLayoutType.DASHBOARD,
            painter = painterResource("icon/ic_dashboard.svg")
        )
        Menu(
            onClick = {
                CoroutineScope(Dispatchers.Main).launch {
                    WindowViewModel.updateCurrentLayout(EvizLayoutType.MONITOR)
                }
            },
            isSelected = selectedIndex.value == EvizLayoutType.MONITOR,
            painter = painterResource("icon/ic_monitor.svg")
        )
        Menu(
            onClick = {
                CoroutineScope(Dispatchers.Main).launch {
                    WindowViewModel.updateCurrentLayout(EvizLayoutType.SETTING)
                }
            },
            isSelected = selectedIndex.value == EvizLayoutType.SETTING,
            painter = painterResource("icon/ic_setting.svg")
        )
    }
}

@Composable
fun Menu(modifier: Modifier = Modifier, painter: Painter, onClick: () -> Unit, isSelected: Boolean) {
    val interactionSource = remember { MutableInteractionSource() }
    val isHovered by interactionSource.collectIsHoveredAsState()
    val bgColor = if (isSelected) {
        LightGray
    } else if (isHovered) {
        LightGray
    } else {
        DarkGray
    }
    val contentColor = White_100

    Button(
        onClick = onClick,
        modifier = modifier.size(140.dp),
        interactionSource = interactionSource,
        colors = ButtonDefaults.buttonColors(
            backgroundColor = bgColor,
            contentColor = contentColor
        )
    ) {
        Box(modifier = Modifier.fillMaxSize()) {
            Image(
                painter = painter,
                contentDescription = null,
                modifier = Modifier.fillMaxSize(),
                contentScale = ContentScale.Crop
            )
        }
    }
}

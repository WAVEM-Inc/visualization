package application

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.*
import application.manager.EvizWindowManager
import application.type.ui.EvizWindowType
import ui.component.title.EvizTitleBar
import ui.layout.DataRecordLayout
import ui.layout.DataReplayLayout
import ui.layout.MapLayout
import ui.layout.MobileyeLayout
import ui.theme.Navy_100
import java.awt.Dimension

@Composable
fun ApplicationScope.EvizWindow(
    type: EvizWindowType,
    useMenu: Boolean = false,
    currentWindowState: WindowState = rememberWindowState()
) = Window(
    onCloseRequest = { EvizWindowManager.closeWindow(type) },
    title = type.title,
    state = currentWindowState,
    alwaysOnTop = true
) {
    window.minimumSize = Dimension(500, 350)

    Column(modifier = Modifier.fillMaxSize()) {
        EvizTitleBar(
            useMenu = useMenu,
            title = type.title,
            onMinimizeClick = {
                currentWindowState.isMinimized = true
            }, onMaximizeClick = {
                when (currentWindowState.placement) {
                    WindowPlacement.Maximized -> currentWindowState.placement = WindowPlacement.Floating
                    WindowPlacement.Floating -> currentWindowState.placement = WindowPlacement.Maximized
                    WindowPlacement.Fullscreen -> currentWindowState.placement = WindowPlacement.Floating
                }
            }, onExitClick = {
                EvizWindowManager.closeWindow(type)
            }
        )

        when(type) {
            EvizWindowType.DATA_REPLAY -> DataReplayLayout(modifier = Modifier.fillMaxSize().background(Navy_100).padding(16.dp))
            EvizWindowType.MOBILEYE -> MobileyeLayout(modifier = Modifier.fillMaxSize().background(Navy_100))
            EvizWindowType.DATA_RECORD -> DataRecordLayout(modifier = Modifier.fillMaxSize().background(Navy_100).padding(16.dp))
            EvizWindowType.MAP -> MapLayout()
        }
    }
}

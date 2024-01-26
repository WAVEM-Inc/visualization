package model

import androidx.compose.ui.Alignment
import androidx.compose.ui.unit.DpSize
import androidx.compose.ui.window.WindowPlacement
import androidx.compose.ui.window.WindowPosition
import androidx.compose.ui.window.WindowState
import kotlinx.coroutines.flow.MutableStateFlow
import window.util.getInitialWindowSize

object WindowViewModel {
    private val windowState = MutableStateFlow(
        WindowState(
            position = WindowPosition(Alignment.Center),
            size = getInitialWindowSize()
        )
    )

    fun getWindowState(): MutableStateFlow<WindowState> {
        return windowState
    }

    fun changeWindowPlacement() {
        if (windowState.value.placement == WindowPlacement.Fullscreen) {
            windowState.value.placement = WindowPlacement.Floating
        } else {
            windowState.value.placement = WindowPlacement.Fullscreen
        }
    }

    fun updateWindowMinimized(doMinimize: Boolean) {
        windowState.value.isMinimized = doMinimize
    }

    fun updateWindowSize(size: DpSize) {
        windowState.value.size = size
    }

    fun updateWindowPosition(position: WindowPosition) {
        windowState.value.position = position
    }
}
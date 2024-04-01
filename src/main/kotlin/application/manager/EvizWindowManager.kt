package application.manager

import androidx.compose.runtime.mutableStateMapOf
import application.type.ui.EvizWindowType

object EvizWindowManager {
    val windows = mutableStateMapOf(
        EvizWindowType.DATA_RECORD to false,
        EvizWindowType.DATA_REPLAY to false,
        EvizWindowType.MOBILEYE to false
    )

    fun openWindow(type: EvizWindowType) {
        windows[type] = true
    }

    fun closeWindow(type: EvizWindowType) {
        windows[type] = false
    }

}
import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material.MaterialTheme
import androidx.compose.material.Surface
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.window.Window
import androidx.compose.ui.window.application
import model.WindowViewModel
import ui.component.layout.MainLayout
import ui.component.title.AppWindowTitleBar
import java.awt.Dimension

@Composable
@Preview
fun App() {
    MaterialTheme {
        Box(modifier = Modifier.fillMaxSize()) {
            MainLayout()
        }
    }
}

fun main() = application {
    val windowState = WindowViewModel.getWindowState()
    Window(onCloseRequest = ::exitApplication, state = windowState.value, undecorated = true, resizable = true) {
        window.minimumSize = Dimension(1200, 675)
        CompositionLocalProvider {
            Surface(
                modifier = Modifier.fillMaxSize()
            ) {
                Column(
                    modifier = Modifier.fillMaxWidth().background(color = Color(0xffF6F4EB))
                ) {
                    AppWindowTitleBar(
                        onClose = { exitApplication() },
                        onDoubleClick = { WindowViewModel.changeWindowPlacement() }
                    )
                    App()
                }
            }
        }
    }
}
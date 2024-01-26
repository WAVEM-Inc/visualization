package ui.component.gl

import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.awt.SwingPanel
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.unit.IntSize
import com.jogamp.opengl.GLCapabilities
import com.jogamp.opengl.GLProfile
import com.jogamp.opengl.awt.GLCanvas
import com.jogamp.opengl.awt.GLJPanel
import com.jogamp.opengl.util.FPSAnimator
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import ui.opengl.OpenGL2Frame
import javax.swing.BoxLayout
import javax.swing.JPanel

@Composable
fun PointCloudViewer(modifier: Modifier = Modifier) {
    var size = remember { mutableStateOf(IntSize(0, 0)) }

    val profile = GLProfile.get(GLProfile.GL2)
    val capabilities = GLCapabilities(profile)

    val gljPanel = GLJPanel(capabilities)
    val b = OpenGL2Frame(gljPanel)

    DisposableEffect(Unit) {
        onDispose {
            b.stopJobs()
        }
    }

    SwingPanel(background = Color.White, modifier = modifier.onSizeChanged { intSize ->
        size.value = intSize
    }, factory = {
        JPanel().apply {
            gljPanel.addGLEventListener(b)
            gljPanel.setSize(width, height)

            layout = BoxLayout(this, BoxLayout.Y_AXIS)
            add(gljPanel)

            val animator = FPSAnimator(gljPanel, 300, true)
            animator.start()
        }
    })
}
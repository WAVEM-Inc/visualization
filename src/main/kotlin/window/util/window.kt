package window.util

import androidx.compose.ui.unit.DpSize
import androidx.compose.ui.unit.dp
import java.awt.Toolkit

fun getWindowSize(): DpSize {
    val dimen = Toolkit.getDefaultToolkit().screenSize
    val width = dimen.getWidth().dp
    val height = dimen.getHeight().dp

    return DpSize(width, height)
}

fun getInitialWindowSize(): DpSize {
    val size = getWindowSize()

    val aw = 16
    val ah = 9

    val width = size.width.value.toInt() * 0.7
    val height = ah * width / aw

    return DpSize(width.dp, height.dp)
}
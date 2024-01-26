package ui.component.view

import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color

fun Modifier.drawUnderline(strokeWidth: Float = 1f, color: Color = Color.Black) = then(
    Modifier.drawBehind {
        val y = size.height - strokeWidth / 2

        drawLine(color, Offset(0f, y), Offset(size.width, y), strokeWidth)
    }
)

fun Modifier.drawTopBottomLine(
    topStrokeWidth: Float = 1f,
    bottomStrokeWidth: Float = 1f,
    topStrokeColor: Color = Color.Black,
    bottomStrokeColor: Color = Color.Black
) = then(
    Modifier.drawBehind {
        val topY = 0f
        val bottomY = size.height - bottomStrokeWidth / 2

        drawLine(topStrokeColor, Offset(0f, topY), Offset(size.width, topY), topStrokeWidth)
        drawLine(bottomStrokeColor, Offset(0f, bottomY), Offset(size.width, bottomY), bottomStrokeWidth)
    }
)
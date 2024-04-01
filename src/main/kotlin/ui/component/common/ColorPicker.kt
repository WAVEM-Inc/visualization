package ui.component.common

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.onClick
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.center
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.layout.positionInRoot
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.IntSize
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Popup
import com.godaddy.android.colorpicker.ClassicColorPicker
import com.godaddy.android.colorpicker.HsvColor
import ui.theme.White_100

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun ColorPicker(
    modifier: Modifier = Modifier,
    color: Color = Color.Black,
    initialVisibility: Boolean = false,
    onColorChange: (changedColor: Color) -> Unit,
    colorPickerSize: IntSize = IntSize(220, 200)
) {
    var popupController by remember { mutableStateOf(initialVisibility) }

    Row(modifier = modifier) {
        // Colored circle
        Canvas(
            modifier = Modifier
                .size(30.dp)
                .align(Alignment.CenterVertically)
                .onClick {
                    popupController = true
                }
        ) {
            drawCircle(color = color, center = size.center, radius = size.width * 0.5f)
            drawCircle(color = White_100, center = size.center, radius = size.width * 0.5f, style = Stroke(width = 2f))
        }

        // Color picker
        if (popupController) {
            Popup(
                alignment = Alignment.TopEnd,
                offset = IntOffset(colorPickerSize.width, -colorPickerSize.height),
                onDismissRequest = { popupController = false }
            ) {
                ClassicColorPicker(
                    modifier = Modifier.size(colorPickerSize.width.dp, colorPickerSize.height.dp),
                    color = HsvColor.from(color),
                    onColorChanged = { hsvColor: HsvColor ->
                        onColorChange(hsvColor.toColor())
                    },
                    showAlphaBar = false
                )
            }
        }
    }
}
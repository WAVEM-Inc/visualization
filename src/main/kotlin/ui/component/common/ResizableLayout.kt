package ui.component.common

import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Icon
import androidx.compose.material.IconButton
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.input.pointer.PointerIcon
import androidx.compose.ui.input.pointer.pointerHoverIcon
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.DpSize
import androidx.compose.ui.unit.dp
import org.jetbrains.skiko.Cursor
import ui.theme.LightGray

@Composable
fun EvizVerticalResizableLayout(
    modifier: Modifier = Modifier,
    content: @Composable () -> Unit
) {
    var size: DpSize? by remember { mutableStateOf(null) }

    Box(
        modifier = modifier.defaultMinSize(10.dp, 10.dp).resize(size).onGloballyPositioned { coordinates ->
            size = DpSize(coordinates.size.width.dp, coordinates.size.height.dp)
        }
    ) {
        content()

        Spacer(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .height(10.dp)
                .pointerHoverIcon(PointerIcon(Cursor(Cursor.N_RESIZE_CURSOR)))
                .pointerInput(Unit) {
                    detectDragGestures { change, dragAmount ->
                        change.consume()
                        size = size?.plus(DpSize(0.dp, dragAmount.y.dp))
                    }
                }
        )
    }
}

@Composable
fun VerticalResizableLayout(
    initialHeight: Int = 500,
    modifier: Modifier = Modifier,
    content: @Composable () -> Unit
) {
    var size: DpSize? by remember { mutableStateOf(null) }

    Box(
        modifier = modifier.defaultMinSize(10.dp, 10.dp).resize(size).onGloballyPositioned { coordinates ->
            size = DpSize(coordinates.size.width.dp, coordinates.size.height.dp)
        }
    ) {
        content()

        Spacer(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .height(10.dp)
                .pointerHoverIcon(PointerIcon(Cursor(Cursor.N_RESIZE_CURSOR)))
                .pointerInput(Unit) {
                    detectDragGestures { change, dragAmount ->
                        change.consume()
                        size = size?.plus(DpSize(0.dp, dragAmount.y.dp))
                    }
                }
        )
    }
}

@Stable
fun Modifier.resize(size: DpSize?) = this.then(
    if (size != null) {
        this.size(size)
    } else {
        this
    }
)
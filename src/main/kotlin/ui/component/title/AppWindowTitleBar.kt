package ui.component.title

import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.AbsoluteRoundedCornerShape
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.window.WindowDraggableArea
import androidx.compose.material.Icon
import androidx.compose.material.IconButton
import androidx.compose.material.MaterialTheme
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.RectangleShape
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.WindowScope
import model.WindowViewModel


@OptIn(ExperimentalFoundationApi::class)
@Composable
fun WindowScope.AppWindowTitleBar(
    onDoubleClick: () -> Unit,
    onClose: () -> Unit,
) {
    WindowDraggableArea(
        modifier = Modifier.fillMaxWidth()
            .height(48.dp)
            .background(Color(0xffD0D4CA))
            .combinedClickable(
                onClick = {},
                onDoubleClick = onDoubleClick,
            )
    ) {
        Box(modifier = Modifier.fillMaxSize()) {
            Text(
                modifier = Modifier.align(Alignment.Center),
                fontSize = TextUnit(15f, TextUnitType.Sp),
                color = Color.Black,
                fontWeight = FontWeight.SemiBold,
                text = "자율주행 모니터링",
                fontFamily = FontFamily.SansSerif,
                textAlign = TextAlign.Center
            )
            WindowOptions(
                modifier = Modifier.align(Alignment.CenterEnd),
                onClose = onClose,
            )
        }
    }
}

@Composable
private fun WindowOptions(
    modifier: Modifier,
    onClose: () -> Unit
) {
    val density = LocalDensity.current
    Row(modifier = modifier) {
        // Window enlarge button
        IconButton(onClick = { WindowViewModel.updateWindowMinimized(true) }) {
            Image(
                painter = painterResource("icon/ic_minimize_window.svg"),
                contentDescription = null,
                modifier = Modifier.size(32.dp),
            )
        }

        // Window maximize button
        IconButton(onClick = { WindowViewModel.changeWindowPlacement() }) {
            Image(
                painter = painterResource("icon/ic_maximize_window.svg"),
                contentDescription = null,
                modifier = Modifier.size(32.dp)
            )
        }

        // Window exit button
        IconButton(onClick = onClose) {
            Icon(
                painter = painterResource("icon/ic_close_window.svg"),
                contentDescription = null,
                modifier = Modifier.size(32.dp).clipToBounds(),
                tint = Color.Red,
            )
        }
    }
}

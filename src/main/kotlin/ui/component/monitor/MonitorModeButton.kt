package ui.component.monitor

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.wrapContentSize
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Icon
import androidx.compose.material.IconButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.res.painterResource
import ui.theme.Black_100
import ui.theme.LightGray
import ui.theme.White_200


@Preview
@Composable
private fun PreViewer() {
    MonitorModeButton(
        modifier = Modifier.wrapContentSize().clip(RoundedCornerShape(15)),
        onModeChange = {},
    )
}

@Composable
fun MonitorModeButton(
    mode: MonitorMode = MonitorMode.VERTICAL,
    onModeChange: (MonitorMode) -> Unit,
    modifier: Modifier = Modifier
) {
    Row(modifier = modifier) {
        IconButton(
            modifier = Modifier.background(if (mode == MonitorMode.VERTICAL) LightGray else Black_100),
            onClick = {
                onModeChange(MonitorMode.VERTICAL)
            }
        ) {
            Icon(
                painter = painterResource("icon/ic_monitor_vertical.svg"),
                contentDescription = null,
                tint = White_200
            )
        }

        IconButton(
            modifier = Modifier.background(if (mode == MonitorMode.HORIZONTAL) LightGray else Black_100),
            onClick = {
                onModeChange(MonitorMode.HORIZONTAL)
            }
        ) {
            Icon(
                painter = painterResource("icon/ic_monitor_horizontal.svg"),
                contentDescription = null,
                tint = White_200
            )
        }
    }
}

enum class MonitorMode {
    HORIZONTAL,
    VERTICAL
}
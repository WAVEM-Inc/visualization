package ui.component.view.main

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.zIndex
import ui.composable.PointCloudViewer

@Composable
fun Main3DView(
) {
    Box(modifier = Modifier.zIndex(0.5f).border(2.dp, Color.Black).padding(2.dp)) {
        PointCloudViewer()
    }
}
package ui.component.view.main

import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.text.selection.SelectionContainer
import androidx.compose.material.IconButton
import androidx.compose.material.Text
import androidx.compose.material.TextField
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.AnnotatedString
import androidx.compose.ui.text.input.TextFieldValue
import androidx.compose.ui.unit.dp
import ui.component.view.drawTopBottomLine
import ui.component.view.drawUnderline
import util.string.tab

@Composable
fun MainMonitorView(modifier: Modifier = Modifier) {
    Column(modifier = modifier) {
        GpsMonitorView(modifier = getViewModifier().then(Modifier.weight(1f)))
        LidarMonitorView(modifier = getViewModifier().then(Modifier.weight(1f)))
        CameraMonitorView(modifier = getViewModifier().then(Modifier.weight(1f)))
    }
}

@Composable
fun GpsMonitorView(
    utcTime: Double = 1681363045.930180,
    strRtk: String = "RTK_FIXED",
    strIns: String = "INS_GOOD",
    strSol: String = "SOL_GOOD",
    modifier: Modifier = Modifier
) {
    val text = "RTK상태: ${strRtk}${tab()}INS상태: ${strIns}${tab()}Sol상태: ${strSol}"

    Box(modifier) {
        SelectionContainer {
            Column {
                Text(text = "GPS UTCtime: $utcTime", modifier = Modifier.padding(start = 5.dp, top = 5.dp))
                Text(text = AnnotatedString(text), modifier = getInfoTextModifier())
                LazyColumn(getLogLazyColumnModifier()) {
                    item {
                        var log = "GPS 데이터 로그\n\n"
                        for (i in 1..100) {
                            log += "Timestamp$i: GPS 데이터 메시지$i\n"
                        }

                        TextField(log, onValueChange = {}, readOnly = true)
                    }
                }
            }
        }
        IconButton(onClick = { }, modifier = Modifier.padding(16.dp).size(32.dp).align(Alignment.BottomEnd)) {
            Image(
                painter = painterResource("icon/ic_down_circle_arrow.svg"),
                contentDescription = null
            )
        }
    }
}

@Composable
private fun LidarMonitorView(
    total: Int = 70000,
    front: Int = 40000,
    left: Int = 15000,
    right: Int = 15000,
    modifier: Modifier = Modifier
) {
    val text = "Total: $total${tab()}Front: $front${tab()}Left: $left${tab()}Right: $right"

    Box(modifier) {
        SelectionContainer {
            Column {
                Text(text = "LiDAR PointCloud수", modifier = Modifier.padding(start = 5.dp, top = 5.dp))
                Text(text = text, modifier = getInfoTextModifier())

                LazyColumn(getLogLazyColumnModifier()) {
                    item {
                        var text = "LiDAR 데이터 로그\n\n"
                        for (i in 1..100) {
                            text += "Timestamp$i: LiDAR 데이터 메시지$i\n"
                        }
                        TextField(text, onValueChange = {}, readOnly = true)
                    }
                }
            }
        }
    }
}


@Composable
private fun CameraMonitorView(
    red: Boolean = true,
    yellow: Boolean = false,
    left: Boolean = true,
    green: Boolean = false,
    modifier: Modifier = Modifier
) {
    val text = "RED: ${getText(red)}${tab()}YELLOW: ${getText(yellow)}${tab()}LEFT: ${getText(left)}${tab()}GREEN: ${
        getText(green)
    }"

    Box(modifier) {
        SelectionContainer {
            Column {
                Text("Camera", modifier = Modifier.padding(start = 5.dp, top = 5.dp))
                Text(text, modifier = getInfoTextModifier())

                LazyColumn(getLogLazyColumnModifier()) {
                    item {
                        var log = "Camera 데이터 로그\n\n"
                        for (i in 1..100) {
                            log += "Timestamp$i: Camera 데이터 메시지$i\n"
                        }
                        TextField(log, onValueChange = { }, readOnly = true)
                    }
                }
            }
        }
    }
}

private fun getText(boolean: Boolean): String {
    return if (boolean) {
        "O"
    } else {
        "X"
    }
}

private fun getViewModifier(): Modifier {
    return Modifier.fillMaxWidth().drawTopBottomLine(topStrokeWidth = 1f, bottomStrokeWidth = 1f)
}

private fun getInfoTextModifier(): Modifier {
    return Modifier.fillMaxWidth().drawUnderline(strokeWidth = 2f).padding(start = 5.dp, bottom = 10.dp)
}

private fun getLogLazyColumnModifier(): Modifier {
    return Modifier.padding(5.dp).fillMaxWidth()
}

private fun getLogTitleTextModifier(): Modifier {
    return Modifier.padding(bottom = 5.dp)
}

private fun getLogItemTextModifier(): Modifier {
    return Modifier.padding(start = 10.dp)
}
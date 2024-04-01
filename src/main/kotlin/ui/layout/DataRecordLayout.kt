package ui.layout

import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import ui.component.common.EvizText
import ui.component.common.TextWithHeader
import ui.theme.*
import java.net.Inet4Address
import kotlin.concurrent.timer

@Composable
fun DataRecordLayout(
    modifier: Modifier = Modifier.fillMaxSize()
) {
    var doRecord by remember { mutableStateOf(false) }
    var editable by remember { mutableStateOf(doRecord.not()) }
    var startTime: Long by remember { mutableStateOf(0) }
    var countTime: Long by remember { mutableStateOf(0) }

    if (doRecord) {
        timer(period = 100, initialDelay = 100) {
            countTime = System.currentTimeMillis() - startTime

            if (!doRecord) {
                cancel()
            }
        }
    }

    Column(modifier = modifier) {
        EvizText("Save Directory", fontSize = 24f, fontWeight = FontWeight.Bold)
        DirectorySelector(modifier = Modifier.fillMaxWidth().height(60.dp))

        EvizText(
            text = "Save Options",
            fontSize = 24f,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.padding(top = 32.dp)
        )

        SaveOptions(
            editable = editable,
            modifier = Modifier.fillMaxWidth().wrapContentHeight().weight(1f).background(LightGray)
        )

        Column(modifier = Modifier.fillMaxWidth().wrapContentHeight()) {
            TextTimer(countTime, modifier = Modifier.align(Alignment.CenterHorizontally).padding(top = 32.dp))

            Button(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(16.dp)
                    .align(Alignment.CenterHorizontally),
                colors = ButtonDefaults.buttonColors(backgroundColor = Orange),
                onClick = {
                    doRecord = doRecord.not()
                    editable = doRecord.not()
                    if (doRecord) {
                        startTime = System.currentTimeMillis()
                    }
                }
            ) {
                EvizText(
                    text = if (doRecord) "Stop Recording" else "Start Recording",
                    color = White_100,
                    fontSize = 16f,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(vertical = 8.dp)
                )
            }
        }
    }
}

@Composable
private fun DirectorySelector(
    modifier: Modifier = Modifier
) {
    val path by remember { mutableStateOf("폴더를 선택해주세요.") }
    Box(modifier = modifier.background(LightGray), contentAlignment = Alignment.Center) {
        Row(modifier = Modifier.fillMaxSize().align(Alignment.Center)) {
            Box(modifier = modifier.align(Alignment.CenterVertically).padding(16.dp).weight(1f).background(White_200)) {
                Text(
                    text = path,
                    modifier = Modifier.align(Alignment.CenterStart).padding(start = 8.dp)
                )
            }

            IconButton(modifier = Modifier.align(Alignment.CenterVertically).padding(horizontal = 8.dp), onClick = {}) {
                Image(
                    painterResource("icon/ic_open_file.svg"),
                    contentDescription = null,
                    contentScale = ContentScale.Fit
                )
            }
        }
    }
}

@Composable
private fun SaveOptions(
    editable: Boolean,
    modifier: Modifier = Modifier.fillMaxWidth()
) {
    val scrollState = rememberLazyListState()
    val scrollAdapter = rememberScrollbarAdapter(scrollState)
    val itemModifier = Modifier.fillMaxWidth()

    Box(modifier.padding(start = 8.dp)) {
        LazyColumn(modifier = Modifier.fillMaxWidth().wrapContentHeight(), state = scrollState) {
            item {
                SaveOptionItem(modifier = itemModifier, title = "Vehicle", editable = editable, onCheckedChange = {})
                SaveOptionItem(modifier = itemModifier, title = "GPS", editable = editable, onCheckedChange = {})
                SaveOptionItem(modifier = itemModifier, title = "LiDAR", editable = editable, onCheckedChange = {})
                SaveOptionItem(modifier = itemModifier, title = "Camera", editable = editable, onCheckedChange = {})
                SaveOptionItem(modifier = itemModifier, title = "Mobileye", editable = editable, onCheckedChange = {})
                SaveOptionItem(modifier = itemModifier, title = "Perception", editable = editable, onCheckedChange = {})
            }
        }

        VerticalScrollbar(
            modifier = Modifier.align(Alignment.TopEnd),
            adapter = scrollAdapter,
            style = ScrollbarStyle(
                minimalHeight = 20.dp,
                thickness = 8.dp,
                shape = RoundedCornerShape(50),
                hoverDurationMillis = 200,
                unhoverColor = White_100,
                hoverColor = Gray
            )
        )
    }
}

@Composable
private fun SaveOptionItem(
    title: String,
    editable: Boolean,
    onCheckedChange: (Boolean) -> Unit,
    modifier: Modifier = Modifier
) {
    var checked by remember { mutableStateOf(true) }

    Box(modifier) {
        TextWithHeader(text = title, modifier = Modifier.align(Alignment.CenterStart), fontSize = 14f)

        Checkbox(
            modifier = Modifier.align(Alignment.CenterEnd).padding(end = 8.dp),
            checked = checked,
            enabled = editable,
            colors = CheckboxDefaults.colors(
                checkedColor = Color(0xff57B8FF),
                uncheckedColor = White_100,
                disabledColor = Gray
            ),
            onCheckedChange = { b ->
                checked = b
                onCheckedChange(b)
            }
        )
    }
}

@Composable
private fun TextTimer(
    countTime: Long,
    modifier: Modifier = Modifier
) {
    val totalSeconds = (countTime / 1000).toInt()
    val hours = totalSeconds / 3600
    val minutes = (totalSeconds % 3600) / 60
    val seconds = totalSeconds % 60
    val millis = (countTime % 1000).toInt()

    val time = String.format("%02d:%02d:%02d:%03d", hours, minutes, seconds, millis)

    EvizText(
        text = time,
        fontSize = 32f,
        fontWeight = FontWeight.Bold,
        fontFamily = FontFamily.Monospace,
        modifier = modifier
    )
}


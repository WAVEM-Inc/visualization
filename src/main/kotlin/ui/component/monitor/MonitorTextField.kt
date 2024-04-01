package ui.component.monitor

import androidx.compose.desktop.ui.tooling.preview.Preview
import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyRow
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.input.key.Key
import androidx.compose.ui.input.key.key
import androidx.compose.ui.input.key.onKeyEvent
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import ui.component.common.EvizText
import ui.component.common.VerticalResizableLayout
import ui.theme.Black
import ui.theme.LightGray
import ui.theme.White_200


@Preview
@Composable
fun MonitorPreview() {
    val contents = remember { mutableStateListOf<String>() }

    LaunchedEffect(true) {
        for (i in 1..1000) {
            delay(200)
            contents.add("${i}. 2023-10-25 14:30:00.0, Lat: 37.7749, Lon: -122.4194")
        }
    }
    MonitorTextField(
        title = "Gps Data",
        info = "GPS UTCtime: 1641363045",
        infoList = listOf("RTK Status: RTK_FIXED", "INS Status: INS_GOOD", "SOL Status: SOL_ GOOD"),
        contents = contents,
        modifier = Modifier.padding(16.dp).heightIn(0.dp, 600.dp)
    )
}

@Composable
fun MonitorTextField(
    title: String,
    info: String,
    infoList: List<String>,
    contents: List<String>,
    modifier: Modifier = Modifier
) {
    var isExpanded by remember { mutableStateOf(true) }
    var buttonOffset by remember { mutableStateOf(IntOffset(0, 0)) }

    Column(modifier = modifier) {
        EvizText(text = title, fontWeight = FontWeight.Bold, fontSize = 24f)

        MonitorInfoField(
            info = info,
            infoList = infoList,
            modifier = Modifier
                .fillMaxWidth()
                .background(White_200)
                .drawBehind {
                    drawLine(
                        color = Black,
                        start = Offset(0f, size.height),
                        end = Offset(size.width, size.height),
                        strokeWidth = 2f,
                        pathEffect = PathEffect.dashPathEffect(floatArrayOf(10f, 10f), 0f)
                    )
                }
        )

        if (isExpanded) {
            VerticalResizableLayout(modifier = Modifier.heightIn()) {
                MonitorContentField(
                    contents = contents,
                    modifier = Modifier.fillMaxSize()
                )
            }
        }

        Icon(
            painter = painterResource("icon/ic_arrow_down.svg"),
            contentDescription = null,
            modifier = Modifier
                .clip(RoundedCornerShape(0, 0, 10, 10))
                .background(LightGray)
                .align(Alignment.CenterHorizontally)
                .size(60.dp, 25.dp)
                .clickable(onClick = { isExpanded = isExpanded.not() })
        )
    }
}

@Composable
private fun MonitorInfoField(
    info: String,
    infoList: List<String>,
    modifier: Modifier = Modifier,
) {
    Column(modifier) {
        EvizText(
            text = info,
            color = Black,
            fontSize = 16f,
            modifier = Modifier.padding(8.dp),
            fontWeight = FontWeight.SemiBold
        )
        LazyRow(horizontalArrangement = Arrangement.SpaceBetween, modifier = Modifier.padding(8.dp).fillMaxWidth()) {
            items(infoList) { text ->
                EvizText(text = text, color = Black, fontSize = 12f)
            }
        }
    }
}

@Composable
private fun MonitorContentField(
    contents: List<String>,
    modifier: Modifier = Modifier
) {
    val scrollState = rememberScrollState(0)
    val coroutineScope = rememberCoroutineScope()

    var contentText: String = ""
    contents.forEach { s ->
        contentText += "${s}\n"
    }

    coroutineScope.launch {
        if (scrollState.value in scrollState.maxValue - 30..scrollState.maxValue) {
            scrollState.scrollTo(scrollState.maxValue)
        }
    }

    TextField(
        value = contentText,
        onValueChange = {},
        readOnly = true,
        modifier = modifier.verticalScroll(scrollState).onKeyEvent(onKeyEvent = { keyEvent ->
            if (keyEvent.key.keyCode == Key.PageUp.keyCode) {
                coroutineScope.launch {
                    scrollState.scrollTo(0)
                }
            } else if (keyEvent.key.keyCode == Key.PageDown.keyCode) {
                coroutineScope.launch {
                    scrollState.scrollTo(scrollState.maxValue)
                }
            }
            true
        }),
        colors = TextFieldDefaults.textFieldColors(
            backgroundColor = White_200,
            focusedIndicatorColor = Color.Transparent
        ),
    )
}
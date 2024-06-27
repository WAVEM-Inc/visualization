package ui.layout

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import application.connect.TcpClient
import application.type.msg.ProtoMessageType
import com.darkrockstudios.libraries.mpfilepicker.FilePicker
import essys_middle.streaming.Streaming.PlaybackState
import essys_middle.streaming.Streaming.StreamingData
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import ui.component.common.TextWithHeaderContent
import ui.theme.*
import viewmodel.ReplayViewModel

@Composable
fun DataReplayLayout(modifier: Modifier = Modifier.fillMaxSize()) {
    DisposableEffect(Unit) {
        onDispose {
            CoroutineScope(Dispatchers.Main).launch {
                ReplayViewModel.updateReplayRequestState(PlaybackState.NONE)
                ReplayViewModel.updateReplayFilePath("")

                val value = StreamingData.newBuilder()
                    .setTimestamp(System.currentTimeMillis())
                    .setState(PlaybackState.NONE)
                    .setFilePath("")
                    .build()

                TcpClient.sendData(ProtoMessageType.REPLAY.type, value.toByteArray())
            }
        }
    }
    Column(modifier = modifier) {
        Column(modifier = Modifier.fillMaxSize().weight(1f).padding(8.dp)) {
            Text(
                modifier = Modifier,
                text = "Load Logging File",
                fontFamily = Poppins,
                fontWeight = FontWeight.Bold,
                fontSize = TextUnit(24f, TextUnitType.Sp),
                color = White_100
            )

            LoadDirectory(modifier = Modifier.fillMaxWidth().height(60.dp))

//            Text(
//                modifier = Modifier.padding(top = 32.dp),
//                text = "Replay List",
//                fontFamily = Poppins,
//                fontWeight = FontWeight.Bold,
//                fontSize = TextUnit(24f, TextUnitType.Sp),
//                color = White_100
//            )
//            ReplayList()
        }

        ReplayController(Modifier.fillMaxWidth().height(100.dp))
    }
}

@Composable
fun LoadDirectory(modifier: Modifier = Modifier) {
    var path by remember { mutableStateOf<String>("") }
    var showFilePicker by remember { mutableStateOf(false) }

    LaunchedEffect(Unit) {
        CoroutineScope(Dispatchers.Main).launch {
            ReplayViewModel.subscribeReplayFilePath(collector = {
                path = it
            })
        }
    }

    FilePicker(show = showFilePicker) { filePath ->
        CoroutineScope(Dispatchers.Main).launch {
            if (filePath != null) {

                CoroutineScope(Dispatchers.Main).launch {
                    ReplayViewModel.updateReplayFilePath(filePath.path)
                    ReplayViewModel.updateReplayState(PlaybackState.REQUEST_PLAY)
                    ReplayViewModel.updateReplayRequestState(PlaybackState.REQUEST_PLAY)

                    val value = StreamingData.newBuilder()
                        .setTimestamp(System.currentTimeMillis())
                        .setFilePath(path)
                        .build()

                    try {
                        TcpClient.sendData(ProtoMessageType.REPLAY.type, value.toByteArray())
                    } catch (e: Exception) { }
                }
            }
        }
        showFilePicker = false
    }

    Box(modifier = modifier.background(LightGray), contentAlignment = Alignment.Center) {
        Row(modifier = Modifier.fillMaxSize().align(Alignment.Center)) {
            Box(modifier = modifier.align(Alignment.CenterVertically).padding(16.dp).weight(1f).background(White_200)) {
                Text(
                    text = path,
                    modifier = Modifier.align(Alignment.CenterStart).padding(start = 8.dp)
                )
            }

            IconButton(modifier = Modifier.align(Alignment.CenterVertically).padding(horizontal = 8.dp), onClick = {
                showFilePicker = true
            }) {
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
fun ReplayList(modifier: Modifier = Modifier) {
    LazyColumn(modifier = modifier.padding(start = 8.dp)) {
        item {
            ReplayItem(modifier = Modifier.fillMaxWidth().wrapContentHeight(), title = "Vehicle", onCheckedChange = {})
            ReplayItem(modifier = Modifier.fillMaxWidth().wrapContentHeight(), title = "GPS", onCheckedChange = {})
            ReplayItem(modifier = Modifier.fillMaxWidth().wrapContentHeight(), title = "LiDAR", onCheckedChange = {})
            ReplayItem(modifier = Modifier.fillMaxWidth().wrapContentHeight(), title = "Camera", onCheckedChange = {})
            ReplayItem(modifier = Modifier.fillMaxWidth().wrapContentHeight(), title = "Mobileye", onCheckedChange = {})
            ReplayItem(
                modifier = Modifier.fillMaxWidth().wrapContentHeight(),
                title = "Perception",
                onCheckedChange = {})
        }
    }
}

@Composable
fun ReplayItem(modifier: Modifier = Modifier, title: String, onCheckedChange: (b: Boolean) -> Unit) {
    var checked by remember { mutableStateOf(true) }

    Box(modifier = modifier) {
        TextWithHeaderContent(
            text = title,
            modifier = Modifier.align(Alignment.CenterStart),
            textSize = TextUnit(14f, TextUnitType.Sp)
        )
        Checkbox(
            checked,
            modifier = Modifier.align(Alignment.CenterEnd),
            colors = CheckboxDefaults.colors(
                checkedColor = Color(0xff57B8FF),
                uncheckedColor = White_100
            ),
            onCheckedChange = { b ->
                checked = b
                onCheckedChange(b)
            }
        )
    }
}

@Composable
fun ReplayController(modifier: Modifier = Modifier) {
    var isPlaying by remember { mutableStateOf(false) }
    var progress by remember { mutableStateOf<Float>(0f) }
    var isDragged by remember { mutableStateOf<Boolean>(false) }

    LaunchedEffect(Unit) {
        CoroutineScope(Dispatchers.Main).launch {
            ReplayViewModel.subscribeReplayState(collector = {
                if (it == PlaybackState.RESPONSE_PLAY) {
                    isPlaying = true
                } else {
                    isPlaying = false
                }
            })
        }

        CoroutineScope(Dispatchers.Main).launch {
            ReplayViewModel.subscribeReplayProgress(collector = {
                progress = (it * 0.01f).toFloat()
                println(progress)
            })
        }
    }

    Column(modifier = modifier.background(Navy_200)) {
        Slider(
            modifier = Modifier.fillMaxWidth().height(12.dp),
            value = progress,
            onValueChange = {
                progress = it
            },
            onValueChangeFinished = {
                val value = StreamingData.newBuilder()
                    .setTimestamp(System.currentTimeMillis())
                    .setState(PlaybackState.REQUEST_MOVING_ON)
                    .setFilePath(ReplayViewModel.filePath)
                    .setProgress((progress * 100).toDouble())
                    .build()

                try {
                    TcpClient.sendData(ProtoMessageType.REPLAY.type, value.toByteArray())
                } catch (e: Exception) {

                }
            })

        Row(modifier = Modifier.wrapContentSize().align(Alignment.CenterHorizontally)) {
            val btnModifier = Modifier.aspectRatio(1f, true).padding(10.dp)
            val imgModifier = Modifier.fillMaxSize(0.6f).align(Alignment.CenterVertically)

//            IconButton(modifier = btnModifier, onClick = {}) {
//                Image(
//                    painter = painterResource("icon/ic_replay_backward.svg"),
//                    contentDescription = null,
//                    modifier = imgModifier,
//                    alignment = Alignment.Center
//                )
//            }
            IconButton(modifier = btnModifier, onClick = {
                if (isPlaying) {
                    CoroutineScope(Dispatchers.Main).launch {
                        ReplayViewModel.updateReplayRequestState(PlaybackState.REQUEST_PAUSE)

                        val value = StreamingData.newBuilder()
                            .setTimestamp(System.currentTimeMillis())
                            .setState(PlaybackState.REQUEST_PAUSE)
                            .setFilePath(ReplayViewModel.filePath)
                            .build()

                        try {
                            TcpClient.sendData(ProtoMessageType.REPLAY.type, value.toByteArray())
                        } catch (e: Exception) {
                            e.printStackTrace()
                        }
                    }
                } else {
                    CoroutineScope(Dispatchers.Main).launch {
                        ReplayViewModel.updateReplayRequestState(PlaybackState.REQUEST_PLAY)

                        val value = StreamingData.newBuilder()
                            .setTimestamp(System.currentTimeMillis())
                            .setState(PlaybackState.REQUEST_PLAY)
                            .setFilePath(ReplayViewModel.filePath)
                            .build()

                        try {
                            TcpClient.sendData(ProtoMessageType.REPLAY.type, value.toByteArray())
                        } catch (e: Exception) { }
                    }
                }
            }) {
                Image(
                    painter = if (isPlaying) painterResource("icon/ic_replay_pause.svg") else painterResource("icon/ic_replay_play.svg"),
                    contentDescription = null,
                    modifier = imgModifier,
                    alignment = Alignment.Center
                )
            }
            IconButton(modifier = btnModifier, onClick = {
                CoroutineScope(Dispatchers.Main).launch {
                    ReplayViewModel.updateReplayRequestState(PlaybackState.REQUEST_STOP)

                    val value = StreamingData.newBuilder()
                        .setTimestamp(System.currentTimeMillis())
                        .setState(PlaybackState.REQUEST_STOP)
                        .setFilePath(ReplayViewModel.filePath)
                        .build()

                    try {
                        TcpClient.sendData(ProtoMessageType.REPLAY.type, value.toByteArray())
                    } catch (e: Exception) {

                    }
                }
            }) {
                Image(
                    painter = painterResource("icon/ic_replay_stop.svg"),
                    contentDescription = null,
                    modifier = imgModifier,
                    alignment = Alignment.Center
                )
            }
        }
    }
}

///**
// * Creates a Composable that displays the progress of a task or state.
// * This Composable adjusts the size of the content based on the given modifier.
// *
// * @param modifier Modifier used to adjust the layout algorithm or draw decoration content. (ex. background)
// * @param progress Integer data representing the current progress, ranging from 0 to 100.
// * @param color Color used to indicate the progress.
// */
//@Composable
//fun ProgressBar(modifier: Modifier = Modifier.fillMaxWidth().height(24.dp), progress: Int = 0, color: Color = Orange) {
//    var size by remember { mutableStateOf(IntSize(0, 0)) }
//    val progressValue = progress.coerceIn(0, 100)
//
//    Box(modifier = modifier.onSizeChanged { intSize ->
//        size = intSize
//    }) {
//
//        Canvas(
//            modifier = Modifier
//                .fillMaxWidth()
//                .fillMaxHeight(0.9f)
//                .background(LightGray)
//                .align(Alignment.Center)
//        ) {
//            drawRect(
//                color = color,
//                topLeft = Offset(0f, 0f),
//                size = Size(width = size.width / (100f / progressValue), height = drawContext.size.height)
//            )
//        }
//
//        Image(
//            painter = painterResource("icon/ic_progress_header.svg"),
//            contentDescription = null,
//            contentScale = ContentScale.Fit,
//            modifier = Modifier
//                .aspectRatio(1f)
//                .fillMaxHeight()
//                .align(Alignment.CenterStart)
//                .offset(x = (size.width / (100f / progressValue) - size.height * 0.5f).dp, y = 0.dp)
//        )
//    }
//}
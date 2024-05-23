package ui.component.setting.obstacle

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import application.type.data.NotificationMessage
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import ui.component.common.*
import ui.component.setting.lidar.LidarOption
import ui.theme.*
import viewmodel.NotificationViewModel


@Composable
fun ObstacleOptionLayout(
    modifier: Modifier = Modifier.background(Navy_200)
) {
    val option = remember {
        mutableStateMapOf(
            OptionType.DRAW to false, OptionType.ID to false, OptionType.TYPE to false, OptionType.HEADING to false,
            OptionType.POSITION to false, OptionType.TTC to false, OptionType.RISK to false, OptionType.SPEED to false,
        )
    }
    var textColor by remember { mutableStateOf(White_100) }
    var strTextColor by remember { mutableStateOf(colorTohex(White_100)) }
    var textSize by remember { mutableStateOf(10) }

    LaunchedEffect(Unit) {
        ObstacleOptionCache.updateData()

        val cache = ObstacleOptionCache.obstacleOption
        option[OptionType.DRAW] = cache.draw
        option[OptionType.ID] = cache.useId
        option[OptionType.TYPE] = cache.useType
        option[OptionType.HEADING] = cache.useHeading
        option[OptionType.POSITION] = cache.usePosition
        option[OptionType.SPEED] = cache.useSpeed
        option[OptionType.TTC] = cache.useTtc
        option[OptionType.RISK] = cache.useRisk
        strTextColor = cache.textColor
        textColor = hexToColor(cache.textColor)
        textSize = cache.textSize
    }

    Column(modifier = modifier) {
        Column(modifier = Modifier.padding(horizontal = 16.dp).padding(top = 8.dp).fillMaxWidth()) {
            EvizText(text = "Dashboard Options", fontSize = 22f, fontWeight = FontWeight.Bold)
            TextWithSwitch(
                checked = option[OptionType.DRAW]!!,
                modifier = Modifier.padding(start = 16.dp),
                text = "- Drawing in Dashboard",
                onCheckedChange = { b ->
                    option[OptionType.DRAW] = b
                    ObstacleOptionCache.obstacleOption.draw = b
                }
            )

            Row(modifier = Modifier.padding(8.dp).padding(end = 32.dp), verticalAlignment = Alignment.CenterVertically) {
                TextWithHeader(text = "Text Color:", modifier = Modifier.padding(end = 8.dp))

                EditText(
                    value = strTextColor,
                    modifier = Modifier.width(150.dp).height(50.dp).padding(end = 8.dp),
                    colors = TextFieldDefaults.textFieldColors(
                        focusedIndicatorColor = Neptune,
                        unfocusedIndicatorColor = White_200,
                        cursorColor = White_200
                    ),
                    onValueChange = { s ->
                        try {
                            val color = hexToColor(s)
                            strTextColor = colorTohex(color)
                            textColor = color
                            ObstacleOptionCache.obstacleOption.textColor = strTextColor
                        } catch (_: Exception) {
                        }
                    }
                )

                ColorPicker(
                    modifier = Modifier.size(20.dp),
                    color = textColor,
                    onColorChange = { color ->
                        textColor = color
                        strTextColor = colorTohex(color)
                        ObstacleOptionCache.obstacleOption.textColor = colorTohex(color)
                    }
                )

                TextWithHeader(text = "Text Size:", modifier = Modifier.padding(start = 32.dp))
                EditText(
                    modifier = Modifier.width(150.dp).height(50.dp).padding(start = 8.dp),
                    value = textSize.toString(),
                    colors = TextFieldDefaults.textFieldColors(
                        focusedIndicatorColor = Neptune,
                        unfocusedIndicatorColor = White_200,
                        cursorColor = White_200
                    ),
                    onValueChange = {
                        try {
                            val intValue = it.toInt()
                            textSize = intValue
                            ObstacleOptionCache.obstacleOption.textSize = intValue
                        } catch (_: Exception) {
                        }
                    }
                )
            }
        }
//        DashboardOption(
//            modifier = Modifier.padding(horizontal = 16.dp).padding(top = 8.dp).fillMaxWidth(),
//            draw = option[OptionType.DRAW]!!,
//            onDrawChange = { b ->
//                option[OptionType.DRAW] = b
//                ObstacleOptionCache.obstacleOption.draw = b
//            }
//        )

        ObstacleOptionEditor(
            modifier = Modifier.padding(horizontal = 16.dp).padding(top = 40.dp),
            data = option.toMap(), onDataChange = { s, b ->
                option[s] = b
                when(s) {
                    OptionType.DRAW -> ObstacleOptionCache.obstacleOption.draw = b
                    OptionType.ID -> ObstacleOptionCache.obstacleOption.useId = b
                    OptionType.TYPE -> ObstacleOptionCache.obstacleOption.useType = b
                    OptionType.HEADING -> ObstacleOptionCache.obstacleOption.useHeading = b
                    OptionType.POSITION -> ObstacleOptionCache.obstacleOption.usePosition = b
                    OptionType.SPEED -> ObstacleOptionCache.obstacleOption.useSpeed = b
                    OptionType.TTC -> ObstacleOptionCache.obstacleOption.useTtc = b
                    OptionType.RISK -> ObstacleOptionCache.obstacleOption.useRisk = b
                }
            }
        )

        ObstacleSettingButtons(modifier = Modifier.align(Alignment.End).padding(top = 8.dp))
    }
}

@Composable
private fun DashboardOption(
    modifier: Modifier = Modifier,
    draw: Boolean = false,
    onDrawChange: (Boolean) -> Unit
) {
    Column(modifier = modifier) {
        EvizText(text = "Dashboard Options", fontSize = 22f, fontWeight = FontWeight.Bold)
        TextWithSwitch(
            checked = draw,
            modifier = Modifier.padding(start = 16.dp),
            text = "- Drawing in Dashboard",
            onCheckedChange = onDrawChange
        )
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun ObstacleOptionEditor(
    modifier: Modifier = Modifier,
    data: Map<OptionType, Boolean>,
    onDataChange: (OptionType, Boolean) -> Unit
) {
    Column(modifier) {
        EvizText("Display Data", fontSize = 22f, fontWeight = FontWeight.Bold)
        FlowRow(
            modifier = Modifier.wrapContentSize().padding(start = 16.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            maxItemsInEachRow = 2
        ) {
            ObstacleOptionItem("ID", data[OptionType.ID]!!, onCheckedChange = { b ->
                onDataChange(OptionType.ID, b)
            })
            ObstacleOptionItem("Type", data[OptionType.TYPE]!!, onCheckedChange = { b ->
                onDataChange(OptionType.TYPE, b)
            })
            ObstacleOptionItem("Heading", data[OptionType.HEADING]!!, onCheckedChange = { b ->
                onDataChange(OptionType.HEADING, b)
            })
            ObstacleOptionItem("Position", data[OptionType.POSITION]!!, onCheckedChange = { b ->
                onDataChange(OptionType.POSITION, b)
            })
            ObstacleOptionItem("Speed", data[OptionType.SPEED]!!, onCheckedChange = { b ->
                onDataChange(OptionType.SPEED, b)
            })
            ObstacleOptionItem("TTC", data[OptionType.TTC]!!, onCheckedChange = { b ->
                onDataChange(OptionType.TTC, b)
            })
            ObstacleOptionItem("Risk", data[OptionType.RISK]!!, onCheckedChange = { b ->
                onDataChange(OptionType.RISK, b)
            })
        }
    }
}

@Composable
private fun ObstacleSettingButtons(
    modifier: Modifier = Modifier
) {
    val btnModifier = Modifier.width(200.dp).height(80.dp).padding(16.dp)

    Row(modifier) {
        Button(
            modifier = btnModifier,
            shape = RoundedCornerShape(50),
            colors = ButtonDefaults.buttonColors(backgroundColor = Orange),
            onClick = {
                LidarOption.updateData()
            }) {
            EvizText(
                "Reset",
                fontWeight = FontWeight.Bold,
                fontSize = 14f
            )
        }

        Button(
            modifier = btnModifier,
            shape = RoundedCornerShape(50),
            colors = ButtonDefaults.buttonColors(backgroundColor = Orange),
            onClick = {
                CoroutineScope(Dispatchers.Main).launch {
                    try {
                        ObstacleOptionCache.saveAndApply()
                        NotificationViewModel.updateNotification(
                            NotificationMessage("Obstacle 옵션 저장이 완료되었습니다.", true)
                        )
                    } catch (e: Exception) {
                        NotificationViewModel.updateNotification(
                            NotificationMessage("데이터 저장에 실패하였습니다. 올바른 값을 입력해주세요.", true)
                        )
                        e.printStackTrace()
                    }
                }
            }) {
            EvizText("Save & Apply", fontWeight = FontWeight.Bold, fontSize = 14f)
        }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun FlowRowScope.ObstacleOptionItem(
    name: String,
    checked: Boolean,
    onCheckedChange: (Boolean) -> Unit
) {
    EvizText("- $name", modifier = Modifier.align(Alignment.CenterVertically).padding(end = 200.dp))
    Checkbox(
        modifier = Modifier.align(Alignment.CenterVertically),
        checked = checked,
        onCheckedChange = onCheckedChange,
        colors = CheckboxDefaults.colors(
            checkedColor = Neptune,
            uncheckedColor = White_100
        )
    )
}

internal enum class OptionType {
    DRAW,
    ID,
    TYPE,
    HEADING,
    POSITION,
    SPEED,
    TTC,
    RISK
}
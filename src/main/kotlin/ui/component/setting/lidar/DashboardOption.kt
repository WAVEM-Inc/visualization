package ui.component.setting.lidar

import androidx.compose.foundation.layout.*
import androidx.compose.material.TextFieldDefaults
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import ui.component.common.*
import ui.theme.Neptune
import ui.theme.White_200
import ui.theme.colorTohex
import ui.theme.hexToColor


@Composable
fun DashboardOptionLayout(modifier: Modifier = Modifier) {
    var draw by remember { mutableStateOf(true) }
    var useDynamicColor by remember { mutableStateOf(true) }
    var farColor by remember { mutableStateOf(Color(0xff000000)) }
    var nearColor by remember { mutableStateOf(Color(0xff000000)) }
    var maxDistance by remember { mutableStateOf(0) }

    LaunchedEffect(Unit) {
        val pointCloudOption = LidarOption.pointCloudOption
        draw = pointCloudOption.draw
        useDynamicColor = pointCloudOption.useDynamicColor
        farColor = hexToColor(pointCloudOption.farColor)
        nearColor = hexToColor(pointCloudOption.nearColor)
        maxDistance = pointCloudOption.maxDistance
    }

    Column(modifier = modifier) {
        DashboardOption(
            draw = draw,
            useDynamicColor = useDynamicColor,
            onDrawingChange = { b ->
                draw = b
                LidarOption.pointCloudOption.draw = b
            },
            onDynamicColorChange = { b ->
                useDynamicColor = b
                LidarOption.pointCloudOption.useDynamicColor = b
            }
        )

        PointColorOption(
            farColor = farColor,
            nearColor = nearColor,
            maxDistance = maxDistance,
            onFarColorChange = {
                LidarOption.pointCloudOption.farColor = colorTohex(it)
                farColor = it
            },
            onNearColorChange = {
                nearColor = it
                LidarOption.pointCloudOption.nearColor = colorTohex(it)
            },
            onMaxDistanceChange = {
                maxDistance = it
                LidarOption.pointCloudOption.maxDistance = it
            }
        )
    }
}

@Composable
private fun DashboardOption(
    draw: Boolean,
    useDynamicColor: Boolean,
    onDrawingChange: (Boolean) -> Unit,
    onDynamicColorChange: (Boolean) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(modifier = modifier) {
        EvizText(text = "Dashboard Options", fontSize = 22f, fontWeight = FontWeight.Bold)
        TextWithSwitch(
            checked = draw,
            modifier = Modifier.padding(start = 16.dp),
            text = "- Drawing in Dashboard",
            onCheckedChange = onDrawingChange
        )

        TextWithSwitch(
            checked = useDynamicColor,
            modifier = Modifier.padding(start = 16.dp),
            text = "- Dynamic Color",
            onCheckedChange = onDynamicColorChange
        )
    }
}

@Composable
private fun PointColorOption(
    modifier: Modifier = Modifier,
    farColor: Color,
    nearColor: Color,
    maxDistance: Int,
    onFarColorChange: (Color) -> Unit,
    onNearColorChange: (Color) -> Unit,
    onMaxDistanceChange: (Int) -> Unit
) {
    val rowModifier = Modifier.padding(8.dp).padding(end = 32.dp)
    val etModifier = Modifier.width(150.dp).height(50.dp).padding(end = 8.dp)

    var strFarColor by mutableStateOf(colorTohex(farColor))
    var strNearColor by mutableStateOf(colorTohex(nearColor))
    var strMaxDistance by mutableStateOf(maxDistance.toString())

    Row(modifier = modifier, verticalAlignment = Alignment.CenterVertically) {
        Row(modifier = rowModifier, verticalAlignment = Alignment.CenterVertically) {
            TextWithHeader(text = "Far Color:", modifier = Modifier.padding(end = 8.dp))

            EditText(
                value = strFarColor,
                modifier = etModifier,
                colors = TextFieldDefaults.textFieldColors(
                    focusedIndicatorColor = Neptune,
                    unfocusedIndicatorColor = White_200,
                    cursorColor = White_200
                ),
                onValueChange = { s ->
                    strFarColor = s
                    try {
                        val color = hexToColor(s)
                        onFarColorChange(color)
                    } catch (_: Exception) {
                    }
                }
            )

            ColorPicker(
                modifier = Modifier.size(20.dp),
                color = farColor,
                onColorChange = onFarColorChange
            )
        }

        Row(modifier = rowModifier, verticalAlignment = Alignment.CenterVertically) {
            TextWithHeader(text = "Near Color:", modifier = Modifier.padding(end = 8.dp))

            EditText(
                value = strNearColor,
                modifier = etModifier,
                colors = TextFieldDefaults.textFieldColors(
                    focusedIndicatorColor = Neptune,
                    unfocusedIndicatorColor = White_200,
                    cursorColor = White_200
                ),
                onValueChange = { s ->
                    strNearColor = s
                    try {
                        val color = hexToColor(s)
                        onNearColorChange(color)
                    } catch (_: Exception) {
                    }
                }
            )

            ColorPicker(
                modifier = Modifier.size(20.dp),
                color = nearColor,
                onColorChange = onNearColorChange
            )
        }

        Row(modifier = rowModifier, verticalAlignment = Alignment.CenterVertically) {
            TextWithHeader(text = "LiDAR max distance", modifier = Modifier.padding(end = 8.dp))

            EditText(
                modifier = etModifier,
                value = strMaxDistance,
                colors = TextFieldDefaults.textFieldColors(
                    focusedIndicatorColor = Neptune,
                    unfocusedIndicatorColor = White_200,
                    cursorColor = White_200
                ),
                onValueChange = {
                    strMaxDistance = it
                    try {
                        onMaxDistanceChange(it.toInt())
                    } catch (_: Exception) {
                    }
                }
            )
        }
    }
}
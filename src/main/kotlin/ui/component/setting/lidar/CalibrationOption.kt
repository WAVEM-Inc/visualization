package ui.component.setting.lidar

import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import com.google.gson.Gson
import ui.component.common.EvizText
import ui.component.common.TreeRow
import ui.theme.Poppins
import ui.theme.White_100
import util.common.LidarCalibration

@Composable
fun CalibrationOptionLayout(modifier: Modifier = Modifier.fillMaxWidth()) {
    var left by remember { mutableStateOf(LidarCalibration()) }
    var front by remember { mutableStateOf(LidarCalibration()) }
    var right by remember { mutableStateOf(LidarCalibration()) }

    LaunchedEffect(Unit) {
        val gson = Gson()
        left = gson.fromJson(LidarOption.left, LidarCalibration::class.java)
        front = gson.fromJson(LidarOption.front, LidarCalibration::class.java)
        right = gson.fromJson(LidarOption.right, LidarCalibration::class.java)
    }

    Column(modifier = modifier) {
        EvizText(
            "Calibration",
            fontSize = 22f,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.padding(vertical = 16.dp).padding(top = 16.dp)
        )

        Row(modifier = Modifier.fillMaxWidth()) {
            CalibrationOption(
                modifier = Modifier.weight(1f).padding(start = 20.dp),
                data = left,
                title = "- Left",
                onValueChange = { s ->
                    LidarOption.left = s
                }
            )

            CalibrationOption(
                modifier = Modifier.weight(1f).padding(start = 20.dp),
                data = front,
                title = "- Front",
                onValueChange = { s ->
                    LidarOption.front = s
                }
            )

            CalibrationOption(
                modifier = Modifier.weight(1f).padding(start = 20.dp),
                data = right,
                title = "- Right",
                onValueChange = { s ->
                    LidarOption.right = s
                }
            )
        }
    }
}

@Composable
private fun CalibrationOption(
    modifier: Modifier = Modifier.wrapContentSize(),
    data: LidarCalibration,
    title: String,
    onValueChange: (String) -> Unit
) {
    val titleStyle = TextStyle(
        color = White_100,
        fontSize = TextUnit(16f, TextUnitType.Sp),
        fontWeight = FontWeight.Bold,
        fontFamily = Poppins
    )

    val headerStyle = TextStyle(
        color = White_100,
        fontSize = TextUnit(14f, TextUnitType.Sp),
        fontWeight = FontWeight.SemiBold,
        fontFamily = Poppins
    )

    val dataStyle = TextStyle(
        color = White_100,
        fontSize = TextUnit(12f, TextUnitType.Sp),
        fontWeight = FontWeight.Normal,
        fontFamily = Poppins,
        textAlign = TextAlign.Center
    )


    TreeRow(
        title = title,
        modifier = modifier,
        titleStyle = titleStyle,
        headerStyle = headerStyle,
        dataStyle = dataStyle,
        data = data,
        onValueChange = { jsonObject ->
            onValueChange(jsonObject.toString())
        }
    )
}
package ui.component.setting.lidar

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Button
import androidx.compose.material.ButtonDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import application.type.data.NotificationMessage
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import ui.component.common.EvizText
import ui.theme.Navy_200
import ui.theme.Orange
import viewmodel.NotificationViewModel

@Composable
fun LidarSettingLayout(
    modifier: Modifier = Modifier.background(Navy_200)
) {
    LaunchedEffect(Unit) {
        LidarOption.updateData()
    }

    Column(modifier = modifier) {
        DashboardOptionLayout(modifier = Modifier.padding(horizontal = 16.dp).padding(top = 8.dp))
        CalibrationOptionLayout(modifier = Modifier.padding(horizontal = 16.dp).padding(top = 8.dp))
        LidarSettingButtons(modifier = Modifier.align(Alignment.End))
    }
}

@Composable
private fun LidarSettingButtons(
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
                        LidarOption.saveAndApply()
                        NotificationViewModel.updateNotification(NotificationMessage("LiDAR 옵션 저장이 완료되었습니다.", true))
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
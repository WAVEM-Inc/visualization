package ui.component.setting.mobileye

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Button
import androidx.compose.material.ButtonDefaults
import androidx.compose.material.Checkbox
import androidx.compose.material.CheckboxDefaults
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import application.type.data.NotificationMessage
import application.type.option.MobileyeOption
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import ui.component.common.EvizText
import ui.theme.Navy_200
import ui.theme.Neptune
import ui.theme.Orange
import ui.theme.White_100
import viewmodel.NotificationViewModel

@OptIn(ExperimentalLayoutApi::class)
@Composable
fun MobileyeOptionLayout(modifier: Modifier = Modifier.background(Navy_200)) {
    var option by remember { mutableStateOf(MobileyeOption()) }

    LaunchedEffect(Unit) {
        MobileyeOptionCache.updateData()

        option = MobileyeOptionCache.mobileyeOption.copy()
    }

    Column(modifier = modifier) {
        Column(modifier = Modifier.padding(horizontal = 16.dp).padding(top = 8.dp).fillMaxWidth()) {
            EvizText("Display Data", fontSize = 22f, fontWeight = FontWeight.Bold)
            FlowRow(
                modifier = Modifier.wrapContentSize().padding(start = 16.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                maxItemsInEachRow = 2
            ) {
                OptionItem(name = "ID", checked = option.useId, onCheckedChange = { b ->
                    option = option.copy(useId = b)
                    MobileyeOptionCache.mobileyeOption.useId = b
                })
                OptionItem(name = "Type", checked = option.useType, onCheckedChange = { b ->
                    option = option.copy(useType = b)
                    MobileyeOptionCache.mobileyeOption.useType = b
                })
                OptionItem(name = "Heading", checked = option.useHeading, onCheckedChange = { b ->
                    option = option.copy(useHeading = b)
                    MobileyeOptionCache.mobileyeOption.useHeading = b
                })
                OptionItem(name = "Position", checked = option.usePosition, onCheckedChange = { b ->
                    option = option.copy(usePosition = b)
                    MobileyeOptionCache.mobileyeOption.usePosition = b
                })
                OptionItem(name = "Speed", checked = option.useSpeed, onCheckedChange = { b ->
                    option = option.copy(useSpeed = b)
                    MobileyeOptionCache.mobileyeOption.useSpeed = b
                })
                OptionItem(name = "TTC", checked = option.useTtc, onCheckedChange = { b ->
                    option = option.copy(useTtc = b)
                    MobileyeOptionCache.mobileyeOption.useTtc = b
                })
                OptionItem(name = "Risk", checked = option.useRisk, onCheckedChange = { b ->
                    option = option.copy(useRisk = b)
                    MobileyeOptionCache.mobileyeOption.useRisk = b
                })
                OptionItem(name = "LDW", checked = option.useLwd, onCheckedChange = { b ->
                    option = option.copy(useLwd = b)
                    MobileyeOptionCache.mobileyeOption.useLwd = b
                })
            }

            Row(modifier = Modifier.align(Alignment.End)) {
                Button(
                    modifier = Modifier.width(200.dp).height(80.dp).padding(16.dp),
                    shape = RoundedCornerShape(50),
                    colors = ButtonDefaults.buttonColors(backgroundColor = Orange),
                    onClick = {
                        MobileyeOptionCache.updateData()
                        option = MobileyeOptionCache.mobileyeOption.copy()
                    }
                ) {
                    EvizText(text = "Reset", fontWeight = FontWeight.Bold, fontSize = 14f)
                }

                Button(
                    modifier = Modifier.width(200.dp).height(80.dp).padding(16.dp),
                    shape = RoundedCornerShape(50),
                    colors = ButtonDefaults.buttonColors(backgroundColor = Orange),
                    onClick = {
                        CoroutineScope(Dispatchers.Main).launch {
                            try {
                                MobileyeOptionCache.saveAndApply()
                                NotificationViewModel.updateNotification(
                                    NotificationMessage("Mobileye 옵션 저장이 완료되었습니다.", true)
                                )
                            } catch (e: Exception) {
                                NotificationViewModel.updateNotification(
                                    NotificationMessage("데이터 저장에 실패하였습니다. 올바른 값을 입력해주세요.", true)
                                )
                                e.printStackTrace()
                            }
                        }
                    }
                ) {
                    EvizText(text = "Save & Apply", fontWeight = FontWeight.Bold, fontSize = 14f)
                }
            }
        }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun FlowRowScope.OptionItem(
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
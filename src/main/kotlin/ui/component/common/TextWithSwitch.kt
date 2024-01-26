package ui.component.common

import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.material.Switch
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import ui.theme.Poppins
import ui.theme.White_100

@Composable
fun TextWithSwitch(
    modifier: Modifier = Modifier,
    text: String,
    color: Color = White_100,
    checked: Boolean = true,
    fontFamily: FontFamily = Poppins,
    fontWeight: FontWeight = FontWeight.Normal,
    fontSize: Float = 12f,
    onCheckedChange: (isChecked: Boolean) -> Unit
) {
    Row(modifier = modifier) {
        EvizText(
            text = text,
            color = color,
            fontFamily = fontFamily,
            fontWeight = fontWeight,
            fontSize = fontSize,
            modifier = Modifier.align(Alignment.CenterVertically).padding(end = 40.dp)
        )

        Switch(
            checked = checked,
            modifier = Modifier.align(Alignment.CenterVertically),
            onCheckedChange = onCheckedChange
        )
    }
}
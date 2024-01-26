package ui.component.common

import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.width
import androidx.compose.material.Checkbox
import androidx.compose.material.CheckboxDefaults
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import ui.theme.Neptune
import ui.theme.Poppins
import ui.theme.White_100

@Composable
fun TextWithCheckBox(
    text: String,
    color: Color = White_100,
    fontFamily: FontFamily = Poppins,
    fontWeight: FontWeight = FontWeight.Normal,
    fontSize: Float = 12f,
    checked: Boolean = false,
    spaceBetween: Int = 30,
    onCheckedChange: (Boolean) -> Unit,
    modifier: Modifier = Modifier
) {
    Row (modifier = modifier) {
        EvizText(
            text = text,
            color = color,
            fontFamily = fontFamily,
            fontWeight = fontWeight,
            fontSize = fontSize
        )

        Spacer(modifier = Modifier.width(spaceBetween.dp))

        Checkbox(
            checked = checked,
            onCheckedChange = onCheckedChange,
            colors = CheckboxDefaults.colors(
                checkedColor = Neptune,
                uncheckedColor = White_100
            )
        )
    }
}
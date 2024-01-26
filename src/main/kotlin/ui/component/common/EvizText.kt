package ui.component.common

import androidx.compose.material.LocalTextStyle
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import ui.theme.Poppins
import ui.theme.White_100


@Composable
fun EvizText(
    text: String,
    color: Color = White_100,
    fontFamily: FontFamily = Poppins,
    fontWeight: FontWeight = FontWeight.Normal,
    fontSize: Float = 12f,
    modifier: Modifier = Modifier,
    style: TextStyle = LocalTextStyle.current
) {
    Text(
        text = text,
        color = color,
        fontFamily = fontFamily,
        fontWeight = fontWeight,
        fontSize = TextUnit(fontSize, TextUnitType.Sp),
        modifier = modifier,
        style = style
    )
}
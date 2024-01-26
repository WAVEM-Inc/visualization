package ui.component.common

import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.DpSize
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import ui.theme.Poppins
import ui.theme.White_100

@Composable
fun TextWithHeaderContent(
    modifier: Modifier = Modifier,
    text: String,
    content: String = "NaN",
    headerSize: DpSize = DpSize(3.dp, 3.dp),
    textSize: TextUnit = TextUnit(13f, TextUnitType.Sp)
) {
    Row(modifier = modifier) {
        Image(
            painter = painterResource("icon/ic_header.svg"),
            contentDescription = null,
            modifier = Modifier.size(headerSize).align(Alignment.CenterVertically)
        )

        Spacer(modifier = Modifier.width(6.dp))

        Text(
            text = "$text: ",
            fontSize = textSize,
            fontFamily = Poppins,
            fontWeight = FontWeight.Normal,
            color = White_100
        )

        Text(
            text = content,
            fontSize = textSize,
            fontFamily = Poppins,
            fontWeight = FontWeight.Normal,
            color = White_100
        )
    }
}

@Composable
fun TextWithHeader(
    modifier: Modifier = Modifier,
    text: String,
    color: Color = White_100,
    fontFamily: FontFamily = Poppins,
    fontWeight: FontWeight = FontWeight.Normal,
    fontSize: Float = 12f,
    headerSize: DpSize = DpSize(3.dp, 3.dp)
) {
    Row(modifier = modifier) {
        Image(
            painter = painterResource("icon/ic_header.svg"),
            contentDescription = null,
            modifier = Modifier.size(headerSize).align(Alignment.CenterVertically)
        )

        Spacer(modifier = Modifier.width(6.dp))

        EvizText(
            text = text,
            color = color,
            fontFamily = fontFamily,
            fontWeight = fontWeight,
            fontSize = fontSize
        )
    }
}
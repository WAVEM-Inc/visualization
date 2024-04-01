package ui.theme

import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.toArgb

val Black = Color(0xff010101)

val Black_100 = Color(0xff2B2C36)

val Navy_100 = Color(0xff131523)

val Navy_200 = Color(0xff313346)

val Gray = Color(0xff6A7070)

val DarkGray = Color(0xff3C4242)

val LightGray = Color(0xff7E84A3)

val White_100 = Color(0xffD7DBEC)

val White_200 = Color(0xffE6E9F4)

val Orange = Color(0xffFF6140)

val Yellow = Color(0xffFFE272)

val Red = Color(0xffFF3737)

val Green = Color(0xff74DC2E)

val Neptune = Color(0xff57B8FF)

fun hexToColor(htmlColor: String): Color {
    val color = htmlColor.substring(1) // Remove the '#' character
    val r = Integer.parseInt(color.substring(0, 2), 16) / 255.0f
    val g = Integer.parseInt(color.substring(2, 4), 16) / 255.0f
    val b = Integer.parseInt(color.substring(4, 6), 16) / 255.0f

    return Color(r, g, b)
}

fun colorTohex(color: Color): String {
    return String.format("#%06x", color.toArgb() and 0xFFFFFF)
}
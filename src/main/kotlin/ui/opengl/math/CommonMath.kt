package ui.opengl.math

import kotlin.math.round

fun Double.round(num: Int): Double {
    return round(this * num) / num
}
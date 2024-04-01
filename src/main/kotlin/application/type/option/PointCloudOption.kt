package application.type.option

import ui.theme.Green
import ui.theme.Red
import ui.theme.colorTohex

data class PointCloudOption(
    var draw: Boolean = true,
    var useDynamicColor: Boolean = true,
    var farColor: String = colorTohex(Green),
    var nearColor: String = colorTohex(Red),
    var maxDistance: Int = 30
)

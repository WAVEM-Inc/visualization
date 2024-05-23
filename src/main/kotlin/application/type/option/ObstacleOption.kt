package application.type.option

import ui.theme.White_100
import ui.theme.colorTohex

data class ObstacleOption(
    var draw:Boolean = true,
    var useId: Boolean = true,
    var useType: Boolean = true,
    var useHeading: Boolean = true,
    var usePosition: Boolean = true,
    var useSpeed: Boolean = true,
    var useTtc: Boolean = true,
    var useRisk: Boolean = true,
    var textColor: String = colorTohex(White_100),
    var textSize: Int = 10
)

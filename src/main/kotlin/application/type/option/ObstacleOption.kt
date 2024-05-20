package application.type.option

data class ObstacleOption(
    var draw:Boolean = true,
    var useId: Boolean = true,
    var useType: Boolean = true,
    var useHeading: Boolean = true,
    var usePosition: Boolean = true,
    var useSpeed: Boolean = true,
    var useTtc: Boolean = true,
    var useRisk: Boolean = true
)

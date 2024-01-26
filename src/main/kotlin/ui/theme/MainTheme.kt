package ui.theme

import androidx.compose.material.Colors
import androidx.compose.ui.graphics.Color

object MainTheme {
    private lateinit var colors: Colors

    init {
        colors = Colors(
            primary = Color.Red,
            primaryVariant = Color.Red,
            secondary = Color.Red,
            secondaryVariant = Color.Red,
            background = Color.Red,
            surface = Color.Red,
            error = Color.Red,
            onPrimary = Color.Red,
            onSecondary = Color.Red,
            onBackground = Color.Red,
            onSurface = Color.Red,
            onError = Color.Red,
            isLight = true
        )
    }

    fun setMode() {

    }

    fun customizeColors(colors: Colors) {

    }
}
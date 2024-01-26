package ui.theme

import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.platform.Font

object Fonts {
    fun getGmarketSans(): FontFamily {
        val bold = Font("font/GmarketSansTTFBold.ttf", weight = FontWeight.Bold)
        val right = Font("font/GmarketSansTTFLight.ttf", weight = FontWeight.Light)
        val medium = Font("font/GmarketSansTTFMedium.ttf", weight = FontWeight.Medium)
        return FontFamily(listOf(bold, right, medium))
    }
}
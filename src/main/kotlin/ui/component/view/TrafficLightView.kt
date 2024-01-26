package ui.component.view

import androidx.compose.foundation.*
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material.Switch
import androidx.compose.material.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.center
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Fill
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import kotlin.math.min

@Composable
fun TrafficLightView(modifier: Modifier = Modifier.fillMaxSize()) {
    val checkedState = remember { mutableStateOf(true) }
    Column {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(
                text = "Front Camera Traffic Light Detection",
                modifier = Modifier.weight(1f).padding(start = 10.dp),
                fontWeight = FontWeight.Bold
            )
            Switch(checked = checkedState.value, onCheckedChange = { isChecked ->
                checkedState.value = isChecked
            })
        }
        Row(modifier = modifier) {
            LightCircle(
                color = Color.Red,
                modifier = Modifier.fillMaxSize().padding(10.dp).weight(1f)
            )
            LightCircle(
                color = Color.Yellow,
                modifier = Modifier.fillMaxSize().padding(10.dp).weight(1f)
            )
            LightCircle(
                color = Color.Green,
                modifier = Modifier.fillMaxSize().padding(10.dp).weight(1f)
            )
            LightArrow(modifier = Modifier.fillMaxSize().padding(10.dp).weight(1f))
        }
    }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun LightCircle(
    modifier: Modifier = Modifier.fillMaxSize().padding(10.dp),
    color: Color,
    isLight: Boolean = false
) {
    var (alpha, setAlpha) = remember { mutableStateOf(0.5f) }

    Canvas(modifier.alpha(alpha).onClick {
        println("Traffic Light View: is clicked")
        if (alpha == 0.8f) {
            setAlpha(0.5f)
        } else {
            setAlpha(0.8f)
        }
    }) {
        val width = size.width
        val height = size.height
        val stroke = width * 0.0390625f
        val radius = min(width, height) / 2

        val brush = Brush.radialGradient(
            colors = listOf(Color.White, color),
            center = Offset(width * 0.3f, height * 0.3f),
            radius = radius * 0.5f
        )
        drawCircle(brush = brush, center = size.center, radius = radius, style = Fill)
        drawCircle(color = Color.Black, center = size.center, radius = radius - stroke / 2, style = Stroke(stroke))
    }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun LightArrow(
    modifier: Modifier = Modifier.fillMaxSize().padding(10.dp),
    isLight: Boolean = false
) {
    val (alpha, setAlpha) = remember { mutableStateOf(0.5f) }

    Image(
        painter = painterResource("icon/ic_left_arrow.svg"),
        contentDescription = null,
        alpha = alpha,
        modifier = modifier.onClick {
            if (alpha == 0.8f) {
                setAlpha(0.5f)
            } else {
                setAlpha(0.8f)
            }
        }
    )
}
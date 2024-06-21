package ui.layout

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.drawscope.scale
import androidx.compose.ui.graphics.drawscope.translate
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Density
import androidx.compose.ui.unit.LayoutDirection
import androidx.compose.ui.unit.dp
import apollo.perception.PerceptionObstacleOuterClass
import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacle
import apollo.perception.perceptionObstacle
import application.type.data.Position
import application.type.option.MobileyeOption
import essys_middle.Mobileye.MobileyeData
import essys_middle.copy
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import org.jetbrains.skia.Paint
import viewmodel.ConfigDataViewModel
import viewmodel.UdpDataViewModel
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

@Composable
fun MobileyeLayout(modifier: Modifier = Modifier.fillMaxSize()) {
    val textMeasurer = rememberTextMeasurer()
    val textStyle = TextStyle(color = Color.White)
    val ldwShape = HexagonShape()

    var objects by remember { mutableStateOf(listOf<PerceptionObstacle>()) }
//    var mobileyeData by remember { mutableStateOf(MobileyeData.getDefaultInstance()) }
    var drawOption by remember { mutableStateOf(MobileyeOption()) }
    val currentLocale = remember { mutableStateOf(Position()) }

    var leftLdwColor by remember { mutableStateOf(Color.Gray) }
    var rightLdwColor by remember { mutableStateOf(Color.Gray) }

    LaunchedEffect(Unit) {
        val currentLocaleState = currentLocale
        CoroutineScope(Dispatchers.Default).launch {
            UdpDataViewModel.subscribeLocalization(collector = { locale ->
                currentLocale.value.x = locale.pose.position.x
                currentLocale.value.y = locale.pose.position.y
            })
        }

        CoroutineScope(Dispatchers.Default).launch {
            UdpDataViewModel.subscribeMobileye(collector = { data ->
                objects = data.perceptionObstacleList

                data.perceptionObstacleList
                if (drawOption.useLwd) {
                    rightLdwColor = if (data.rightLdwAvailability.number == 0) {
                        Color.Red
                    } else {
                        Color.Green
                    }

                    leftLdwColor = if (data.leftLdwAvailability.number == 0) {
                        Color.Red
                    } else {
                        Color.Green
                    }
                }
            })
        }

        CoroutineScope(Dispatchers.Default).launch {
            ConfigDataViewModel.subscribeMobileyeOption(collector = { option ->
                drawOption = option
            })
        }
    }

    Box(modifier = modifier, contentAlignment = Alignment.BottomCenter) {
        Canvas(modifier = Modifier.fillMaxSize()) {
            val width = size.width
            val height = size.height

            val originX = 0
            val originY = 25
            val meterPx = (height - originY) / 50

            translate(left = width / 2) {
                scale(scaleX = 1f, scaleY = -1f) {
                    val dashedEffect = PathEffect.dashPathEffect(floatArrayOf(10f, 10f), 0f)

                    drawLine(
                        color = Color.White,
                        start = Offset(0f, 0f),
                        end = Offset(0f, height),
                        pathEffect = dashedEffect
                    )

                    drawLine(
                        color = Color.White,
                        start = Offset(-width / 2, originY.toFloat()),
                        end = Offset(width / 2, originY.toFloat()),
                        pathEffect = dashedEffect
                    )

                    // draw meter texts
                    for (i in 10..50 step 10) {
                        val x = 0f
                        val y = originY + i * meterPx
                        rotate(180f, pivot = Offset(x, y)) {
                            scale(scaleX = -1f, scaleY = 1f, pivot = Offset(x, y)) {
                                drawText(
                                    textMeasurer = textMeasurer,
                                    text = "${i}m",
                                    topLeft = Offset(x.dp.toPx(), y.dp.toPx()),
                                    style = textStyle
                                )
                            }
                        }
                    }

                    for (obstacle in objects) {
                        var x = (originX + (obstacle.position.x - currentLocale.value.x) * meterPx).toFloat()
                        var y = (originY + (obstacle.position.y - currentLocale.value.y) * meterPx).toFloat()

                        drawCircle(color = Color.Red, center = Offset(x, y), radius = 15f)

                        var text = ""
                        if (drawOption.useId) {
                            text += "ID: ${obstacle.id}"
                        }
                        if (drawOption.useType) {
                            text += "\nType: ${obstacle.type}"
                        }
                        if (drawOption.useHeading) {
                            text += "\nHeading: %.2f".format(obstacle.theta)
                        }
                        if (drawOption.usePosition) {
                            text += "\nPosition:"
                            text += "\n   -x: %.2f".format(obstacle.position.x)
                            text += "\n   -y: %.2f".format(obstacle.position.y)
                        }
                        if (drawOption.useSpeed) {
                            val speed = sqrt(obstacle.velocity.x * obstacle.velocity.x +
                                    obstacle.velocity.y * obstacle.velocity.x +
                                    obstacle.velocity.z * obstacle.velocity.z) / 1000

                            text += "\nSpeed: %.2f".format(speed)
                        }
                        if (drawOption.useTtc) {
                            text += "\nTTC: ${obstacle.ttc}"
                        }
                        if (drawOption.useRisk) {
                            text += "\nRisk: ${obstacle.riskLevel}"
                        }
                        x += 20 //adjust x for text x coordinates. add circle radius + 5
                        rotate(180f, pivot = Offset(x, y)) {
                            scale(scaleX = -1f, scaleY = 1f, pivot = Offset(x, y)) {
                                try {
                                    drawText(
                                        textMeasurer = textMeasurer,
                                        text = text,
                                        topLeft = Offset(x.dp.toPx(), y.dp.toPx()),
                                        style = textStyle
                                    )
                                } catch (_: Exception) {

                                }
                            }
                        }
                    }
                }
            }
        }

        Row(modifier = Modifier.wrapContentSize()) {
            Box(
                modifier = Modifier.background(color = leftLdwColor, shape = ldwShape).size(50.dp, 50.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    "LDW",
                    textAlign = TextAlign.Center,
                    style = TextStyle(color = Color.White, fontWeight = FontWeight.Bold)
                )
            }

            Image(
                painter = painterResource("icon/ic_top_view_car.svg"),
                contentDescription = null,
                contentScale = ContentScale.Fit,
                modifier = Modifier.size(100.dp, 50.dp).rotate(270f)
            )

            Box(
                modifier = Modifier.background(color = rightLdwColor, shape = ldwShape).size(50.dp, 50.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    "LDW",
                    textAlign = TextAlign.Center,
                    style = TextStyle(color = Color.White, fontWeight = FontWeight.Bold)
                )
            }
        }

    }
}


class HexagonShape : Shape {
    override fun createOutline(size: Size, layoutDirection: LayoutDirection, density: Density): Outline {
        val path = Path().apply {
            val radius = size.width.coerceAtMost(size.height) / 2
            val centerX = size.width / 2
            val centerY = size.height / 2
            val angle = (Math.PI / 3).toFloat() // 60 degrees in radians

            moveTo(centerX + radius * cos(0f), centerY + radius * sin(0f))
            for (i in 1..5) {
                val x = centerX + radius * cos(angle * i)
                val y = centerY + radius * sin(angle * i)
                lineTo(x, y)
            }
            close()
        }
        return Outline.Generic(path)
    }
}
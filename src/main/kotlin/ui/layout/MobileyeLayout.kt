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
import apollo.perception.perceptionObstacle
import application.type.data.Position
import essys_middle.Mobileye.MobileyeData
import essys_middle.copy
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import org.jetbrains.skia.Paint
import viewmodel.UdpDataViewModel
import kotlin.math.cos
import kotlin.math.sin

@Composable
fun MobileyeLayout(modifier: Modifier = Modifier.fillMaxSize()) {
    val textMeasurer = rememberTextMeasurer()
    val textStyle = TextStyle(color = Color.White)
    val ldwShape = HexagonShape()

    var mobileyeData by remember { mutableStateOf(MobileyeData.getDefaultInstance()) }
    val currentLocale = Position()

    LaunchedEffect(Unit) {
        CoroutineScope(Dispatchers.Default).launch {
            UdpDataViewModel.subscribeLocalization(collector = { locale ->
                currentLocale.x = locale.pose.position.x
                currentLocale.y = locale.pose.position.y
            })
        }

        CoroutineScope(Dispatchers.Default).launch {
            UdpDataViewModel.subscribeMobileye(collector = { data ->
                mobileyeData = data
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

            println(height)
            println(meterPx)

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

//                    drawCircle(color = Color.Red, center = Offset(100f, 100f), radius = 15f)
//                    rotate(180f, pivot = Offset(120f, 100f)) {
//                        scale(scaleX = -1f, scaleY = 1f, pivot = Offset(120f, 100f)) {
//                            drawText(
//                                textMeasurer = textMeasurer,
//                                text = "Obstacle 정보\n가\n나\n다\n라\n마\n바",
//                                topLeft = Offset(120.dp.toPx(), 100.dp.toPx()),
//                                style = textStyle
//                            )
//                        }
//                    }
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

                    for (obstacle in mobileyeData.perceptionObstacleList) {
                        var x = (originX + (obstacle.position.x - currentLocale.x) * meterPx).toFloat()
                        var y = (originY + (obstacle.position.y - currentLocale.y) * meterPx).toFloat()

                        drawCircle(color = Color.Red, center = Offset(x, y), radius = 15f)

                        x += 20 //adjust x for text x coordinates. add circle radius + 5
                        rotate(180f, pivot = Offset(x, y)) {
                            scale(scaleX = -1f, scaleY = 1f, pivot = Offset(x, y)) {
                                drawText(
                                    textMeasurer = textMeasurer,
                                    text = "Obstacle 정보",
                                    topLeft = Offset(x.dp.toPx(), y.dp.toPx()),
                                    style = textStyle
                                )
                            }
                        }
                    }
                }
            }
        }

        Row(modifier = Modifier.wrapContentSize()) {
            Box(
                modifier = Modifier.background(color = Color.Red, shape = ldwShape).size(50.dp, 50.dp),
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
                modifier = Modifier.background(color = Color.Red, shape = ldwShape).size(50.dp, 50.dp),
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
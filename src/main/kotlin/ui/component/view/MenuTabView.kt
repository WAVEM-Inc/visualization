package ui.component.view

import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.*
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Color.Companion.Black
import androidx.compose.ui.graphics.Color.Companion.White
import androidx.compose.ui.layout.LayoutCoordinates
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.layout.positionInParent
import androidx.compose.ui.layout.positionInWindow
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.plus


@Composable
fun MultiColumnMenuTabView(
    selectedItemIndex: Int,
    items: List<String>,
    modifier: Modifier = Modifier,
    tabWidth: Dp = 100.dp,
    onClick: (index: Int) -> Unit,
    columnNum: Int = 1
) {
    Box(
        modifier = modifier
            .height(intrinsicSize = IntrinsicSize.Min)
            .border(2.dp, Black)
    ) {
        Column(modifier = Modifier.fillMaxSize()) {
            for (i in 1..columnNum) {
                val startIndex = items.size / (columnNum) * (i - 1)
                val endIndex = (items.size / (columnNum) * i) - 1

                Row(
                    horizontalArrangement = Arrangement.Center,
                    modifier = Modifier
                ) {
                    for (index in startIndex..endIndex) {
                        val isSelected = index == selectedItemIndex
                        val color = if (isSelected) {
                            Color(0xff008CFF)
                        } else {
                            Color(0xffD2E0FB)
                        }

                        MenuTabItem(
                            isSelected = isSelected,
                            onClick = {
                                onClick(index)
                            },
                            tabWidth = tabWidth,
                            text = items[index],
                            modifier = Modifier.background(color = color).border(1.dp, Black)
                        )
                    }
                }
            }
        }
    }
}

@Composable
fun MenuTabView(
    selectedItemIndex: Int,
    items: List<String>,
    modifier: Modifier = Modifier,
    tabWidth: Dp = 100.dp,
    onClick: (index: Int) -> Unit,
) {
    Box(
        modifier = modifier
            .background(White)
            .height(intrinsicSize = IntrinsicSize.Min),
    ) {
        Row(
            horizontalArrangement = Arrangement.Center,
            modifier = Modifier
        ) {
            items.mapIndexed { index, text ->
                val isSelected = index == selectedItemIndex

                MenuTabItem(
                    isSelected = isSelected,
                    onClick = {
                        onClick(index)
                    },
                    tabWidth = tabWidth,
                    text = text,
                    modifier = Modifier.background(
                        color = if (isSelected) {
                            Color(0xff008CFF)
                        } else {
                            Color(0xffD2E0FB)
//                            Color.White
                        }
                    ).border(2.dp, Black)
                )
            }
        }
    }
}

@Composable
private fun MenuTabItem(
    isSelected: Boolean,
    onClick: () -> Unit,
    tabWidth: Dp,
    text: String,
    modifier: Modifier = Modifier
) {
    val tabTextColor: Color by animateColorAsState(
        targetValue = if (isSelected) {
            White
        } else {
            Black
        },
        animationSpec = tween(easing = LinearEasing),
    )
    Text(
        modifier = modifier
            .clickable {
                onClick()
            }
            .width(tabWidth)
            .padding(
                vertical = 8.dp,
                horizontal = 12.dp,
            ),
        text = text,
        color = tabTextColor,
        textAlign = TextAlign.Center
    )
}
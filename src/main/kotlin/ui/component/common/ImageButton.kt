package ui.component.common

import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material.IconButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource

@Composable
fun ImageButton(
    imgPath: String,
    modifier: Modifier = Modifier,
    onClick: () -> Unit,
    contentScale: ContentScale = ContentScale.Fit
) {
    IconButton(onClick = { onClick() }, modifier = modifier) {
        Image(
            painter = painterResource(imgPath),
            contentDescription = null,
            contentScale = contentScale
        )
    }
}
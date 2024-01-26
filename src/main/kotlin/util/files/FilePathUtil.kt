package util.files

import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.res.ResourceLoader
import java.io.InputStream
import java.net.URL

fun getDefaultCachePath(): String {
    return System.getProperty("compose.application.resources.dir")
}

@OptIn(ExperimentalComposeUiApi::class)
fun loadResource(path: String): InputStream {
    return ResourceLoader.Default.load(path)
}
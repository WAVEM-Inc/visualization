package utils

import androidx.compose.ui.ExperimentalComposeUiApi
import androidx.compose.ui.res.ResourceLoader
import java.io.File
import java.io.InputStream

fun getDefaultCachePath(): String {
    val appName = "Eviz"
    val appDataPath = "${System.getenv("AppData")}${File.separator}$appName"

    File(appDataPath).mkdirs()

    return appDataPath
}

@OptIn(ExperimentalComposeUiApi::class)
fun loadResource(path: String): InputStream {
    return ResourceLoader.Default.load(path)
}
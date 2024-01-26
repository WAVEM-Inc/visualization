package application

import application.config.ObstacleOption
import application.config.PointCloudOption
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.KotlinModule
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import util.files.getDefaultCachePath
import viewmodel.config.ObstacleViewModel
import viewmodel.config.PointCloudViewModel
import java.io.File

object EvizConfigManager {
    private val SEPARATOR: String = File.separator
    val POINT_CLOUD_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}pcl_option.yaml"
    val OBSTACLE_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}obstacle_option.yaml"
    val MAP_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}map_option.yaml"


    init {
        CoroutineScope(Dispatchers.IO).launch {
            val pclConfig: PointCloudOption = loadFile(POINT_CLOUD_CONFIG_PATH, PointCloudOption())
            PointCloudViewModel.updatePointCloudOption(pclConfig)

            val obstacleConfig: ObstacleOption = loadFile(OBSTACLE_CONFIG_PATH, ObstacleOption())
            ObstacleViewModel.updateObstacleOption(obstacleConfig)
        }
    }

    fun<T : Any> saveFile(path: String, data: T) {
        val file = File(getDefaultCachePath(), path)

        val objectMapper = ObjectMapper(YAMLFactory()).registerModule(KotlinModule())
        File(getDirectoryPath(file.absolutePath)).mkdirs()
        objectMapper.writeValue(file, data)

        println("Save File: ${file.absolutePath}")
    }

    fun<T : Any> loadFile(path: String, defaultData: T): T {
        val file = File(getDefaultCachePath(), path)

        val objectMapper = ObjectMapper(YAMLFactory()).registerModule(KotlinModule())

        val result: T
        val fileExists = file.exists()

        if (fileExists) {
            result = objectMapper.readValue(file, defaultData.javaClass) ?: defaultData
        } else {
            saveFile(path, defaultData)
            result = defaultData
        }

        return result
    }

    private fun getDirectoryPath(path: String): String {
        val index = path.lastIndexOf(File.separator)
        return path.substring(0, index)
    }
}
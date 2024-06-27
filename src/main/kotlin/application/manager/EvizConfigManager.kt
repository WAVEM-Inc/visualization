package application.type

import application.type.option.ConnectOption
import application.type.option.MobileyeOption
import application.type.option.ObstacleOption
import application.type.option.PointCloudOption
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.KotlinModule
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import utils.getDefaultCachePath
import viewmodel.ConfigDataViewModel
import java.io.File

object EvizConfigManager {
    private val SEPARATOR: String = File.separator
    val POINT_CLOUD_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}pcl_option.yaml"
    val OBSTACLE_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}obstacle_option.yaml"
    val CONNECT_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}connect_option.yaml"
    val MOBILEYE_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}mobileye_option.yaml"
    val MAP_CONFIG_PATH = "${SEPARATOR}config${SEPARATOR}map_option.yaml"

    init {
        CoroutineScope(Dispatchers.IO).launch {
            val pclConfig: PointCloudOption = loadFile(POINT_CLOUD_CONFIG_PATH, PointCloudOption())
            ConfigDataViewModel.updatePointCloudOption(pclConfig)

            val obstacleConfig: ObstacleOption = loadFile(OBSTACLE_CONFIG_PATH, ObstacleOption())
            ConfigDataViewModel.updateObstacleOption(obstacleConfig)

            val connectOption: ConnectOption = loadFile(CONNECT_CONFIG_PATH, ConnectOption("127.0.0.1", 40000, 40001, 40002));
            ConfigDataViewModel.updateConnectOption(connectOption)

            val mobileyeOption: MobileyeOption = loadFile(MOBILEYE_CONFIG_PATH, MobileyeOption())
            ConfigDataViewModel.updateMobileyeOption(mobileyeOption)
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
package viewmodel.config

import application.EvizConfigManager
import application.config.ObstacleOption
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.launch

object ObstacleViewModel {
    private val obstacleOption = MutableSharedFlow<ObstacleOption>(replay = 1)
    private var obstacleOptionValue = ObstacleOption()

    init {
        CoroutineScope(Dispatchers.Main).launch {
            obstacleOption.collect { option ->
                obstacleOptionValue = option
            }
        }
    }

    fun updateObstacleOption(option: ObstacleOption) {
        CoroutineScope(Dispatchers.Main).launch {
            obstacleOption.emit(option)
        }
    }

    fun subscribeObstacleOption(collector: FlowCollector<ObstacleOption>) {
        CoroutineScope(Dispatchers.Main).launch {
            obstacleOption.collect(collector)
        }
    }

    fun getObstacleOption(): ObstacleOption {
        return obstacleOptionValue
    }

    fun saveAndApplyObstacleOption(option: ObstacleOption) {
        CoroutineScope(Dispatchers.Main).launch {
            obstacleOption.emit(option)
        }

        CoroutineScope(Dispatchers.Main).launch {
            EvizConfigManager.saveFile(EvizConfigManager.OBSTACLE_CONFIG_PATH, option)
        }
    }
}
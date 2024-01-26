package viewmodel.config

import application.EvizConfigManager
import application.config.PointCloudOption
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.launch

object PointCloudViewModel {
    private val _pointCloudOption = MutableSharedFlow<PointCloudOption>(replay = 1)
    private val pointCloudOption = _pointCloudOption.asSharedFlow()

    private var valueOfPointCloudOption = PointCloudOption()

    init {
        CoroutineScope(Dispatchers.Main).launch {
            pointCloudOption.collect { data ->
                valueOfPointCloudOption  = data
            }
        }
    }

    suspend fun updatePointCloudOption(option: PointCloudOption) {
            _pointCloudOption.emit(option)
    }

    suspend fun subscribePointCloudOption(collector: FlowCollector<PointCloudOption>) {
            _pointCloudOption.collect(collector)
    }

    fun getPointCloudOption(): PointCloudOption {
        return valueOfPointCloudOption
    }

    fun saveAndApplyPointCloudOption(option: PointCloudOption) {
        CoroutineScope(Dispatchers.Main).launch {
            _pointCloudOption.emit(option)
        }

        CoroutineScope(Dispatchers.IO).launch {
            EvizConfigManager.saveFile(EvizConfigManager.POINT_CLOUD_CONFIG_PATH, option)
        }
    }
}
package viewmodel

import apollo.localization.Localization.LocalizationEstimate
import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacle
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.launch
import ui.opengl.math.Coordinate3D
import ui.opengl.math.Polygon
import util.common.ObjectInfo
import kotlin.coroutines.CoroutineContext
import kotlin.coroutines.coroutineContext

object TestViewModel {
    private val pointCloudFlow = MutableSharedFlow<List<Coordinate3D>>()
    private val obstacleFlow = MutableSharedFlow<List<PerceptionObstacle>>()
    private val localizationFlow = MutableSharedFlow<LocalizationEstimate>()
    private val dummyFlow = MutableSharedFlow<Map<Int, List<String>>>()

    suspend fun updatePointCloud(value: List<Coordinate3D>) {
        pointCloudFlow.emit(value)
    }

    suspend fun subscribePointCloud(collector: FlowCollector<List<Coordinate3D>>) {
        pointCloudFlow.collect(collector)
    }

    suspend fun updateObstacle(value: List<PerceptionObstacle>) {
        obstacleFlow.emit(value)
    }

    suspend fun subscribeObstacle(collector: FlowCollector<List<PerceptionObstacle>>) {
        obstacleFlow.collect(collector)
    }

    suspend fun updateLocalization(value: LocalizationEstimate) {
        localizationFlow.emit(value)
    }

    suspend fun subscribeLocalization(collector: FlowCollector<LocalizationEstimate>) {
        localizationFlow.collect(collector)
    }

    suspend fun subscribeDummy(collector: FlowCollector<Map<Int, List<String>>>) {
            dummyFlow.collect(collector)
    }

    suspend fun updateDummy(value: Map<Int, List<String>>) {
        dummyFlow.emit(value)
    }
}
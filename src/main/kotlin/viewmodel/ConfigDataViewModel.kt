package viewmodel

import application.type.EvizConfigManager
import application.type.option.ConnectOption
import application.type.option.MobileyeOption
import application.type.option.ObstacleOption
import application.type.option.PointCloudOption
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.lastOrNull

object ConfigDataViewModel {
    private val connectOptionFlow = MutableSharedFlow<ConnectOption>(replay = 1)
    private val obstacleOptionFlow = MutableSharedFlow<ObstacleOption>(replay = 1)
    private val pointCloudOptionFlow = MutableSharedFlow<PointCloudOption>(replay = 1)
    private val mobileyeOptionFlow = MutableSharedFlow<MobileyeOption>(replay = 1)


    suspend fun updateConnectOption(option: ConnectOption) {
        connectOptionFlow.emit(option)
    }

    suspend fun subscribeConnectOption(collector: FlowCollector<ConnectOption>) {
        connectOptionFlow.collect(collector)
    }

    suspend fun getConnectOption(): ConnectOption? {
        return connectOptionFlow.lastOrNull()
    }

    suspend fun updateObstacleOption(option: ObstacleOption) {
        obstacleOptionFlow.emit(option)
    }

    suspend fun subscribeObstacleOption(collector: FlowCollector<ObstacleOption>) {
        obstacleOptionFlow.collect(collector)
    }

    suspend fun getObstacleOption(): ObstacleOption? {
        return obstacleOptionFlow.lastOrNull()
    }

    suspend fun saveAndApplyObstacleOption(option: ObstacleOption) {
        obstacleOptionFlow.emit(option)
        EvizConfigManager.saveFile(EvizConfigManager.OBSTACLE_CONFIG_PATH, option)
    }

    suspend fun updatePointCloudOption(option: PointCloudOption) {
        pointCloudOptionFlow.emit(option)
    }

    suspend fun subscribePointCloudOption(collector: FlowCollector<PointCloudOption>) {
        pointCloudOptionFlow.collect(collector)
    }

    suspend fun getPointCloudOption(): PointCloudOption? {
        return pointCloudOptionFlow.lastOrNull()
    }

    suspend fun saveAndApplyPointCloudOption(option: PointCloudOption) {
        pointCloudOptionFlow.emit(option)
        EvizConfigManager.saveFile(EvizConfigManager.POINT_CLOUD_CONFIG_PATH, option)
    }

    suspend fun updateMobileyeOption(option: MobileyeOption) {
        mobileyeOptionFlow.emit(option)
    }

    suspend fun subscribeMobileyeOption(collector: FlowCollector<MobileyeOption>) {
        mobileyeOptionFlow.collect(collector)
    }

    suspend fun getMobileyeOption(): MobileyeOption? {
        return mobileyeOptionFlow.lastOrNull()
    }

    suspend fun saveAndApplyMobileyeOption(option: MobileyeOption) {
        mobileyeOptionFlow.emit(option)
        EvizConfigManager.saveFile(EvizConfigManager.MOBILEYE_CONFIG_PATH, option)
    }
}
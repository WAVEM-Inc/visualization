package viewmodel

import apollo.dreamview.PointCloudOuterClass.PointCloud
import apollo.localization.Localization.LocalizationEstimate
import apollo.perception.PerceptionObstacleOuterClass
import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacles
import essys_middle.Dashboard.TrafficLight
import essys_middle.Dashboard.VehicleSignal
import essys_middle.Mobileye.MobileyeData
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.lastOrNull

object UdpDataViewModel {
    private val pointCloudFlow = MutableSharedFlow<PointCloud>()
    private val obstaclesFlow = MutableSharedFlow<PerceptionObstacles>()
    private val localizationFlow = MutableSharedFlow<LocalizationEstimate>()
    private val vehicleSignalFlow = MutableSharedFlow<VehicleSignal>()
    private val trafficLightFlow = MutableSharedFlow<TrafficLight>()
    private val mobileyeFlow = MutableSharedFlow<MobileyeData>()

    // Point Cloud Data
    suspend fun updatePointCloud(value: PointCloud) {
        pointCloudFlow.emit(value)
    }

    suspend fun subscribePointCloud(collector: FlowCollector<PointCloud>) {
        pointCloudFlow.collect(collector)
    }

    suspend fun getPointCloud(): PointCloud? {
        return pointCloudFlow.lastOrNull()
    }

    // Obstacles Data
    suspend fun updateObstacles(value: PerceptionObstacles) {
        obstaclesFlow.emit(value)
    }

    suspend fun subscribeObstacles(collector: FlowCollector<PerceptionObstacles>) {
        obstaclesFlow.collect(collector)
    }

    suspend fun getObstacles(): PerceptionObstacles? {
        return obstaclesFlow.lastOrNull()
    }

    // Localization Data
    suspend fun updateLocalization(value: LocalizationEstimate) {
        localizationFlow.emit(value)
    }

    suspend fun subscribeLocalization(collector: FlowCollector<LocalizationEstimate>) {
        localizationFlow.collect(collector)
    }

    suspend fun getLocalization(): LocalizationEstimate? {
        return localizationFlow.lastOrNull()
    }

    // Vehicle Signal Data
    suspend fun updateVehicleSignal(value: VehicleSignal){
        vehicleSignalFlow.emit(value)
    }

    suspend fun subscribeVehicleSignal(collector: FlowCollector<VehicleSignal>) {
        vehicleSignalFlow.collect(collector)
    }

    suspend fun getVehicleSignal(): VehicleSignal? {
        return vehicleSignalFlow.lastOrNull()
    }

    // Traffic Light Data
    suspend fun updateTrafficLight(value: TrafficLight) {
        trafficLightFlow.emit(value)
    }

    suspend fun subscribeTrafficLight(collector: FlowCollector<TrafficLight>) {
        trafficLightFlow.collect(collector)
    }

    suspend fun getTrafficLight(): TrafficLight? {
        return trafficLightFlow.lastOrNull()
    }

    suspend fun updateMobileye(value: MobileyeData) {
        mobileyeFlow.emit(value)
    }

    suspend fun subscribeMobileye(collector: FlowCollector<MobileyeData>) {
        mobileyeFlow.collect(collector)
    }

    suspend fun getMobileye(): MobileyeData? {
        return mobileyeFlow.lastOrNull()
    }
}
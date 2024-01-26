package viewmodel.config

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.last
import kotlinx.coroutines.launch
import util.common.LidarCalibration

object LidarCalibrationViewModel {
    private val left = MutableSharedFlow<LidarCalibration>(replay = 1)
    private val front = MutableSharedFlow<LidarCalibration>(replay = 1)
    private val right = MutableSharedFlow<LidarCalibration>(replay = 1)

    fun updateLeft(calibration: LidarCalibration) {
        CoroutineScope(Dispatchers.Main).launch {
            left.emit(calibration)
        }
    }

    fun subscribeLeft(collector: FlowCollector<LidarCalibration>) {
        CoroutineScope(Dispatchers.Main).launch {
            left.collect(collector)
        }
    }

    suspend fun getLeft(): LidarCalibration {
        return left.last()
    }

    fun updateFront(calibration: LidarCalibration) {
        CoroutineScope(Dispatchers.Main).launch {
            front.emit(calibration)
        }
    }

    fun subscribeFront(collector: FlowCollector<LidarCalibration>) {
        CoroutineScope(Dispatchers.Main).launch {
            front.collect(collector)
        }
    }

    suspend fun getFront(): LidarCalibration {
        return front.last()
    }

    fun updateRight(calibration: LidarCalibration) {
        CoroutineScope(Dispatchers.Main).launch {
            right.emit(calibration)
        }
    }

    fun subscribeRight(collector: FlowCollector<LidarCalibration>) {
        CoroutineScope(Dispatchers.Main).launch {
            right.collect(collector)
        }
    }

    suspend fun getRight(): LidarCalibration {
        return right.last()
    }
}
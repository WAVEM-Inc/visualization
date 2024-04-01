package viewmodel

import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow

object MonitorViewModel {
    private val isHorizontalFlow = MutableSharedFlow<Boolean>(replay = 1)

    suspend fun updateIsHorizontal(boolean: Boolean) {
        isHorizontalFlow.emit(boolean)
    }

    suspend fun subscribeIsHorizontal(collector: FlowCollector<Boolean>) {
        isHorizontalFlow.collect(collector)
    }
}
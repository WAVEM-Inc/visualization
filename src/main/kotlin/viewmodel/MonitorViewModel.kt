package viewmodel

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.launch

object MonitorViewModel {
    private val isHorizontal = MutableSharedFlow<Boolean>(replay = 1)

    fun updateIsHorizontal(boolean: Boolean) {
        CoroutineScope(Dispatchers.Main).launch {
            isHorizontal.emit(boolean)
        }
    }

    fun subscribeIsHorizontal(collector: FlowCollector<Boolean>) {
        CoroutineScope(Dispatchers.Main).launch {
            isHorizontal.collect(collector)
        }
    }
}
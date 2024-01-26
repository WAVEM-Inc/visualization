package viewmodel

import application.EvizLayoutType
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.launch

object WindowViewModel {
    private val currentLayout = MutableSharedFlow<EvizLayoutType>(1)

    init {
        updateCurrentLayout(EvizLayoutType.DASHBOARD)
    }

    suspend fun subscribeCurrentLayout(collector: FlowCollector<EvizLayoutType>) {
        currentLayout.collect(collector)
    }

    fun updateCurrentLayout(id: EvizLayoutType) {
        CoroutineScope(Dispatchers.Default).launch {
            currentLayout.emit(id)
        }
    }
}
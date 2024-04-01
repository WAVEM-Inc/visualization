package viewmodel

import application.type.ui.EvizLayoutType
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.launch

object WindowViewModel {
    private val currentLayoutId = MutableSharedFlow<EvizLayoutType>(1)

    init {
        CoroutineScope(Dispatchers.Default).launch {
            updateCurrentLayout(EvizLayoutType.DASHBOARD)
        }
    }

    suspend fun subscribeCurrentLayout(collector: FlowCollector<EvizLayoutType>) {
        currentLayoutId.collect(collector)
    }

    suspend fun updateCurrentLayout(id: EvizLayoutType) {
        currentLayoutId.emit(id)
    }

}
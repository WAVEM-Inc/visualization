package viewmodel

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import util.common.NotificationMessage

object NotificationViewModel {
    private val _notificationInfo = MutableSharedFlow<NotificationMessage>(replay = 1)

    fun updateNotification(message: NotificationMessage) {
        CoroutineScope(Dispatchers.Main).launch {
            _notificationInfo.emit(message)
        }
    }

    suspend fun subscribeNotification(collector: FlowCollector<NotificationMessage>) {
        _notificationInfo.collect(collector)
    }
}
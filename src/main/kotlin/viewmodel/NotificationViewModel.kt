package viewmodel

import application.type.data.NotificationMessage
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow

object NotificationViewModel {
    private val notificationInfoFlow = MutableSharedFlow<NotificationMessage>(replay = 1)

    suspend fun updateNotification(message: NotificationMessage) {
        notificationInfoFlow.emit(message)
    }

    suspend fun subscribeNotification(collector: FlowCollector<NotificationMessage>) {
        notificationInfoFlow.collect(collector)
    }
}
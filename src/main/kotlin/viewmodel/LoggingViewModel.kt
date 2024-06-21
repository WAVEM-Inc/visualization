package viewmodel

import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.lastOrNull

object LoggingViewModel {
    private val loggingDirFlow = MutableSharedFlow<String>()
    private val loggingDurationFlow = MutableSharedFlow<Long>()
    private val isLoggingFlow = MutableSharedFlow<Boolean>()

    suspend fun updateLoggingDirectory(value: String) {
        loggingDirFlow.emit(value)
    }

    suspend fun subscribeLoggingDirectory(collector: FlowCollector<String>) {
        loggingDirFlow.collect(collector)
    }

    suspend fun getLoggingDirectory(): String? {
        return loggingDirFlow.lastOrNull()
    }

    suspend fun updateLoggingDuration(value: Long) {
        loggingDurationFlow.emit(value)
    }

    suspend fun subscribeLoggingDuration(collector: FlowCollector<Long>) {
        loggingDurationFlow.collect(collector)
    }

    suspend fun getLoggingDuration(): Long? {
        return loggingDurationFlow.lastOrNull()
    }

    suspend fun updateIsLoggingState(value: Boolean) {
        isLoggingFlow.emit(value)
    }

    suspend fun subscribeIsLoggingState(collector: FlowCollector<Boolean>) {
        isLoggingFlow.collect(collector)
    }

    suspend fun getIsLoggingState(): Boolean? {
        return isLoggingFlow.lastOrNull()
    }
}
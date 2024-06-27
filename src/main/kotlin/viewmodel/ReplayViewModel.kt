package viewmodel

import essys_middle.streaming.Streaming
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.lastOrNull
import kotlinx.coroutines.launch

object ReplayViewModel {
    private val replayFilePathFlow = MutableSharedFlow<String>()
    private val replayStateFlow = MutableSharedFlow<Streaming.PlaybackState>()
    private val replayRequestStateFlow = MutableSharedFlow<Streaming.PlaybackState>()
    private val replayProgressFlow = MutableSharedFlow<Double>()

    public var requestState = Streaming.PlaybackState.NONE
    public var filePath = ""

    init {
        CoroutineScope(Dispatchers.Main).launch {
            replayRequestStateFlow.emit(Streaming.PlaybackState.NONE)
        }
    }

    suspend fun updateReplayFilePath(value: String) {
        filePath = value
        replayFilePathFlow.emit(value)
    }

    suspend fun subscribeReplayFilePath(collector: FlowCollector<String>) {
        replayFilePathFlow.collect(collector)
    }

    suspend fun getReplayFilePath(): String? {
        return replayFilePathFlow.lastOrNull()
    }

    suspend fun updateReplayState(value: Streaming.PlaybackState) {
        replayStateFlow.emit(value)
    }

    suspend fun subscribeReplayState(collector: FlowCollector<Streaming.PlaybackState>) {
        replayStateFlow.collect(collector)
    }

    suspend fun getReplayState(): Streaming.PlaybackState? {
        return replayStateFlow.lastOrNull()
    }

    suspend fun updateReplayProgress(value: Double) {
        replayProgressFlow.emit(value)
    }

    suspend fun subscribeReplayProgress(collector: FlowCollector<Double>) {
        replayProgressFlow.collect(collector)
    }

    suspend fun getReplayProgress(): Double? {
        return replayProgressFlow.lastOrNull()
    }

    suspend fun updateReplayRequestState(value: Streaming.PlaybackState) {
        requestState = value
        replayRequestStateFlow.emit(value)
    }

    suspend fun getReplayRequestState(): Streaming.PlaybackState? {
        return replayRequestStateFlow.lastOrNull()
    }
}
package viewmodel

import essys_middle.Streaming.PlaybackState
import kotlinx.coroutines.flow.FlowCollector
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.lastOrNull

object ReplayViewModel {
    private val replayDirFlow = MutableSharedFlow<String>()
    private val replayStateFlow = MutableSharedFlow<PlaybackState>()
    private val replayProgressFlow = MutableSharedFlow<Double>()

    suspend fun updateReplayDirectory(value: String) {
        replayDirFlow.emit(value)
    }

    suspend fun subscribeReplayDirectory(collector: FlowCollector<String>) {
        replayDirFlow.collect(collector)
    }

    suspend fun getReplayDirectory(): String? {
        return replayDirFlow.lastOrNull()
    }

    suspend fun updateReplayState(value: PlaybackState) {
        replayStateFlow.emit(value)
    }

    suspend fun subscribeReplayState(collector: FlowCollector<PlaybackState>) {
        replayStateFlow.collect(collector)
    }

    suspend fun getReplayState(): PlaybackState? {
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
}
package application.connect

import application.type.msg.ProtoMessageType
import essys_middle.streaming.Streaming
import kotlinx.coroutines.*
import viewmodel.ReplayViewModel
import java.io.DataOutputStream
import java.io.InputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.net.SocketException
import java.nio.ByteBuffer
import java.nio.ByteOrder

object TcpClient {
    private var socket: Socket = Socket()
    private var listener: OnMessageListener? = null

    private var receiveThread: Thread? = null
    private var requestThread: Thread? = null

    interface OnMessageListener {
        fun onStreamingReceive(message: Streaming.StreamingData)
        fun onLoggingReceive(message: Streaming.LoggingData)
    }

    fun connect(ipAddr: String, port: Int) {
        if (receiveThread != null) {
            receiveThread?.interrupt()
            receiveThread = null
        }

        if (requestThread != null) {
            requestThread?.interrupt()
            requestThread = null
        }

        try {
            socket.connect(InetSocketAddress(ipAddr, port))
        } catch (e: Exception) {
            e.printStackTrace()
        }

        receiveThread = Thread {
            while (Thread.currentThread().isInterrupted.not()) {
                val inputStream: InputStream = socket.getInputStream()
                var response = ByteArray(65545)
                val length = inputStream.read(response)
                response = response.copyOfRange(0, length)

                val payload = getPayloadFromArray(response)
                if (payload?.first == ProtoMessageType.REPLAY) {
                    val data = Streaming.StreamingData.parseFrom(payload.second)

                    listener?.onStreamingReceive(data)

                } else if (payload?.first == ProtoMessageType.LOGGING) {
                    val data = Streaming.LoggingData.parseFrom(payload.second)

                    listener?.onLoggingReceive(data)
                }
            }
        }
        receiveThread?.start()

        requestThread = Thread {
            while (Thread.currentThread().isInterrupted.not()) {
                Thread.sleep(1000)
                try {
                    val request = Streaming.StreamingData.newBuilder()
                        .setTimestamp(System.currentTimeMillis())
                        .setState(ReplayViewModel.requestState)
                        .setFilePath(ReplayViewModel.filePath)
                        .build()

                    println(ReplayViewModel.requestState)
                    println(ReplayViewModel.requestState)

                    sendData(ProtoMessageType.REPLAY.type, request.toByteArray())
                } catch (e: SocketException) {
                    e.printStackTrace()

                    try {
                        CoroutineScope(Dispatchers.Main).launch {
                            ReplayViewModel.updateReplayFilePath("")
                            ReplayViewModel.updateReplayRequestState(Streaming.PlaybackState.NONE)
                        }

                        println("Try reconnect to server...")
                        socket.close()
                        socket = Socket()
                        socket.connect(InetSocketAddress(ipAddr, port))
                    } catch (e: Exception) {
                        e.printStackTrace()
                    }
                }
            }
        }
        requestThread?.start()
    }

    fun sendData(msgId: Int, payload: ByteArray) {
        val payloadLength: Int = payload.size

        val buffer: ByteBuffer = ByteBuffer.allocate(13 + payloadLength)
        buffer.order(ByteOrder.LITTLE_ENDIAN)
        buffer.put("STX/".toByteArray())
        buffer.put(msgId.toByte())
        buffer.putInt(payloadLength)
        buffer.put(payload)
        buffer.put("/ETX".toByteArray())

        val message: ByteArray = buffer.array()

        try {
            val out: DataOutputStream = DataOutputStream(socket.getOutputStream())
            out.write(message)
            out.flush()
        } catch (e: Exception) {
            throw SocketException()
        } finally {
            buffer.clear()
        }
    }

    private fun getPayloadFromArray(data: ByteArray): Pair<ProtoMessageType, ByteArray>? {
        if (data.size < 13) { // Minimum size check based on the new protocol definition
            return null
        }

        // Check for start point "/STX" instead of "/STX"
        val stx = String(data.sliceArray(0..3))
        if (stx != "STX/") { // Corrected to match "STX/" as the start point example value
            return null
        }

        val msgId = data[4].toInt() and 0xFF // No change needed here
        val payloadLength =
            ByteBuffer.wrap(data.sliceArray(5..8)).order(ByteOrder.LITTLE_ENDIAN).getInt() // No change needed
        println("Length: $payloadLength")

        // Check if the total size matches or exceeds expected size (start + msgId + payloadLength + end)
        if (data.size < 9 + payloadLength + 4) { // Adjusted to reflect the new protocol definition
            return null
        }

        // Adjust end point extraction based on the new protocol definition
        val end = String(data.sliceArray((9 + payloadLength)..(9 + payloadLength + 3)))
        if (end != "/ETX") { // Corrected to match "/ETX" as the end point example value
            return null
        }

        // Extract the payload with the updated indices based on the new protocol
        val body: ByteArray = data.sliceArray(9 until (9 + payloadLength))

        return Pair(ProtoMessageType.get(msgId), body)
    }

    fun setOnMessageListener(listener: OnMessageListener) {
        this.listener = listener
    }
}
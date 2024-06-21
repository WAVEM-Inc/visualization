package application.connect

import application.type.msg.ProtoMessageType
import essys_middle.Streaming.LoggingData
import essys_middle.Streaming.StreamingData
import java.io.DataInputStream
import java.io.DataOutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder

class TcpClient {
    private var socket: Socket? = null

    private var receivingThread: Thread? = null
    private lateinit var listener: OnMessageListener

    interface OnMessageListener {
        fun onStreamingReceive(message: StreamingData)
        fun onLoggingReceive(message: LoggingData)
    }

    fun connect(ipAddress: String, serverPort: Int, clientPort: Int) {
        disconnect()

        socket = Socket()
        socket?.bind(InetSocketAddress(ipAddress, clientPort))
        socket?.connect(InetSocketAddress(ipAddress, serverPort))

        receivingThread = Thread {
            receiveData()
        }
        receivingThread?.start()
    }

    fun disconnect() {
        if (socket != null) {
            if (socket!!.isConnected) {
                socket!!.close()
            }

            socket = null
        }

        receivingThread?.interrupt()
        receivingThread?.join(200)
        receivingThread = null
    }

    private fun receiveData() {
        while (!Thread.currentThread().isInterrupted) {
            try {
                val inputStream = socket?.getInputStream()?.let { DataInputStream(it) }
                val buffer = ByteArray(4096)
                val bytesRead = inputStream?.read(buffer)
                if (bytesRead != -1) {
                    val messageBytes = bytesRead?.let { buffer.copyOfRange(0, it) }
                    val payload = messageBytes?.let { getPayloadFromArray(it) }

                    if (payload != null) {
                        if (payload.first == ProtoMessageType.SERVER_TO_EVIZ) {

                        }
                    }
                }

            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }

    fun sendData(msgId: Int, payload: ByteArray) {
        val payloadLength: Int = payload.size

        val buffer: ByteBuffer = ByteBuffer.allocate(13 + payloadLength)
        buffer.put("STX/".toByteArray())
        buffer.put(msgId.toByte())
        buffer.putInt(payloadLength)
        buffer.put(payload)
        buffer.put("/ETX".toByteArray())

        val message: ByteArray = buffer.array()

        try {
            val out: DataOutputStream = DataOutputStream(socket?.getOutputStream())
            out.write(message)
            out.flush()
        } catch (e: Exception) {
            e.printStackTrace()
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
        val payloadLength = ByteBuffer.wrap(data.sliceArray(5..8)).order(ByteOrder.LITTLE_ENDIAN).getInt() // No change needed

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
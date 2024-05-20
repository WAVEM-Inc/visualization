package application.connect

import apollo.dreamview.PointCloudOuterClass
import apollo.localization.Localization.LocalizationEstimate
import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacles
import application.type.msg.ProtoMessageType
import essys_middle.Dashboard.TrafficLight
import essys_middle.Dashboard.VehicleSignal
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder


class UdpClient {
    var socket = DatagramSocket()

    private lateinit var listener: OnMessageListener
    private var receivingThread: Thread? = null



    interface OnMessageListener {
        fun onLocalizationReceive(message: LocalizationEstimate)
        fun onObstacleReceive(message: PerceptionObstacles)
        fun onTrafficLightReceive(message: TrafficLight)
        fun onPointCloudReceive(message: PointCloudOuterClass.PointCloud)
        fun onDashboardReceive(message: VehicleSignal)
    }


    fun connect(ipAddress: String, port: Int) {
        disconnect()

        socket = DatagramSocket(null)
        socket.reuseAddress = true
        socket.bind(InetSocketAddress(ipAddress, port))

        receivingThread = Thread {
            receiveData()
        }
        receivingThread?.start()
    }

    fun disconnect() {
        if (socket.isConnected) {
            socket.disconnect()
        }

        socket.close()

        receivingThread?.interrupt()
        receivingThread = null
    }

    fun sendData(ipAddress: String, port: Int, payload: ByteArray) {
        val dp: DatagramPacket = DatagramPacket(payload, payload.size, InetSocketAddress(ipAddress, port))
        socket.send(dp)
    }

    private fun receiveData() {
        println("start to receive")
        while (!Thread.currentThread().isInterrupted) {
            try {
                val receiveBuffer = ByteArray(65535)
                val receivePacket = DatagramPacket(receiveBuffer, receiveBuffer.size)

                socket.receive(receivePacket)

                val payload = getPayloadFromArray(receivePacket.data)

                when (payload?.first) {
                    ProtoMessageType.LOCALIZATION -> listener.onLocalizationReceive(LocalizationEstimate.parseFrom(payload.second))
                    ProtoMessageType.OBSTACLE -> listener.onObstacleReceive(PerceptionObstacles.parseFrom(payload.second))
                    ProtoMessageType.TRAFFIC_LIGHT -> listener.onTrafficLightReceive(TrafficLight.parseFrom(payload.second))
                    ProtoMessageType.POINT_CLOUD -> listener.onPointCloudReceive(PointCloudOuterClass.PointCloud.parseFrom(payload.second))
                    ProtoMessageType.VEHICLE_SIGNAL -> listener.onDashboardReceive(VehicleSignal.parseFrom(payload.second))
                    null -> println("Received not invalid data.")
                }
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }

        println("end to receive")
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
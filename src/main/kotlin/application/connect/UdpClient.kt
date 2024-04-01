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



    interface OnMessageListener {
        fun onLocalizationReceive(message: LocalizationEstimate)
        fun onObstacleReceive(message: PerceptionObstacles)
        fun onTrafficLightReceive(message: TrafficLight)
        fun onPointCloudReceive(message: PointCloudOuterClass.PointCloud)
        fun onDashboardReceive(message: VehicleSignal)
    }


    fun connect(idAddress: String, port: Int) {
        disconnect()

        socket = DatagramSocket(InetSocketAddress(idAddress, port))
        receiveData()
    }

    fun disconnect() {
        if (socket.isConnected) {
            socket.disconnect()
        }

        socket.close()
    }

    fun sendData(ipAddress: String, port: Int, payload: ByteArray) {
        val dp: DatagramPacket = DatagramPacket(payload, payload.size, InetSocketAddress(ipAddress, port))
        socket.send(dp)
    }

    private fun receiveData() {
        while (socket.isConnected) {
            try {
                val receiveBuffer = ByteArray(65535)
                val receivePacket = DatagramPacket(receiveBuffer, receiveBuffer.size)

                socket.receive(receivePacket)

                val payload = getPayloadFromArray(receivePacket.data)

                if (::listener.isInitialized) {
                    when (payload?.first) {
                        ProtoMessageType.LOCALIZATION -> listener.onLocalizationReceive(LocalizationEstimate.parseFrom(payload.second))
                        ProtoMessageType.OBSTACLE -> listener.onObstacleReceive(PerceptionObstacles.parseFrom(payload.second))
                        ProtoMessageType.TRAFFIC_LIGHT -> listener.onTrafficLightReceive(TrafficLight.parseFrom(payload.second))
                        ProtoMessageType.POINT_CLOUD -> listener.onPointCloudReceive(PointCloudOuterClass.PointCloud.parseFrom(payload.second))
                        ProtoMessageType.VEHICLE_SIGNAL -> listener.onDashboardReceive(VehicleSignal.parseFrom(payload.second))
                        null -> println("Received not invalid data.")
                    }
                }
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }

    private fun getPayloadFromArray(data: ByteArray): Pair<ProtoMessageType, ByteArray>? {
        if (data.size < 13) {
            return null
        }

        val stx = data.slice(0..3)
        if (!stx.equals("/STX")) {
            return null
        }

        val msgId = data[4].toInt() and 0xFF
        val payloadLength = ByteBuffer.wrap(data.sliceArray(5..8)).order(ByteOrder.LITTLE_ENDIAN).getInt();
        if (data.size < 9 + payloadLength + 4) {
            return null
        }

        val end = String(data.sliceArray(9 + payloadLength..9 + payloadLength + 3))
        if (!end.equals("/ETX")) {
            return null
        }

        val body: ByteArray = data.sliceArray(9 until 9 + payloadLength)

        return Pair(ProtoMessageType.get(msgId), body)
    }

    fun setOnMessageListener(listener: OnMessageListener) {
        this.listener = listener
    }
}
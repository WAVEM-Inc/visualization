package util.protobuf

import apollo.localization.Localization.LocalizationEstimate
import com.google.protobuf.GeneratedMessageV3
import java.nio.ByteBuffer
import java.nio.ByteOrder



fun bytesToProtobufMessage(bytes: ByteArray): GeneratedMessageV3 {
    val data = bytesToBufferData(bytes)

    val message: GeneratedMessageV3 =
        when (data.header.msgId) {
            ProtoMessageType.LOCALIZATION -> LocalizationEstimate.parseFrom(data.payload)
            ProtoMessageType.OBSTACLE -> LocalizationEstimate.parseFrom(data.payload)
            ProtoMessageType.POINT_CLOUD -> LocalizationEstimate.parseFrom(data.payload)
        }

    return message
}

fun bytesToBufferData(data: ByteArray): BufferData {
    println()
    val header = getHeaderFromBytes(data.sliceArray(0..8))
    println(header)
    val payload = data.sliceArray(9 until 9 + header.payloadLength)
    val hasEnder = DataChecker.checkEtx(data.sliceArray(9 + header.payloadLength .. 12 + header.payloadLength))

    try {
        if (hasEnder) {
            return BufferData(header, payload)
        } else {
            throw Exception("_ETX is not founded in bytes.")
        }
    } catch (e: Exception) {
        e.printStackTrace()
        throw Exception("Parsing Error")
    }
}

private fun getHeaderFromBytes(headerBytes: ByteArray): BufferHeader {
    val stx = headerBytes.sliceArray(0..3)

    try {
        if (DataChecker.checkStx(stx)) {
            val id = headerBytes[4].toInt()
            val length = byteToInt(headerBytes.sliceArray(5..8))

            return BufferHeader(ProtoMessageType.get(id), length, 8 + length + 4)
        } else {
            throw Exception("STX_ is not founded in bytes.")
        }
    } catch (e: Exception) {
        e.printStackTrace()
        throw Exception("Parsing Error")
    }
}

fun byteToInt(bytes: ByteArray): Int {
    require(bytes.size == 4) { "length must be 4, got: ${bytes.size}" }
    return ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).int
}
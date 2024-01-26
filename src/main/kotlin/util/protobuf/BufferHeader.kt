package util.protobuf

data class BufferHeader(val msgId: ProtoMessageType, val payloadLength: Int, val endIndex: Int) {

}

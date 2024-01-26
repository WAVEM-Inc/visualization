package util.protobuf

enum class ProtoMessageType(val type: Int) {
    LOCALIZATION(1),
    OBSTACLE(2),
    POINT_CLOUD(4);

    companion object {
        @Throws
        fun get(id: Int): ProtoMessageType {
            return when(id) {
                0b00000001 -> LOCALIZATION
                0b00000010 -> OBSTACLE
                0b00000100 -> POINT_CLOUD
                else -> throw Exception()
            }
        }
    }
}
package application.type.msg

enum class ProtoMessageType(val type: Int) {
    LOCALIZATION(1),
    OBSTACLE(2),
    TRAFFIC_LIGHT(3),
    POINT_CLOUD(4),
    VEHICLE_SIGNAL(5),
    MOBIILEYE(6);

    companion object {
        @Throws
        fun get(id: Int): ProtoMessageType {
            return when(id) {
                1 -> LOCALIZATION
                2 -> OBSTACLE
                3 -> TRAFFIC_LIGHT
                4 -> POINT_CLOUD
                5 -> VEHICLE_SIGNAL
                else -> throw Exception("Unknown ProtoMessageType ID: $id")
            }
        }
    }
}
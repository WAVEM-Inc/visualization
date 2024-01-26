package util.protobuf

class DataChecker {
    companion object {
        private val HEADER_STX = "STX_".toByteArray()
        private val ENDER_ETX = "_ETX".toByteArray()

        fun checkStx(stx: ByteArray): Boolean {
            return stx contentEquals HEADER_STX
        }

        fun checkEtx(etx: ByteArray): Boolean {
            return etx contentEquals ENDER_ETX
        }
    }

}
package util.protobuf

import androidx.compose.ui.res.useResource
import apollo.dreamview.PointCloudOuterClass
import apollo.localization.Localization.LocalizationEstimate
import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacles
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.SupervisorJob
import java.io.File



fun splitBytes(bytes: ByteArray): ArrayList<ByteArray> {
    val result = ArrayList<ByteArray>()

    var findStx = false
    var findEtx = false
    var startIndex = 0
    var endIndex = 0

    for (i in 0..bytes.indices.last - 3) {
        val arr = bytes.sliceArray(i..i + 3)

        if (findStx && findEtx) {
            findStx = false
            findEtx = false
        }

        if (!findStx && !findEtx) {
            findStx = DataChecker.checkStx(arr)
            startIndex = i
        } else if (findStx && !findEtx) {
            findEtx = DataChecker.checkEtx(arr)
            endIndex = i + 3

            result.add(bytes.sliceArray(startIndex..endIndex))
        }
    }

    return result
}

fun readFileBuffers(): ReplayData {
    val localization = ArrayList<LocalizationEstimate>()
    val obstacle = ArrayList<PerceptionObstacles>()
    val pointCloud = ArrayList<PointCloudOuterClass.PointCloud>()

    var bytes: ByteArray = ByteArray(0)
    useResource("data/local_obs_pcl_proto_msg_20231101.log") { inputStream ->
        bytes = inputStream.readBytes()
    }

    var startIndex = 0
    var num = 0
    while (true) {
        try {
            val stx = bytes.sliceArray(startIndex..startIndex + 3)
            if (DataChecker.checkStx(stx)) {
                val id = bytes[startIndex + 4].toInt()
                val payLength = byteToInt(bytes.sliceArray(startIndex + 5..startIndex + 8))
                val payload = bytes.sliceArray(startIndex + 9 until startIndex + 9 + payLength)
                val etx = bytes.sliceArray(startIndex + 9 + payLength..startIndex + 12 + payLength)

                when (id) {
                    1 -> localization.add(LocalizationEstimate.parseFrom(payload))
                    2 -> obstacle.add(PerceptionObstacles.parseFrom(payload))
                    4 -> pointCloud.add(PointCloudOuterClass.PointCloud.parseFrom(payload))
                }

                startIndex += 13 + payLength
                num++
            } else {
                println("finish")
                break
            }
        } catch (e: IndexOutOfBoundsException) {
            println("Success all task")
            break
        } catch (e: Exception) {
            e.printStackTrace()
            break
        }
    }

    return ReplayData(localization, obstacle, pointCloud)
}
package util.protobuf

import apollo.dreamview.PointCloudOuterClass.PointCloud
import apollo.localization.Localization.LocalizationEstimate
import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacle
import apollo.perception.PerceptionObstacleOuterClass.PerceptionObstacles

data class ReplayData(
    val localization: List<LocalizationEstimate> = ArrayList(),
    val obstacles: List<PerceptionObstacles> = ArrayList(),
    val pointCloud: List<PointCloud> = ArrayList()
)

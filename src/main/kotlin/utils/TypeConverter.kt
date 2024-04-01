package utils

import apollo.dreamview.PointCloudOuterClass.PointCloud
import ui.opengl.math.Coordinate3D

fun pointCloudToCoordinate3DList(pointCloud: PointCloud): List<Coordinate3D> {
    val result = ArrayList<Coordinate3D>()
    for (i in 0 until pointCloud.numList.size step 3) {
        val x = pointCloud.numList[i]
        val y = pointCloud.numList[i + 1]
        val z = pointCloud.numList[i + 2]

        result.add(Coordinate3D(x.toDouble(), y.toDouble(), z.toDouble()))
    }

    return result
}
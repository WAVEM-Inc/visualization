package util.proj

import org.jxmapviewer.viewer.GeoPosition
import org.locationtech.proj4j.CRSFactory
import org.locationtech.proj4j.CoordinateTransformFactory
import org.locationtech.proj4j.ProjCoordinate


class ProjUtil {
    companion object {
        private val crsFactory = CRSFactory()
        private val WGS84 = crsFactory.createFromParameters("WGS84",
            "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")
        private val UTM = crsFactory.createFromParameters(
            "UTM",
            "+proj=utm +zone=52 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        )

        fun utmToWgs84(x: Double, y: Double, z: Double = 0.0): GeoPosition {
            val ctFactory = CoordinateTransformFactory()

            val utmToWgs = ctFactory.createTransform(UTM, WGS84)
            val result = ProjCoordinate()

            utmToWgs.transform(ProjCoordinate(x, y, z), result)

            return GeoPosition(result.y, result.x)
        }
    }
}
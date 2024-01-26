package ui.map

import org.jxmapviewer.JXMapViewer
import org.jxmapviewer.viewer.GeoPosition
import util.files.loadResource
import java.awt.Graphics2D
import java.awt.Image
import java.awt.image.BufferedImage
import javax.imageio.ImageIO

class DefaultLocationRenderer : LocationRenderer {
    private var img: BufferedImage?

    init {
        try {
            img = ImageIO.read(loadResource("icon/ic_navigation.png"))
        } catch (e: Exception) {
            img = null
            e.printStackTrace()
        }
    }

    override fun paintLocation(g: Graphics2D, map: JXMapViewer, geoPosition: GeoPosition, heading: Double) {
        if (img != null) {
            val point = map.tileFactory.geoToPixel(geoPosition, map.zoom)
            val image = img!!.getScaledInstance(30, 30, Image.SCALE_SMOOTH)

            val x = point.x.toInt() - image.getWidth(null) / 2
            val y = point.y.toInt() - image.getHeight(null) / 2

            g.rotate(heading, x.toDouble(), y.toDouble())
            g.drawImage(image, x, y, null)
            g.rotate(-heading, x.toDouble(), y.toDouble())
        }
    }
}
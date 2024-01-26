package ui.map

import org.jxmapviewer.JXMapViewer
import org.jxmapviewer.painter.AbstractPainter
import org.jxmapviewer.viewer.GeoPosition
import java.awt.Graphics2D

class LocationPainter: AbstractPainter<JXMapViewer>() {
    private var renderer: LocationRenderer? = DefaultLocationRenderer()
    private var geoPosition = GeoPosition(0.0, 0.0)
    private var heading = 0.0

    init {
        isAntialiasing = true
        isCacheable = false
    }

    fun setRenderer(renderer: LocationRenderer) {
        this.renderer = renderer
    }

    fun getGeoPosition(): GeoPosition {
        return geoPosition
    }

    fun setGeoPosition(geoPosition: GeoPosition) {
        this.geoPosition = geoPosition
    }

    fun getHeading(): Double {
        return heading
    }

    fun setHeading(heading: Double) {
        this.heading = heading
    }

    override fun doPaint(g: Graphics2D, map: JXMapViewer, width: Int, height: Int) {
        if (renderer != null) {
            val viewportBounds = map.viewportBounds

            g.translate(-viewportBounds.x, -viewportBounds.y)

            renderer!!.paintLocation(g, map, geoPosition, heading)

            g.translate(viewportBounds.x, viewportBounds.y)
        }
    }
}
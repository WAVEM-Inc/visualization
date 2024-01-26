package ui.map

import org.jxmapviewer.JXMapViewer
import org.jxmapviewer.viewer.GeoPosition
import java.awt.Graphics2D

interface LocationRenderer {
    fun paintLocation(g: Graphics2D, map: JXMapViewer, geoPosition: GeoPosition, heading: Double)
}
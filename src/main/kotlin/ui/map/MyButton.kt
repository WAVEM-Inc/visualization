package ui.map

import java.awt.Color
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.event.MouseAdapter
import java.awt.event.MouseEvent
import javax.swing.JButton


class MyButton : JButton() {
    fun getColor(): Color? {
        return color
    }

    fun setColor(color: Color?) {
        this.color = color
        setBackground(color)
    }

    var isOver = false
    private var color: Color? = null
    var colorOver: Color
    var colorClick: Color
    var borderColor: Color
    var radius = 0

    init {
        //  Init Color
        setColor(Color.WHITE)
        colorOver = Color(179, 250, 160)
        colorClick = Color(152, 184, 144)
        borderColor = Color(30, 136, 56)
        setContentAreaFilled(false)
        //  Add event mouse
        addMouseListener(object : MouseAdapter() {
            override fun mouseEntered(me: MouseEvent) {
                setBackground(colorOver)
                isOver = true
            }

            override fun mouseExited(me: MouseEvent) {
                setBackground(color)
                isOver = false
            }

            override fun mousePressed(me: MouseEvent) {
                setBackground(colorClick)
            }

            override fun mouseReleased(me: MouseEvent) {
                if (isOver) {
                    setBackground(colorOver)
                } else {
                    setBackground(color)
                }
            }
        })

        isFocusable = false
    }

    override fun paintComponent(grphcs: Graphics) {
        val g2 = grphcs as Graphics2D
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)

        //  Paint Border
        g2.color = borderColor
        if (isSelected) {
            g2.fillRoundRect(0, 0, width, height, radius, radius)
        }
        g2.color = getBackground()
        //  Border set 2 Pix
        g2.fillRoundRect(2, 2, width - 4, height - 4, radius, radius)
        super.paintComponent(grphcs)
    }
}
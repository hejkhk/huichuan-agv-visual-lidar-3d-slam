import QtQuick
import ".."

Canvas {
    id: root
    property string iconName: ""
    property color iconColor: Theme.textPrimary
    implicitWidth: 22 * AppMetrics.scale
    implicitHeight: 22 * AppMetrics.scale

    onIconNameChanged: requestPaint()
    onIconColorChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()

    onPaint: {
        var ctx = getContext("2d")
        ctx.reset()
        ctx.strokeStyle = root.iconColor
        ctx.fillStyle = root.iconColor
        ctx.lineWidth = Math.max(1.8, 2.0 * AppMetrics.scale)
        ctx.lineCap = "round"
        ctx.lineJoin = "round"

        var w = width
        var h = height
        if (root.iconName === "back") {
            ctx.beginPath()
            ctx.moveTo(w * 0.66, h * 0.18)
            ctx.lineTo(w * 0.34, h * 0.50)
            ctx.lineTo(w * 0.66, h * 0.82)
            ctx.stroke()
        } else if (root.iconName === "home") {
            ctx.beginPath()
            ctx.moveTo(w * 0.16, h * 0.47)
            ctx.lineTo(w * 0.50, h * 0.17)
            ctx.lineTo(w * 0.84, h * 0.47)
            ctx.stroke()
            ctx.beginPath()
            ctx.rect(w * 0.26, h * 0.43, w * 0.48, h * 0.39)
            ctx.stroke()
        } else if (root.iconName === "fullscreen") {
            ctx.beginPath()
            ctx.moveTo(w * 0.12, h * 0.38)
            ctx.lineTo(w * 0.12, h * 0.12)
            ctx.lineTo(w * 0.38, h * 0.12)
            ctx.moveTo(w * 0.62, h * 0.12)
            ctx.lineTo(w * 0.88, h * 0.12)
            ctx.lineTo(w * 0.88, h * 0.38)
            ctx.moveTo(w * 0.88, h * 0.62)
            ctx.lineTo(w * 0.88, h * 0.88)
            ctx.lineTo(w * 0.62, h * 0.88)
            ctx.moveTo(w * 0.38, h * 0.88)
            ctx.lineTo(w * 0.12, h * 0.88)
            ctx.lineTo(w * 0.12, h * 0.62)
            ctx.stroke()
        } else if (root.iconName === "restore") {
            ctx.strokeRect(w * 0.18, h * 0.30, w * 0.52, h * 0.52)
            ctx.strokeRect(w * 0.30, h * 0.18, w * 0.52, h * 0.52)
        }
    }
}

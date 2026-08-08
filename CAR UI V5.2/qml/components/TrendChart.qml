import QtQuick
import QtQuick.Layouts
import ".."

Rectangle {
    id: root
    property var seriesData: ({})
    property var timestamps: []
    property var seriesColors: ({})
    property var seriesLabels: ({})
    property string unit: ""
    property string title: ""

    color: Theme.surfaceMuted
    radius: Theme.radiusSmall * AppMetrics.scale
    clip: true

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: AppMetrics.unit
        spacing: 4 * AppMetrics.scale

        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.unit
            Text {
                text: root.title
                color: Theme.textPrimary
                font.pixelSize: AppMetrics.body
                font.weight: Font.DemiBold
            }
            Item { Layout.fillWidth: true }
            Repeater {
                model: {
                    var keys = Object.keys(root.seriesLabels)
                    return keys
                }
                RowLayout {
                    spacing: 4 * AppMetrics.scale
                    required property string modelData
                    Rectangle {
                        width: 12 * AppMetrics.scale; height: 12 * AppMetrics.scale
                        radius: 2
                        color: root.seriesColors[modelData] || Theme.textMuted
                    }
                    Text {
                        text: root.seriesLabels[modelData] || modelData
                        color: Theme.textSecondary
                        font.pixelSize: AppMetrics.caption
                    }
                }
            }
        }

        Canvas {
            id: chart
            Layout.fillWidth: true
            Layout.fillHeight: true
            renderStrategy: Canvas.Threaded
            property var renderData: ({})

            Component.onCompleted: renderData = root.seriesData
            onPaint: {
                var ctx = getContext("2d")
                var w = width; var h = height
                if (w < 10 || h < 10) return
                ctx.clearRect(0, 0, w, h)

                var margin = { top: 8, right: 12, bottom: 24, left: 48 }
                var cw = w - margin.left - margin.right
                var ch = h - margin.top - margin.bottom
                if (cw < 10 || ch < 10) return

                var data = root.seriesData
                var ts = root.timestamps
                if (!data || !ts || ts.length < 2) {
                    ctx.fillStyle = Theme.textMuted
                    ctx.font = (12 * AppMetrics.scale) + "px sans-serif"
                    ctx.textAlign = "center"
                    ctx.fillText(I18n.t("暂无数据"), w / 2, h / 2)
                    return
                }

                var globalMin = Infinity, globalMax = -Infinity
                var keys = Object.keys(data)
                for (var k = 0; k < keys.length; k++) {
                    var arr = data[keys[k]]
                    if (!arr) continue
                    for (var j = 0; j < arr.length; j++) {
                        if (arr[j] < globalMin) globalMin = arr[j]
                        if (arr[j] > globalMax) globalMax = arr[j]
                    }
                }
                if (!isFinite(globalMin) || !isFinite(globalMax)) return
                var range = globalMax - globalMin
                if (range < 0.01) { globalMin -= 5; globalMax += 5; range = 10 }
                var pad = range * 0.08
                globalMin -= pad; globalMax += pad; range = globalMax - globalMin

                ctx.strokeStyle = Theme.divider
                ctx.lineWidth = 1
                for (var gi = 0; gi <= 4; gi++) {
                    var gy = margin.top + ch - (gi / 4) * ch
                    ctx.beginPath(); ctx.moveTo(margin.left, gy); ctx.lineTo(w - margin.right, gy); ctx.stroke()
                    var gval = globalMin + (gi / 4) * range
                    ctx.fillStyle = Theme.textMuted
                    ctx.font = (10 * AppMetrics.scale) + "px sans-serif"
                    ctx.textAlign = "right"
                    ctx.fillText(gval.toFixed(1), margin.left - 4, gy + 3)
                }

                var tMin = ts[0], tMax = ts[ts.length - 1]
                ctx.fillStyle = Theme.textMuted
                ctx.font = (10 * AppMetrics.scale) + "px sans-serif"
                ctx.textAlign = "center"
                for (var ti = 0; ti <= 4; ti++) {
                    var tx = margin.left + (ti / 4) * cw
                    var tval = tMin + (ti / 4) * (tMax - tMin)
                    var d = new Date(tval * 1000)
                    var hh = ("0" + d.getHours()).slice(-2)
                    var mm = ("0" + d.getMinutes()).slice(-2)
                    ctx.fillText(hh + ":" + mm, tx, h - 4)
                }

                for (var ki = 0; ki < keys.length; ki++) {
                    var key = keys[ki]
                    var series = data[key]
                    if (!series || series.length < 2) continue
                    var color = root.seriesColors[key] || Theme.primary
                    ctx.strokeStyle = color
                    ctx.lineWidth = 2 * AppMetrics.scale
                    ctx.beginPath()
                    for (var si = 0; si < series.length; si++) {
                        var sx = margin.left + ((ts[si] - tMin) / (tMax - tMin || 1)) * cw
                        var sy = margin.top + ch - ((series[si] - globalMin) / range) * ch
                        if (si === 0) ctx.moveTo(sx, sy)
                        else ctx.lineTo(sx, sy)
                    }
                    ctx.stroke()
                }
            }

            Connections {
                target: root
                function onSeriesDataChanged() {
                    chart.renderData = root.seriesData
                    chart.requestPaint()
                }
                function onTimestampsChanged() { chart.requestPaint() }
            }
        }
    }
}

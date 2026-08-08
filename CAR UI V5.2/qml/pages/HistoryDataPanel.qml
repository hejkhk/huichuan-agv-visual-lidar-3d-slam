import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

ColumnLayout {
    id: root
    spacing: AppMetrics.gap
    property int chartGroup: 0

    SectionHeader {
        Layout.fillWidth: true
        title: I18n.t("历史数据")
        subtitle: I18n.t("过去 1 小时的趋势图表")
    }

    SegmentedControl {
        Layout.fillWidth: true
        options: [I18n.t("系统资源"), I18n.t("电池"), I18n.t("运动")]
        currentIndex: root.chartGroup
        onSelected: function(index) { root.chartGroup = index }
    }

    AppCard {
        Layout.fillWidth: true
        Layout.preferredHeight: 82 * AppMetrics.scale
        RowLayout {
            anchors.fill: parent
            anchors.margins: AppMetrics.cardPadding
            spacing: AppMetrics.sectionGap

            Repeater {
                model: {
                    if (root.chartGroup === 0) return [
                        { label: I18n.t("CPU 占用率"), value: (backend.snapshot.cpu_percent ?? 0).toFixed(1) + "%", color: Theme.primary },
                        { label: I18n.t("内存占用率"), value: (backend.snapshot.memory_percent ?? 0).toFixed(1) + "%", color: Theme.info },
                        { label: I18n.t("CPU 温度"), value: (backend.snapshot.cpu_temperature ?? 0).toFixed(1) + "°C", color: Theme.warning }
                    ]
                    if (root.chartGroup === 1) return [
                        { label: I18n.t("电池电量"), value: (backend.snapshot.battery_percent ?? 0) + "%", color: Theme.success },
                        { label: I18n.t("电池电压"), value: (backend.snapshot.battery_voltage ?? 0).toFixed(1) + "V", color: Theme.purple }
                    ]
                    return [
                        { label: I18n.t("速度"), value: (backend.snapshot.vx ?? 0).toFixed(2) + " m/s", color: Theme.primary }
                    ]
                }
                ColumnLayout {
                    required property var modelData
                    spacing: 2 * AppMetrics.scale
                    Text {
                        text: modelData.label
                        color: Theme.textMuted
                        font.pixelSize: AppMetrics.caption
                    }
                    Text {
                        text: modelData.value
                        color: modelData.color
                        font.pixelSize: AppMetrics.sectionTitle
                        font.weight: Font.Bold
                    }
                }
            }
            Item { Layout.fillWidth: true }
        }
    }

    TrendChart {
        Layout.fillWidth: true
        Layout.fillHeight: true
        title: {
            if (root.chartGroup === 0) return I18n.t("系统资源")
            if (root.chartGroup === 1) return I18n.t("电池")
            return I18n.t("运动")
        }
        timestamps: backend.history.timestamps ?? []
        seriesData: {
            var h = backend.history || {}
            if (root.chartGroup === 0) return {
                "cpu_percent": h.cpu_percent || [],
                "memory_percent": h.memory_percent || [],
                "cpu_temperature": h.cpu_temperature || []
            }
            if (root.chartGroup === 1) return {
                "battery_percent": h.battery_percent || [],
                "battery_voltage": h.battery_voltage || []
            }
            return { "vx": h.vx || [] }
        }
        seriesColors: root.chartGroup === 0 ? ({ "cpu_percent": Theme.primary, "memory_percent": Theme.info, "cpu_temperature": Theme.warning })
            : root.chartGroup === 1 ? ({ "battery_percent": Theme.success, "battery_voltage": Theme.purple })
            : ({ "vx": Theme.primary })
        seriesLabels: root.chartGroup === 0 ? ({
            "cpu_percent": I18n.t("CPU 占用率"),
            "memory_percent": I18n.t("内存占用率"),
            "cpu_temperature": I18n.t("CPU 温度")
        }) : root.chartGroup === 1 ? ({
            "battery_percent": I18n.t("电池电量"),
            "battery_voltage": I18n.t("电池电压")
        }) : ({
            "vx": I18n.t("速度")
        })
    }

    Connections {
        target: backend
        function onHistoryChanged() {}
    }
}

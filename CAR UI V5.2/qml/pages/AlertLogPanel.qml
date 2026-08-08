import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

ColumnLayout {
    id: root
    spacing: AppMetrics.gap
    property string filterLevel: "ALL"

    SectionHeader {
        Layout.fillWidth: true
        title: I18n.t("告警日志")
        subtitle: I18n.t("系统事件与异常记录")
        StatusBadge {
            status: alertCount > 0 ? "WARNING" : "NORMAL"
            label: "" + alertCount
        }
        property int alertCount: (backend.alerts || []).length
        AppButton {
            compact: true
            outlined: true
            text: I18n.t("清空")
            onClicked: backend.clearAlerts()
        }
    }

    SegmentedControl {
        Layout.fillWidth: true
        options: [I18n.t("全部"), I18n.t("信息"), I18n.t("警告"), I18n.t("错误")]
        currentIndex: root.filterLevel === "ALL" ? 0 : root.filterLevel === "INFO" ? 1 : root.filterLevel === "WARNING" ? 2 : 3
        onSelected: function(index) {
            root.filterLevel = ["ALL", "INFO", "WARNING", "ERROR"][index]
        }
    }

    ListView {
        Layout.fillWidth: true
        Layout.fillHeight: true
        clip: true
        spacing: 2 * AppMetrics.scale
        model: {
            var alerts = backend.alerts || []
            if (root.filterLevel === "ALL") return alerts
            return alerts.filter(function(a) { return a.level === root.filterLevel })
        }
        delegate: Rectangle {
            required property var modelData
            required property int index
            width: ListView.view.width
            height: 62 * AppMetrics.scale
            radius: Theme.radiusSmall * AppMetrics.scale
            color: modelData.level === "ERROR" ? Qt.rgba(0.9, 0.2, 0.2, 0.1)
                : modelData.level === "WARNING" ? Qt.rgba(0.9, 0.6, 0.1, 0.1)
                : Theme.surfaceMuted
            border.color: modelData.level === "ERROR" ? Theme.danger
                : modelData.level === "WARNING" ? Theme.warning
                : Theme.divider
            border.width: Theme.borderWidth

            RowLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                spacing: AppMetrics.unit

                StatusBadge {
                    status: modelData.level
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 2 * AppMetrics.scale
                    Text {
                        text: I18n.t(modelData.message_key)
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.body
                        font.weight: Font.DemiBold
                    }
                    Text {
                        text: {
                            var d = new Date(modelData.timestamp * 1000)
                            var hh = ("0" + d.getHours()).slice(-2)
                            var mm = ("0" + d.getMinutes()).slice(-2)
                            var ss = ("0" + d.getSeconds()).slice(-2)
                            return hh + ":" + mm + ":" + ss
                        }
                        color: Theme.textMuted
                        font.pixelSize: AppMetrics.caption
                    }
                }

                Text {
                    text: modelData.category
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.caption
                }
            }
        }

        EmptyState {
            anchors.centerIn: parent
            visible: parent.count === 0
            title: I18n.t("暂无告警")
            description: I18n.t("系统运行正常时不会产生告警记录")
        }
    }
}

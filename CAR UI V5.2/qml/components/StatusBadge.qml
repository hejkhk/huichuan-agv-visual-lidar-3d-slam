import QtQuick
import ".."

Rectangle {
    id: root
    property string status: "NORMAL"
    property string label: status === "NORMAL" ? I18n.t("正常")
        : status === "WARNING" ? I18n.t("警告")
        : status === "ERROR" ? I18n.t("错误")
        : status === "DISCONNECTED" ? I18n.t("未接入")
        : I18n.t("未知")
    readonly property color tone: status === "NORMAL" ? Theme.success
        : status === "WARNING" ? Theme.warning
        : status === "ERROR" ? Theme.danger
        : Theme.textMuted
    implicitWidth: badgeText.implicitWidth + 18 * AppMetrics.scale
    implicitHeight: 26 * AppMetrics.scale
    radius: height / 2
    color: Qt.rgba(tone.r, tone.g, tone.b, Theme.darkMode ? 0.18 : 0.12)
    Text { id: badgeText; anchors.centerIn: parent; text: root.label; color: root.tone; font.pixelSize: AppMetrics.caption; font.weight: Font.DemiBold }
}

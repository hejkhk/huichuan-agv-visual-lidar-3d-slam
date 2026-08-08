import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."

Button {
    id: root
    required property var point
    property bool selected: false
    implicitHeight: 58 * AppMetrics.scale
    hoverEnabled: true

    background: Rectangle {
        radius: Theme.radiusSmall * AppMetrics.scale
        color: root.selected ? Theme.primarySoft : root.down ? Theme.surfaceMuted : Theme.surface
        border.color: root.selected ? Theme.primary : Theme.divider
        border.width: root.selected ? Theme.borderWidthStrong : Theme.borderWidth
        SequentialAnimation on opacity {
            running: root.selected && Performance.decorativeAnimations
            loops: Animation.Infinite
            NumberAnimation { to: 0.82; duration: Performance.pulseDuration; easing.type: Easing.InOutSine }
            NumberAnimation { to: 1.0; duration: Performance.pulseDuration; easing.type: Easing.InOutSine }
        }
    }
    contentItem: RowLayout {
        spacing: AppMetrics.unit
        StatusDot { status: root.point.is_charging_point ? "NORMAL" : "UNKNOWN" }
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 1
            Text { text: I18n.t(root.point.name); color: root.selected ? Theme.primary : Theme.textPrimary; font.pixelSize: AppMetrics.body; font.weight: Font.DemiBold; elide: Text.ElideRight }
            Text { visible: root.point.is_charging_point ?? false; text: I18n.t("充电点"); color: Theme.success; font.pixelSize: AppMetrics.caption }
        }
    }
}

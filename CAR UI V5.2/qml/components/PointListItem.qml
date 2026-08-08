import QtQuick
import QtQuick.Layouts
import ".."

AppCard {
    id: root
    property var point
    readonly property bool compact: width < 720 * AppMetrics.scale
    readonly property int headingDegrees:
        Math.round((((Number(point.yaw ?? 0) * 180 / Math.PI) % 360) + 360) % 360)
    signal headingRequested(); signal renameRequested(); signal deleteRequested(); signal addRequested()
    implicitHeight: 68 * AppMetrics.scale
    RowLayout { anchors.fill: parent; anchors.margins: AppMetrics.unit*1.2; spacing: AppMetrics.unit
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 1
            Text { Layout.fillWidth: true; text: I18n.t(point.name) + (point.is_charging_point ? "  · " + I18n.t("充电点") : ""); color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.bold: true; elide: Text.ElideRight }
            RowLayout {
                spacing: 4 * AppMetrics.scale
                Text { text: "➜"; rotation: -root.headingDegrees; color: Theme.primary; font.pixelSize: AppMetrics.body; font.bold: true }
                Text { text: I18n.t("到达方向") + " " + root.headingDegrees + "°"; color: Theme.textMuted; font.pixelSize: AppMetrics.caption }
            }
        }
        AppButton { text: I18n.t("朝向"); implicitWidth: (root.compact ? 66 : 88)*AppMetrics.scale; outlined: true; onClicked: root.headingRequested() }
        AppButton { text: I18n.t("重命名"); implicitWidth: (root.compact ? 70 : 104)*AppMetrics.scale; accent: Theme.textMuted; onClicked: root.renameRequested() }
        AppButton { text: I18n.t("删除"); implicitWidth: (root.compact ? 58 : 86)*AppMetrics.scale; accent: Theme.danger; onClicked: root.deleteRequested() }
        AppButton { text: I18n.t("加入"); implicitWidth: (root.compact ? 58 : 86)*AppMetrics.scale; accent: Theme.primary; onClicked: root.addRequested() }
    }
}

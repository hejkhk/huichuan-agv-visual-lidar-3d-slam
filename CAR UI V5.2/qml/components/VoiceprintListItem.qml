import QtQuick
import QtQuick.Layouts
import ".."

AppCard {
    id: root
    property var voiceprint
    property bool canMoveUp: false
    property bool canMoveDown: false
    signal moveUpRequested()
    signal moveDownRequested()
    signal renameRequested()
    signal deleteRequested()
    implicitHeight: 68*AppMetrics.scale
    RowLayout { anchors.fill: parent; anchors.margins: AppMetrics.unit*1.5
        Rectangle {
            implicitWidth: 42 * AppMetrics.scale
            implicitHeight: 42 * AppMetrics.scale
            radius: width / 2
            color: Theme.purpleSoft
            Text {
                anchors.centerIn: parent
                text: voiceprint.priority ?? ""
                color: Theme.purple
                font.pixelSize: AppMetrics.body
                font.bold: true
            }
        }
        Text { Layout.fillWidth: true; text: I18n.t(voiceprint.name); font.pixelSize: AppMetrics.body; font.bold: true; color: Theme.textPrimary }
        AppButton { text: I18n.t("上移"); implicitWidth: 84*AppMetrics.scale; outlined: true; accent: Theme.purple; enabled: root.canMoveUp; onClicked: root.moveUpRequested() }
        AppButton { text: I18n.t("下移"); implicitWidth: 84*AppMetrics.scale; outlined: true; accent: Theme.purple; enabled: root.canMoveDown; onClicked: root.moveDownRequested() }
        AppButton { text: I18n.t("重命名"); implicitWidth: 108*AppMetrics.scale; accent: Theme.textMuted; onClicked: root.renameRequested() }
        AppButton { text: I18n.t("删除"); implicitWidth: 90*AppMetrics.scale; accent: Theme.danger; onClicked: root.deleteRequested() }
    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

Dialog {
    id: root
    signal acceptedAction()
    signal tutorialRequested()
    title: I18n.t("建图模式")
    modal: true
    padding: AppMetrics.cardPadding
    Overlay.modal: Rectangle { color: Theme.overlay }
    anchors.centerIn: Overlay.overlay
    width: Math.min(560 * AppMetrics.scale, parent ? parent.width * 0.7 : 560)
    background: Rectangle { color: Theme.surface; radius: Theme.radius }
    contentItem: ColumnLayout {
        spacing: AppMetrics.gap
        Text {
            Layout.fillWidth: true
            text: root.title
            font.pixelSize: AppMetrics.title
            font.bold: true
            color: Theme.textPrimary
        }
        Text {
            Layout.fillWidth: true
            text: I18n.t("请使用手柄操作，前往地图灰色区域建图，建图完成后请手动保存")
            wrapMode: Text.WordWrap
            font.pixelSize: AppMetrics.body
            color: Theme.textMuted
        }
        Text {
            Layout.fillWidth: true
            text: I18n.t("建议观看手柄教程，使用手柄切换至建图档位")
            wrapMode: Text.WordWrap
            font.pixelSize: AppMetrics.body
            font.weight: Font.DemiBold
            color: Theme.warning
        }
        RowLayout {
            Layout.fillWidth: true
            SecondaryButton {
                text: I18n.t("手柄教程")
                onClicked: {
                    root.close()
                    root.tutorialRequested()
                }
            }
            Item { Layout.fillWidth: true }
            AppButton {
                text: I18n.t("取消")
                accent: Theme.textMuted
                onClicked: root.close()
            }
            AppButton {
                text: I18n.t("确认")
                accent: Theme.info
                onClicked: { root.close(); root.acceptedAction() }
            }
        }
    }
}

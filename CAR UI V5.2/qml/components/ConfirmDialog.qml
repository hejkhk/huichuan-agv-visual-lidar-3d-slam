import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."

Dialog {
    id: root
    property string message: I18n.t("确认执行此操作？")
    signal acceptedAction()
    modal: true;
    padding: AppMetrics.cardPadding
    Overlay.modal: Rectangle { color: Theme.overlay } anchors.centerIn: Overlay.overlay; width: Math.min(520*AppMetrics.scale, parent ? parent.width*.7 : 520)
    background: Rectangle { color: Theme.surface; radius: Theme.radius }
    contentItem: ColumnLayout { spacing: AppMetrics.gap
        Text { Layout.fillWidth: true; text: root.title; font.pixelSize: AppMetrics.title; font.bold: true; color: Theme.textPrimary }
        Text { Layout.fillWidth: true; text: root.message; wrapMode: Text.WordWrap; font.pixelSize: AppMetrics.body; color: Theme.textMuted }
        RowLayout { Layout.fillWidth: true; Item { Layout.fillWidth: true }
AppButton { text: I18n.t("取消"); accent: Theme.textMuted; onClicked: root.close() }
AppButton { text: I18n.t("确认"); accent: Theme.danger; onClicked: { root.close(); root.acceptedAction() } } }
    }
}

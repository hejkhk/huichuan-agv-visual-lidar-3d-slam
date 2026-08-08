import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppInputDialog {
    id: root
    objectName: "saveMapDialog"
    parent: window.contentItem
    padding: AppMetrics.cardPadding
    dialogWidth: 560 * AppMetrics.scale
    dialogHeight: 280 * AppMetrics.scale
    onOpened: nameField.forceActiveFocus()
    onClosed: { Qt.inputMethod.hide(); nameField.clear() }
    function commit() {
        backend.saveMap(nameField.text)
        root.close()
    }
    ColumnLayout {
        anchors.fill: parent
        spacing: AppMetrics.gap
        Text {
            text: I18n.t("保存地图")
            font.pixelSize: AppMetrics.title
            font.bold: true
            color: Theme.textPrimary
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.unit
            TextField {
                id: nameField
                Layout.fillWidth: true
                placeholderText: I18n.t("地图名称")
                font.pixelSize: AppMetrics.body
            }
            KeyboardToggleButton { inputItem: nameField }
        }
        Item { Layout.fillHeight: true }
        RowLayout {
            Layout.fillWidth: true
            Item { Layout.fillWidth: true }
            AppButton {
                text: I18n.t("取消")
                accent: Theme.textMuted
                onClicked: root.close()
            }
            AppButton {
                text: I18n.t("保存")
                enabled: nameField.text.trim().length > 0
                accent: Theme.success
                onClicked: root.commit()
            }
        }
    }
}

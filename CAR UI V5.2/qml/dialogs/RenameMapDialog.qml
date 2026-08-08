import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppInputDialog {
    id: root
    objectName: "renameMapDialog"
    parent: window.contentItem
    property string mapId: ""
    property string currentName: ""
    padding: AppMetrics.cardPadding
    dialogWidth: 620 * AppMetrics.scale
    dialogHeight: 280 * AppMetrics.scale

    onOpened: {
        nameField.text = root.currentName
        nameField.selectAll()
        nameField.forceActiveFocus()
    }
    onClosed: {
        Qt.inputMethod.hide()
        nameField.clear()
    }

    function commit() {
        backend.renameMap(root.mapId, nameField.text)
        root.close()
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: AppMetrics.gap

        Text {
            text: I18n.t("重命名地图")
            color: Theme.textPrimary
            font.pixelSize: AppMetrics.title
            font.weight: Font.DemiBold
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.unit
            TextField {
                id: nameField
                Layout.fillWidth: true
                placeholderText: I18n.t("地图名称")
                font.pixelSize: AppMetrics.body
                onAccepted: if (text.trim().length > 0) root.commit()
            }
            KeyboardToggleButton { inputItem: nameField }
        }
        Text {
            Layout.fillWidth: true
            text: I18n.t("名称不包含扩展名，支持中文、英文和空格")
            color: Theme.textMuted
            font.pixelSize: AppMetrics.caption
        }
        Item { Layout.fillHeight: true }
        RowLayout {
            Layout.fillWidth: true
            Item { Layout.fillWidth: true }
            SecondaryButton {
                text: I18n.t("取消")
                onClicked: root.close()
            }
            PrimaryButton {
                text: I18n.t("保存")
                enabled: nameField.text.trim().length > 0
                    && nameField.text.trim() !== root.currentName
                onClicked: root.commit()
            }
        }
    }
}

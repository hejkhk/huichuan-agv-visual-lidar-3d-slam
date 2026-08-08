import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppInputDialog {
    id: root
    parent: window.contentItem
    property string targetSsid: ""
    padding: AppMetrics.cardPadding
    dialogWidth: 700 * AppMetrics.scale
    dialogHeight: 270 * AppMetrics.scale
    onOpened: passwordField.forceActiveFocus()
    onClosed: Qt.inputMethod.hide()
    function dismiss() { root.close() }
    ColumnLayout {
        anchors.fill: parent
        spacing: AppMetrics.gap
        Text {
            text: I18n.t("连接到") + " " + root.targetSsid
            font.pixelSize: AppMetrics.title
            font.bold: true
            color: Theme.textPrimary
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.unit
            TextField {
                id: passwordField
                Layout.fillWidth: true
                placeholderText: I18n.t("输入密码")
                echoMode: TextInput.Password
                font.pixelSize: AppMetrics.body
            }
            KeyboardToggleButton { inputItem: passwordField }
        }
        Item { Layout.fillHeight: true }
        RowLayout {
            Layout.fillWidth: true
            Item { Layout.fillWidth: true }
            AppButton {
                text: I18n.t("取消")
                accent: Theme.danger
                onClicked: root.close()
            }
            AppButton {
                text: I18n.t("连接")
                accent: Theme.success
                onClicked: {
                    backend.connectWifi(root.targetSsid, passwordField.text)
                    passwordField.clear()
                    root.close()
                }
            }
        }
    }
}

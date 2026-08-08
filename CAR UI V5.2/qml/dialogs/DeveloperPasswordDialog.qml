import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppInputDialog {
    id: root
    objectName: "developerPasswordDialog"
    signal unlocked()
    dialogWidth: 620
    dialogHeight: 300

    function begin() {
        passwordField.text = ""
        errorText.text = ""
        open()
    }

    onOpened: passwordField.forceActiveFocus()

    ColumnLayout {
        anchors.fill: parent
        spacing: AppMetrics.gap

        Text {
            text: I18n.t("进入开发者模式")
            color: Theme.textPrimary
            font.pixelSize: AppMetrics.title
            font.weight: Font.DemiBold
        }
        Text {
            text: I18n.t("请输入开发者密码")
            color: Theme.textSecondary
            font.pixelSize: AppMetrics.body
        }
        RowLayout {
            Layout.fillWidth: true
            TextField {
                id: passwordField
                objectName: "developerPasswordField"
                Layout.fillWidth: true
                echoMode: TextInput.Password
                inputMethodHints: Qt.ImhDigitsOnly
                placeholderText: I18n.t("密码")
                font.pixelSize: AppMetrics.body
                onAccepted: confirmButton.clicked()
            }
            KeyboardToggleButton { inputItem: passwordField }
        }
        Text {
            id: errorText
            Layout.fillWidth: true
            color: Theme.danger
            font.pixelSize: AppMetrics.small
        }
        Item { Layout.fillHeight: true }
        RowLayout {
            Layout.alignment: Qt.AlignRight
            SecondaryButton {
                text: I18n.t("取消")
                onClicked: root.close()
            }
            PrimaryButton {
                id: confirmButton
                text: I18n.t("确认")
                onClicked: {
                    if (passwordField.text === "123") {
                        root.close()
                        root.unlocked()
                    } else {
                        errorText.text = I18n.t("密码错误")
                        passwordField.selectAll()
                        window.showKeyboardFor(passwordField)
                    }
                }
            }
        }
    }
}

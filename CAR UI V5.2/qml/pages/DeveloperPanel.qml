import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

ColumnLayout {
    id: root
    spacing: AppMetrics.gap

    Component.onCompleted: backend.refreshDeveloperData()

    Timer {
        interval: 1500
        repeat: true
        running: root.visible
        onTriggered: backend.refreshDeveloperData()
    }

    SectionHeader {
        Layout.fillWidth: true
        title: I18n.t("开发者模式")
        subtitle: I18n.t("ROS 2 状态与通信设置")
        StatusBadge { status: "WARNING"; label: I18n.t("日志只读") }
        AppButton {
            compact: true
            outlined: true
            text: I18n.t("立即刷新")
            onClicked: backend.refreshDeveloperData()
        }
    }

    AppCard {
        Layout.fillWidth: true
        Layout.preferredHeight: 82 * AppMetrics.scale

        RowLayout {
            anchors.fill: parent
            anchors.margins: AppMetrics.cardPadding
            spacing: AppMetrics.gap

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 2 * AppMetrics.scale
                Text {
                    text: I18n.t("ROS 2 DDS Domain ID")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.body
                    font.weight: Font.DemiBold
                }
                Text {
                    text: I18n.t("范围 0–232，保存后重启应用生效")
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.caption
                }
            }

            TextField {
                id: rosDomainField
                objectName: "rosDomainIdField"
                Layout.preferredWidth: 132 * AppMetrics.scale
                Layout.preferredHeight: AppMetrics.buttonHeight
                text: String(backend.rosDomainId)
                horizontalAlignment: TextInput.AlignHCenter
                verticalAlignment: TextInput.AlignVCenter
                selectByMouse: true
                inputMethodHints: Qt.ImhDigitsOnly
                validator: IntValidator { bottom: 0; top: 232 }
                color: Theme.textPrimary
                font.pixelSize: AppMetrics.body
                background: Rectangle {
                    color: Theme.surfaceMuted
                    radius: Theme.radiusSmall
                    border.color: rosDomainField.activeFocus ? Theme.primary : Theme.border
                    border.width: Theme.borderWidth
                }
            }
            KeyboardToggleButton { inputItem: rosDomainField }
            PrimaryButton {
                text: I18n.t("保存")
                enabled: rosDomainField.acceptableInput
                onClicked: backend.setRosDomainId(Number(rosDomainField.text))
            }
        }
    }

    AppCard {
        Layout.fillWidth: true
        Layout.preferredHeight: 82 * AppMetrics.scale

        RowLayout {
            anchors.fill: parent
            anchors.margins: AppMetrics.cardPadding
            spacing: AppMetrics.gap

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 2 * AppMetrics.scale
                Text {
                    text: I18n.t("开机显示教程指引")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.body
                    font.weight: Font.DemiBold
                }
                Text {
                    text: I18n.t("开启后，每次启动都会询问是否查看首页教程")
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.caption
                }
            }
            AppSwitch {
                objectName: "showHomeTutorialOnStartupSwitch"
                checked: backend.settings.show_home_tutorial_on_startup ?? false
                onToggled: backend.setShowHomeTutorialOnStartup(checked)
            }
            PrimaryButton {
                objectName: "startHomeTutorialButton"
                text: I18n.t("立即查看首页教程")
                onClicked: window.startHomeTutorial()
            }
        }
    }

    RowLayout {
        Layout.fillWidth: true
        Layout.fillHeight: true
        spacing: AppMetrics.gap

        AppCard {
            Layout.fillWidth: true
            Layout.fillHeight: true
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                Text {
                    text: I18n.t("UI 运行日志")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.cardTitle
                    font.weight: Font.DemiBold
                }
                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    TextArea {
                        readOnly: true
                        selectByMouse: true
                        wrapMode: TextEdit.WrapAnywhere
                        text: backend.developerUiLog
                        color: Theme.textSecondary
                        font.family: "monospace"
                        font.pixelSize: AppMetrics.caption
                        background: Rectangle { color: Theme.surfaceMuted; radius: Theme.radiusSmall }
                    }
                }
            }
        }

        AppCard {
            Layout.fillWidth: true
            Layout.fillHeight: true
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                Text {
                    text: I18n.t("ROS 2 公共状态")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.cardTitle
                    font.weight: Font.DemiBold
                }
                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    TextArea {
                        readOnly: true
                        selectByMouse: true
                        wrapMode: TextEdit.WrapAnywhere
                        text: backend.developerRosLog
                        color: Theme.textSecondary
                        font.family: "monospace"
                        font.pixelSize: AppMetrics.caption
                        background: Rectangle { color: Theme.surfaceMuted; radius: Theme.radiusSmall }
                    }
                }
            }
        }
    }
}

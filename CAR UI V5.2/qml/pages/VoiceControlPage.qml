import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

Page {
    id: root
    objectName: embedded ? "embeddedVoiceControl" : "voiceControlPage"
    property bool embedded: false
    signal exitRequested()
    background: Rectangle { color: Theme.pageBackground }
    readonly property string voiceState: backend.snapshot.voice_state ?? "LISTENING"
    function stateActive(stateName) { return voiceState === stateName }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: root.embedded ? 0 : AppMetrics.margin * 1.5
        spacing: AppMetrics.gap

        PageHeader {
            Layout.fillWidth: true
            showCloseButton: true
            closeObjectName: root.embedded ? "exitEmbeddedVoiceButton" : "pageCloseButton"
            closeAction: function() {
                if (root.embedded)
                    root.exitRequested()
                else
                    window.goBack()
            }
            title: I18n.t("语音控制")
            subtitle: I18n.t("查看谁在说话并控制语音功能")
            SecondaryButton {
                objectName: "voiceprintManagerButton"
                text: I18n.t("语音人员管理")
                implicitWidth: 176 * AppMetrics.scale
                accent: Theme.purple
                onClicked: window.pushPage("VoiceprintManagerPage.qml")
            }
            AppSwitch {
                checked: backend.snapshot.voice_control_enabled ?? false
                onToggled: backend.setVoiceEnabled(checked)
            }
        }

        Rectangle {
            visible: !(backend.snapshot.voice_control_enabled ?? false)
            Layout.fillWidth: true
            implicitHeight: 48 * AppMetrics.scale
            radius: 8 * AppMetrics.scale
            color: Theme.purpleSoft
            Text {
                anchors.centerIn: parent
                text: I18n.t("语音控制未启用")
                color: Theme.purple
                font.pixelSize: AppMetrics.body
                font.bold: true
            }
        }

        AppCard {
            objectName: "currentSpeakerCard"
            Layout.fillWidth: true
            Layout.fillHeight: true
            ColumnLayout {
                anchors.centerIn: parent
                width: parent.width - AppMetrics.margin * 4
                spacing: AppMetrics.unit
                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: I18n.t("当前使用语音的人")
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.body
                }
                Text {
                    Layout.fillWidth: true
                    text: I18n.t(backend.snapshot.speaker_name ?? "未知")
                    color: Theme.textPrimary
                    font.pixelSize: root.embedded
                        ? AppMetrics.title * 1.25
                        : Math.max(AppMetrics.title * 1.6, 42 * AppMetrics.scale)
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    elide: Text.ElideRight
                }
                Rectangle {
                    Layout.alignment: Qt.AlignHCenter
                    Layout.preferredWidth: Math.min(parent.width * 0.56, 560 * AppMetrics.scale)
                    implicitHeight: 1
                    color: Theme.border
                }
                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: I18n.t("已登记的人员")
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.small
                }
                Text {
                    Layout.fillWidth: true
                    text: I18n.t(backend.snapshot.speaker_voiceprint ?? backend.snapshot.speaker_name ?? "未知")
                    color: Theme.purple
                    font.pixelSize: AppMetrics.title
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    elide: Text.ElideRight
                }
            }
        }

        AppCard {
            Layout.fillWidth: true
            implicitHeight: 178 * AppMetrics.scale
            RowLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.gap * 1.5
                spacing: AppMetrics.gap

                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.preferredWidth: 1
                    Layout.minimumWidth: 0
                    Layout.fillHeight: true
                    Text {
                        text: I18n.t("语音状态")
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.body
                        font.bold: true
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: AppMetrics.gap
                        Repeater {
                            model: [
                                { code: "LISTENING", label: "正在听" },
                                { code: "SPEAKING", label: "正在回答" },
                                { code: "READY", label: "可以说话" }
                            ]
                            Rectangle {
                                required property var modelData
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                radius: 10 * AppMetrics.scale
                                color: root.stateActive(modelData.code) ? Theme.purpleSoft : Theme.surfaceMuted
                                border.color: root.stateActive(modelData.code) ? Theme.purple : Theme.border
                                border.width: root.stateActive(modelData.code)
                                    ? Theme.borderWidthStrong : Theme.borderWidth
                                Text {
                                    anchors.centerIn: parent
                                    width: parent.width - AppMetrics.unit * 2
                                    text: I18n.t(modelData.label)
                                    color: root.stateActive(modelData.code) ? Theme.purple : Theme.textMuted
                                    font.pixelSize: AppMetrics.small
                                    font.bold: root.stateActive(modelData.code)
                                    horizontalAlignment: Text.AlignHCenter
                                    elide: Text.ElideRight
                                }
                            }
                        }
                    }
                }

                Rectangle { Layout.fillHeight: true; implicitWidth: 1; color: Theme.border }

                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.preferredWidth: 1
                    Layout.minimumWidth: 0
                    Layout.fillHeight: true
                    spacing: AppMetrics.unit
                    Text {
                        Layout.fillWidth: true
                        text: I18n.t("允许陌生人使用语音控制")
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.body
                        font.bold: true
                        wrapMode: Text.WordWrap
                    }
                    Text {
                        Layout.fillWidth: true
                        text: I18n.t("开启后，未登记的人员也能对机器人说话")
                        color: Theme.textMuted
                        font.pixelSize: AppMetrics.small
                        wrapMode: Text.WordWrap
                    }
                    Item { Layout.fillHeight: true }
                    AppSwitch {
                        checked: backend.settings.unknown_voice_allowed ?? true
                        onToggled: backend.setUnknownVoiceAllowed(checked)
                    }
                }
            }
        }
    }
}

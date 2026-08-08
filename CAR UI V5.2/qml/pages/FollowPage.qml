import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

Page {
    id: root
    objectName: embedded ? "embeddedFollowControl" : "followControlPage"
    property bool embedded: false
    signal exitRequested()
    property string selectedActor: backend.snapshot.follow_target || "Actor1"
    function friendlyActorName(actorId) {
        var raw = String(actorId || "")
        var match = raw.match(/Actor\s*(\d+)/i)
        return match ? I18n.t("人员") + match[1] : (raw || I18n.t("尚未选择"))
    }
    readonly property real followDistance: Number(
        backend.settings.parameters?.follow_distance ?? 1.0
    )
    background: Rectangle { color: Theme.pageBackground }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: root.embedded ? 0 : AppMetrics.margin
        spacing: AppMetrics.gap

        PageHeader {
            Layout.fillWidth: true
            title: I18n.t("视觉跟随")
            subtitle: I18n.t("选择一个人，让机器人自动跟随")
            StatusBadge {
                status: backend.snapshot.visual_follow_enabled
                    ? (backend.snapshot.follow_state ?? "NORMAL")
                    : "UNKNOWN"
            }
            AppSwitch {
                checked: backend.snapshot.visual_follow_enabled ?? false
                onToggled: backend.setFollowEnabled(checked)
            }
            SecondaryButton {
                objectName: "exitEmbeddedFollowButton"
                visible: root.embedded
                compact: true
                text: I18n.t("返回控制首页")
                onClicked: root.exitRequested()
            }
        }

        Rectangle {
            visible: !(backend.snapshot.visual_follow_enabled ?? false)
            Layout.fillWidth: true
            implicitHeight: 48 * AppMetrics.scale
            radius: Theme.radiusSmall
            color: Theme.warningSoft
            Text {
                anchors.centerIn: parent
                width: parent.width - AppMetrics.gap * 2
                text: I18n.t("视觉跟随尚未开启，仍可先选择要跟随的人")
                color: Theme.warning
                font.pixelSize: AppMetrics.body
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
            }
        }

        CameraPlaceholder {
            Layout.fillWidth: true
            Layout.fillHeight: true
            selectedActor: root.selectedActor
            onActorSelected: function(actorId) {
                root.selectedActor = actorId
                backend.selectActor(actorId)
            }
        }

        AppCard {
            Layout.fillWidth: true
            implicitHeight: 124 * AppMetrics.scale
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.gap
                spacing: AppMetrics.unit

                RowLayout {
                    Layout.fillWidth: true
                    spacing: AppMetrics.gap
                    Text {
                        text: I18n.t((backend.snapshot.follow_state ?? "IDLE") === "FOLLOWING" ? "正在跟随" : "等待开始")
                        color: (backend.snapshot.follow_state ?? "IDLE") === "FOLLOWING" ? Theme.success : Theme.textSecondary
                        font.pixelSize: AppMetrics.body
                        font.bold: true
                    }
                    Text {
                        Layout.fillWidth: true
                        text: I18n.t("正在跟随的人") + "  " + root.friendlyActorName(root.selectedActor)
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.body
                        elide: Text.ElideRight
                    }
                    Text {
                        visible: root.width >= 720
                        text: I18n.t("转向速度") + "  0.5 m/s"
                        color: Theme.textMuted
                        font.pixelSize: AppMetrics.small
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: AppMetrics.unit
                    Text {
                        text: I18n.t("跟随距离")
                        color: Theme.textSecondary
                        font.pixelSize: AppMetrics.small
                    }
                    AppSlider {
                        id: followControlDistanceSlider
                        objectName: "followControlDistanceSlider"
                        Layout.fillWidth: true
                        from: 0.5
                        to: 10.0
                        stepSize: 0.1
                        value: root.followDistance
                        onPressedChanged: if (!pressed) {
                            backend.setParameter(
                                "follow_distance", Number(value.toFixed(1))
                            )
                        }
                    }
                    Text {
                        text: followControlDistanceSlider.value.toFixed(1) + " m"
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.small
                        font.bold: true
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.gap

            SecondaryButton {
                Layout.fillWidth: true
                text: I18n.t("换一个人")
                accent: Theme.primary
                onClicked: {
                    var actors = backend.snapshot.detected_actors ?? []
                    if (!actors.length)
                        return
                    var next = 0
                    for (var i = 0; i < actors.length; ++i) {
                        if (actors[i].id === root.selectedActor)
                            next = (i + 1) % actors.length
                    }
                    root.selectedActor = actors[next].id
                    backend.selectActor(root.selectedActor)
                }
            }
            PrimaryButton {
                Layout.fillWidth: true
                text: I18n.t("开始跟随")
                enabled: backend.snapshot.visual_follow_enabled ?? false
                onClicked: backend.startFollowing(root.selectedActor)
            }
            DangerButton {
                Layout.fillWidth: true
                text: I18n.t("结束跟随")
                onClicked: backend.stopFollowing()
            }
        }
    }
}

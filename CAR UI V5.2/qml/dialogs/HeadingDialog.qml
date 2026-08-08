import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppDialog {
    id: root
    property string editMode: "saved"
    property var point: ({})
    property real draftYaw: 0
    signal yawAccepted(real yaw)

    parent: Overlay.overlay
    anchors.centerIn: Overlay.overlay
    width: Math.min(760 * AppMetrics.scale, parent ? parent.width * 0.72 : 760)
    height: Math.min(570 * AppMetrics.scale, parent ? parent.height * 0.78 : 570)
    closePolicy: Popup.CloseOnEscape

    function normalized(value) {
        return ((Number(value) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
    }
    function openForPoint(value) {
        editMode = "saved"
        point = value ?? ({})
        draftYaw = normalized(point.yaw ?? 0)
        headingDial.yaw = draftYaw
        open()
    }
    function openForMapGoal() {
        editMode = "map"
        point = backend.mapGoal
        draftYaw = normalized(point.yaw ?? 0)
        headingDial.yaw = draftYaw
        open()
    }
    function openForYaw(value) {
        editMode = "draft"
        point = ({})
        draftYaw = normalized(value)
        headingDial.yaw = draftYaw
        open()
    }
    function saveHeading() {
        if (editMode === "saved")
            backend.updatePointYaw(point.id ?? "", draftYaw)
        else if (editMode === "map")
            backend.setMapGoalYaw(draftYaw)
        else
            yawAccepted(draftYaw)
        close()
    }

    contentItem: ColumnLayout {
        spacing: AppMetrics.gap

        RowLayout {
            Layout.fillWidth: true
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 2 * AppMetrics.scale
                Text {
                    text: I18n.t("设置到达方向")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.title
                    font.weight: Font.DemiBold
                }
                Text {
                    text: root.editMode === "saved"
                        ? I18n.t(root.point.name ?? "目标点")
                        : I18n.t("拖动外环圆点，设置车辆到达后的车头方向")
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.body
                }
            }
            AppIconButton {
                text: "×"
                accessibleName: I18n.t("取消")
                outlined: true
                onClicked: root.close()
            }
        }

        HeadingDial {
            id: headingDial
            Layout.alignment: Qt.AlignHCenter
            Layout.fillHeight: true
            Layout.preferredWidth: height
            onYawEdited: function(value) { root.draftYaw = value }
        }

        Text {
            Layout.alignment: Qt.AlignHCenter
            text: I18n.t("0°向右，90°向上，180°向左，270°向下")
            color: Theme.textSecondary
            font.pixelSize: AppMetrics.small
        }

        RowLayout {
            Layout.fillWidth: true
            Item { Layout.fillWidth: true }
            AppButton {
                text: I18n.t("取消")
                outlined: true
                accent: Theme.textMuted
                onClicked: root.close()
            }
            AppButton {
                objectName: "saveHeadingButton"
                text: I18n.t("保存朝向")
                accent: Theme.primary
                onClicked: root.saveHeading()
            }
        }
    }
}

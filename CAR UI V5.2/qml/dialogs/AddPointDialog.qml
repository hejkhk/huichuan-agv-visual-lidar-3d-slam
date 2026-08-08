import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppInputDialog {
    id: root
    objectName: "addPointDialog"
    parent: window.contentItem
    property var pose: ({x:0,y:0,yaw:0})
    property real draftYaw: 0
    padding: AppMetrics.cardPadding
    dialogWidth: 760 * AppMetrics.scale
    dialogHeight: 410 * AppMetrics.scale
    onOpened: {
        draftYaw = Number(pose.yaw ?? 0)
        nameField.forceActiveFocus()
    }
    onClosed: Qt.inputMethod.hide()
    function dismiss() {
        if (replaceChargingDialog.visible) replaceChargingDialog.close()
        root.close()
    }
    function commitPoint() {
        backend.savePoint(nameField.text, root.pose.x ?? 0, root.pose.y ?? 0, root.draftYaw, chargingSwitch.checked)
        nameField.clear(); chargingSwitch.checked=false; root.close()
    }
    HeadingDialog { id: addPointHeadingDialog; onYawAccepted: function(value) { root.draftYaw = value } }
    ConfirmDialog {
        id: replaceChargingDialog
        title: I18n.t("替换充电点")
        message: I18n.t("当前已有充电点，是否替换？")
        onAcceptedAction: root.commitPoint()
    }
    ColumnLayout { anchors.fill: parent; spacing: AppMetrics.gap
        Text { text: I18n.t("添加新目标点"); font.pixelSize: AppMetrics.title; font.bold: true; color: Theme.textPrimary }
        Text { text: "X: " + Number(root.pose.x ?? 0).toFixed(2) + "    Y: " + Number(root.pose.y ?? 0).toFixed(2); color: Theme.textMuted; font.pixelSize: AppMetrics.body }
        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.unit
            Text { text: "➜"; rotation: -Math.round((((root.draftYaw * 180 / Math.PI) % 360) + 360) % 360); color: Theme.primary; font.pixelSize: AppMetrics.sectionTitle; font.bold: true }
            Text {
                Layout.fillWidth: true
                text: I18n.t("到达后车头朝向") + "：" + Math.round((((root.draftYaw * 180 / Math.PI) % 360) + 360) % 360) + "°"
                color: Theme.textPrimary
                font.pixelSize: AppMetrics.body
            }
            AppButton { text: I18n.t("调整朝向"); outlined: true; onClicked: addPointHeadingDialog.openForYaw(root.draftYaw) }
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.unit
            TextField { id: nameField; objectName: "addPointNameField"; Layout.fillWidth: true; placeholderText: I18n.t("目标点名称"); font.pixelSize: AppMetrics.body }
            KeyboardToggleButton { inputItem: nameField }
        }
        RowLayout { Text { text: I18n.t("设置为充电点"); color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
AppSwitch { id: chargingSwitch } }
        Item { Layout.fillHeight: true }
        RowLayout { Layout.fillWidth:true; Item { Layout.fillWidth:true }
AppButton { text:I18n.t("取消"); accent:Theme.danger; onClicked:root.close() }
AppButton { text:I18n.t("保存"); enabled:nameField.text.trim().length>0; accent:Theme.success; onClicked:{ if (chargingSwitch.checked && backend.hasChargingPoint) replaceChargingDialog.open(); else root.commitPoint() } } }
    }
}

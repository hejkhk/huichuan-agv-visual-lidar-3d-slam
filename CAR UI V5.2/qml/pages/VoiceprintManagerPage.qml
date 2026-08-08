import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"
import "../dialogs"

Page {
    id: root
    property int pageIndex: 0
    readonly property int pageSize: 5
    readonly property int maximumVoiceprints: 10
    readonly property int pageCount: Math.max(1, Math.ceil(backend.voiceprints.length / pageSize))
    property var activeVoice: ({})
    onPageCountChanged: pageIndex = Math.min(pageIndex, pageCount - 1)
    function closeTransient() {
        if (deleteDialog.visible) { deleteDialog.close(); return true }
        if (renameDialog.visible) { renameDialog.close(); return true }
        if (addDialog.visible) { addDialog.close(); return true }
        return false
    }
    background: Rectangle { color: Theme.pageBackground }
    AddVoiceprintDialog { id: addDialog }
    RenameVoiceprintDialog { id: renameDialog; voiceprint: root.activeVoice }
    ConfirmDialog { id: deleteDialog; title: I18n.t("删除声纹"); message: I18n.t("确认删除") + " “" + I18n.t(activeVoice.name ?? "") + "”?"; onAcceptedAction: backend.deleteVoiceprint(activeVoice.id) }
    ColumnLayout { anchors.fill: parent; anchors.margins: AppMetrics.margin; spacing: AppMetrics.gap
        PageHeader { Layout.fillWidth: true; title: I18n.t("声纹管理"); subtitle: backend.voiceprints.length + " / " + maximumVoiceprints }
        ListView { objectName: "voiceprintList"; Layout.fillWidth: true; Layout.fillHeight: true; spacing: AppMetrics.unit; clip: true; model: backend.voiceprints.slice(pageIndex*pageSize,pageIndex*pageSize+pageSize)
            delegate: VoiceprintListItem {
                required property var modelData
                width: ListView.view.width
                voiceprint: modelData
                canMoveUp: (modelData.priority ?? 1) > 1
                canMoveDown: (modelData.priority ?? 1) < backend.voiceprints.length
                onMoveUpRequested: backend.moveVoiceprint(modelData.id, -1)
                onMoveDownRequested: backend.moveVoiceprint(modelData.id, 1)
                onRenameRequested: { root.activeVoice=modelData; renameDialog.open() }
                onDeleteRequested: { root.activeVoice=modelData; deleteDialog.open() }
            }
        }
        Text {
            visible: backend.voiceprints.length >= maximumVoiceprints
            Layout.fillWidth: true
            text: I18n.t("声纹已满，请先删除一个声纹")
            color: Theme.warning
            font.pixelSize: AppMetrics.body
            font.bold: true
            horizontalAlignment: Text.AlignRight
        }
        RowLayout { Layout.fillWidth: true; AppButton { text:I18n.t("上一页"); outlined:true; enabled:pageIndex>0; onClicked:pageIndex-- }
Text { text:I18n.t("第") + " " + (pageIndex+1) + " / " + pageCount + " " + I18n.t("页"); color:Theme.textPrimary; font.pixelSize:AppMetrics.body }
AppButton { text:I18n.t("下一页"); outlined:true; enabled:pageIndex+1<pageCount; onClicked:pageIndex++ }
Item { Layout.fillWidth:true }
AppButton { objectName: "addVoiceprintButton"; text:I18n.t("加入新声纹"); implicitWidth:240*AppMetrics.scale; accent:backend.voiceprints.length >= maximumVoiceprints ? Theme.warning : Theme.success; onClicked:{ if (backend.voiceprints.length >= maximumVoiceprints) backend.notifyVoiceprintLimit(); else addDialog.open() } } }
    }
}

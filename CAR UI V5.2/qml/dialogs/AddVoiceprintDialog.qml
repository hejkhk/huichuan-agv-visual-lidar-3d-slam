import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppInputDialog {
    id:root;objectName:"addVoiceprintDialog"
    parent: window.contentItem
    padding: AppMetrics.cardPadding
    dialogWidth:760*AppMetrics.scale
    dialogHeight:340*AppMetrics.scale
    onOpened: nameField.forceActiveFocus()
    onClosed: {
        Qt.inputMethod.hide()
        if(backend.recordingState==="RECORDING") backend.cancelVoiceprintRecording()
    }
    ColumnLayout{anchors.fill:parent;spacing:AppMetrics.gap
        Text{text:I18n.t("添加新声纹");font.pixelSize:AppMetrics.title;font.bold:true;color:Theme.textPrimary}
        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.unit
            TextField{id:nameField;Layout.fillWidth:true;placeholderText:I18n.t("声纹名称");font.pixelSize:AppMetrics.body}
            KeyboardToggleButton { inputItem: nameField }
        }
        RowLayout{AppButton{text:I18n.t(backend.recordingState==="RECORDING"?"录入中…":"开始录入");implicitWidth:190*AppMetrics.scale;enabled:nameField.text.trim().length>0&&backend.recordingState!=="RECORDING";accent:Theme.purple;onClicked:backend.beginVoiceprint(nameField.text)}
Rectangle{width:14*AppMetrics.scale;height:width;radius:width/2;color:backend.recordingState==="READY"?Theme.success:backend.recordingState==="FAILED"?Theme.danger:Theme.warning}
Text{text:I18n.t(backend.recordingState);color:Theme.textMuted;font.pixelSize:AppMetrics.body}}
        Item { Layout.fillHeight: true }
        RowLayout{Item{Layout.fillWidth:true}
AppButton{text:I18n.t("取消");accent:Theme.danger;onClicked:{backend.cancelVoiceprintRecording();root.close()}}
AppButton{text:I18n.t("保存");enabled:backend.recordingState==="READY";accent:Theme.success;onClicked:{backend.saveVoiceprint(nameField.text);nameField.clear();root.close()}}}
    }
}

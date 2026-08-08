import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

AppInputDialog {
    id: root; property var point: ({})
    parent: window.contentItem
    padding: AppMetrics.cardPadding
    dialogWidth:680*AppMetrics.scale
    dialogHeight:260*AppMetrics.scale
    onOpened: { nameField.text=point.name ?? ""; nameField.forceActiveFocus() }
    onClosed: Qt.inputMethod.hide()
    ColumnLayout{anchors.fill:parent;spacing:AppMetrics.gap;Text{text:I18n.t("重命名目标点");font.pixelSize:AppMetrics.title;font.bold:true;color:Theme.textPrimary}
RowLayout {
    Layout.fillWidth: true
    spacing: AppMetrics.unit
    TextField{id:nameField;Layout.fillWidth:true;font.pixelSize:AppMetrics.body}
    KeyboardToggleButton { inputItem: nameField }
}
Item{Layout.fillHeight:true}
RowLayout{Item{Layout.fillWidth:true}
AppButton{text:I18n.t("取消");accent:Theme.textMuted;onClicked:root.close()}
AppButton{text:I18n.t("保存");accent:Theme.success;enabled:nameField.text.trim().length>0;onClicked:{backend.renamePoint(point.id,nameField.text);root.close()}}}}
}

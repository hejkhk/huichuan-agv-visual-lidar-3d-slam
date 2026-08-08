import QtQuick
import QtQuick.Layouts
import ".."

Item {
    id: root
    property string title: ""
    property string description: ""
    property string value: ""
    default property alias action: actionSlot.data
    implicitHeight: 68 * AppMetrics.scale

    RowLayout {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: divider.top
        spacing: AppMetrics.gap
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 3 * AppMetrics.scale
            Text { text: root.title; color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.weight: Font.Medium; elide: Text.ElideRight }
            Text { visible: root.description.length > 0; text: root.description; color: Theme.textMuted; font.pixelSize: AppMetrics.caption; elide: Text.ElideRight }
        }
        Text { visible: root.value.length > 0; text: root.value; color: Theme.textSecondary; font.pixelSize: AppMetrics.body }
        RowLayout { id: actionSlot }
    }
    Rectangle { id: divider; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: Theme.divider }
}

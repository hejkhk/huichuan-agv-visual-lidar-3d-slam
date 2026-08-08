import QtQuick
import QtQuick.Layouts
import ".."

Item {
    id: root
    property string label: ""
    property string value: ""
    property string status: ""
    property bool showDivider: true
    implicitHeight: 44 * AppMetrics.scale

    RowLayout {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: divider.top
        spacing: AppMetrics.unit
        StatusDot { visible: root.status.length > 0; status: root.status }
        Text { Layout.fillWidth: true; text: root.label; color: Theme.textSecondary; font.pixelSize: AppMetrics.body; elide: Text.ElideRight }
        Text { text: root.value; color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.weight: Font.Medium; horizontalAlignment: Text.AlignRight }
        StatusBadge { visible: root.status.length > 0; status: root.status }
    }
    Rectangle { id: divider; visible: root.showDivider; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: Theme.divider }
}

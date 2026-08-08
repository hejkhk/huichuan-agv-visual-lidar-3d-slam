import QtQuick
import QtQuick.Layouts
import ".."

RowLayout {
    id: root
    property string label: ""
    property string status: "NORMAL"
    property color textColor: Theme.textSecondary
    spacing: AppMetrics.unit

    StatusDot { status: root.status }
    Text { Layout.fillWidth: true; text: root.label; color: root.textColor; font.pixelSize: AppMetrics.small; elide: Text.ElideRight }
    StatusBadge { status: root.status }
}

import QtQuick
import QtQuick.Layouts
import ".."

RowLayout {
    id: root
    property string title: ""
    property string subtitle: ""
    default property alias actions: actionSlot.data
    spacing: AppMetrics.unit
    ColumnLayout {
        Layout.fillWidth: true
        spacing: 2 * AppMetrics.scale
        Text { text: root.title; color: Theme.textPrimary; font.pixelSize: AppMetrics.sectionTitle; font.weight: Font.DemiBold; elide: Text.ElideRight }
        Text { visible: root.subtitle.length > 0; text: root.subtitle; color: Theme.textMuted; font.pixelSize: AppMetrics.caption; elide: Text.ElideRight }
    }
    RowLayout { id: actionSlot; spacing: AppMetrics.unit }
}

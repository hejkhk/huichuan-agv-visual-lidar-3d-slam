import QtQuick
import QtQuick.Layouts
import ".."

ColumnLayout {
    id: root
    property string title: ""
    property string description: ""
    spacing: AppMetrics.unit
    Text { Layout.alignment: Qt.AlignHCenter; text: root.title; color: Theme.textSecondary; font.pixelSize: AppMetrics.sectionTitle; font.weight: Font.DemiBold; horizontalAlignment: Text.AlignHCenter }
    Text { Layout.alignment: Qt.AlignHCenter; Layout.maximumWidth: 420 * AppMetrics.scale; text: root.description; color: Theme.textMuted; font.pixelSize: AppMetrics.small; wrapMode: Text.WordWrap; horizontalAlignment: Text.AlignHCenter }
}

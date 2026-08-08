import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."

Rectangle {
    id: root
    property var options: []
    property int currentIndex: 0
    signal selected(int index)
    implicitHeight: AppMetrics.touch
    implicitWidth: 240 * AppMetrics.scale
    radius: Theme.radiusSmall * AppMetrics.scale
    color: Theme.surfaceMuted
    border.color: Theme.border
    border.width: Theme.borderWidth
    RowLayout {
        anchors.fill: parent
        anchors.margins: 3 * AppMetrics.scale
        spacing: 3 * AppMetrics.scale
        Repeater {
            model: root.options
            Button {
                required property var modelData
                required property int index
                Layout.fillWidth: true
                Layout.fillHeight: true
                text: modelData
                scale: root.currentIndex === index && Performance.smooth ? 1.025 : 1
                Behavior on scale {
                    NumberAnimation { duration: Performance.shortDuration; easing.type: Easing.OutBack }
                }
                onClicked: { root.currentIndex = index; root.selected(index) }
                background: Rectangle {
                    radius: 7 * AppMetrics.scale
                    color: root.currentIndex === index ? Theme.surfaceElevated : "transparent"
                    border.color: root.currentIndex === index ? Theme.border : "transparent"
                    border.width: root.currentIndex === index ? Theme.borderWidth : 0
                    Behavior on color { ColorAnimation { duration: Performance.shortDuration } }
                }
                contentItem: Text {
                    text: parent.text
                    color: root.currentIndex === index ? Theme.primary : Theme.textSecondary
                    font.pixelSize: AppMetrics.body
                    font.weight: root.currentIndex === index ? Font.DemiBold : Font.Normal
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }
    }
}

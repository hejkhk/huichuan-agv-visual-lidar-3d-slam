import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."

Button {
    id: root
    property url iconSource
    property color accent: Theme.primary
    padding: AppMetrics.unit
    hoverEnabled: true
    background: Rectangle {
        radius: 18 * AppMetrics.scale
        color: root.down ? Qt.darker(root.accent, 1.10) : root.hovered ? Qt.lighter(root.accent, 1.06) : root.accent
        border.color: Qt.darker(root.accent, 1.18)
        border.width: Theme.borderWidth
    }
    contentItem: ColumnLayout {
        spacing: AppMetrics.unit * 0.6
        Image {
            id: featureIcon
            Layout.alignment: Qt.AlignHCenter
            Layout.preferredHeight: Math.min(76 * AppMetrics.scale, root.height * 0.58)
            Layout.preferredWidth: Layout.preferredHeight
            source: root.iconSource
            sourceSize.width: Math.ceil(width * 2 * Performance.imageScale)
            sourceSize.height: Math.ceil(height * 2 * Performance.imageScale)
            fillMode: Image.PreserveAspectFit
            smooth: !Performance.lowPower
            mipmap: false
        }
        Text {
            Layout.fillWidth: true
            text: root.text
            color: "white"
            font.pixelSize: AppMetrics.body
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            maximumLineCount: 2
            elide: Text.ElideRight
        }
    }
}

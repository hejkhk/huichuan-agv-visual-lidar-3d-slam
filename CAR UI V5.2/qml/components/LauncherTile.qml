import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."

Button {
    id: root
    property url iconSource
    property color accent: Theme.primary
    padding: 0
    hoverEnabled: true
    background: Item {}
    contentItem: ColumnLayout {
        spacing: 7 * AppMetrics.scale
        Item {
            Layout.fillHeight: true
            Layout.fillWidth: true
            Rectangle {
                id: iconBadge
                anchors.centerIn: parent
                width: Math.min(parent.width * 0.72, parent.height * 0.78, 122 * AppMetrics.scale)
                height: width
                radius: 20 * AppMetrics.scale
                color: root.down ? Qt.darker(root.accent, 1.12) : root.hovered ? Qt.lighter(root.accent, 1.07) : root.accent
                border.color: Qt.darker(root.accent, 1.16)
                border.width: Theme.borderWidth
                scale: root.down ? 0.94 : 1
                Behavior on scale { NumberAnimation { duration: Performance.instantDuration } }
                Image {
                    id: launcherIcon
                    anchors.centerIn: parent
                    width: parent.width * 0.58
                    height: width
                    source: root.iconSource
                    // SVG files declare a 16 px intrinsic size. Request a texture
                    // at the actual display size so Qt does not upscale 16 px.
                    sourceSize.width: Math.ceil(width * 2 * Performance.imageScale)
                    sourceSize.height: Math.ceil(height * 2 * Performance.imageScale)
                    fillMode: Image.PreserveAspectFit
                    smooth: !Performance.lowPower
                    mipmap: false
                }
            }
        }
        Text {
            Layout.fillWidth: true
            text: root.text
            color: Theme.textPrimary
            font.pixelSize: AppMetrics.body
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            maximumLineCount: 2
            elide: Text.ElideRight
        }
    }
}

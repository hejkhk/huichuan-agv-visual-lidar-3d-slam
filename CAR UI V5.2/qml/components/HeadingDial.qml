pragma ComponentBehavior: Bound
import QtQuick
import ".."

Item {
    id: root
    property real yaw: 0
    property bool interactive: true
    readonly property real normalizedYaw: ((yaw % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
    readonly property int degrees: Math.round(normalizedYaw * 180 / Math.PI) % 360
    signal yawEdited(real yaw)

    implicitWidth: 310 * AppMetrics.scale
    implicitHeight: implicitWidth

    function updateFromPoint(px, py) {
        var dx = px - width / 2
        var dy = height / 2 - py
        if (Math.abs(dx) + Math.abs(dy) < 4)
            return
        yaw = ((Math.atan2(dy, dx) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
        yawEdited(yaw)
    }

    Rectangle {
        id: ring
        anchors.centerIn: parent
        width: Math.min(parent.width, parent.height) * 0.88
        height: width
        radius: width / 2
        color: Theme.surfaceMuted
        border.color: Theme.primary
        border.width: Theme.borderWidthStrong

        Repeater {
            model: 12
            Item {
                id: tick
                required property int index
                anchors.centerIn: parent
                width: ring.width
                height: ring.height
                rotation: tick.index * 30
                Rectangle {
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: parent.top
                    anchors.topMargin: 7 * AppMetrics.scale
                    width: tick.index % 3 === 0 ? 3 * AppMetrics.scale : 2 * AppMetrics.scale
                    height: tick.index % 3 === 0 ? 13 * AppMetrics.scale : 8 * AppMetrics.scale
                    radius: width / 2
                    color: tick.index % 3 === 0 ? Theme.primary : Theme.textMuted
                }
            }
        }

        Text {
            anchors.horizontalCenter: parent.right
            anchors.verticalCenter: parent.verticalCenter
            text: "0°"
            color: Theme.textSecondary
            font.pixelSize: AppMetrics.caption
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.top
            text: "90°"
            color: Theme.textSecondary
            font.pixelSize: AppMetrics.caption
        }
        Text {
            anchors.horizontalCenter: parent.left
            anchors.verticalCenter: parent.verticalCenter
            text: "180°"
            color: Theme.textSecondary
            font.pixelSize: AppMetrics.caption
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.bottom
            text: "270°"
            color: Theme.textSecondary
            font.pixelSize: AppMetrics.caption
        }

        Item {
            id: vehicle
            anchors.centerIn: parent
            width: ring.width * 0.34
            height: ring.width * 0.22
            rotation: -root.degrees

            Rectangle {
                anchors.fill: parent
                radius: 11 * AppMetrics.scale
                color: Theme.primary
                border.color: Theme.surface
                border.width: Theme.borderWidthStrong
            }
            Rectangle {
                anchors.left: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                width: parent.width * 0.72
                height: 5 * AppMetrics.scale
                radius: height / 2
                color: "white"
            }
            Text {
                anchors.left: parent.right
                anchors.leftMargin: -7 * AppMetrics.scale
                anchors.verticalCenter: parent.verticalCenter
                text: "▶"
                color: "white"
                font.pixelSize: AppMetrics.sectionTitle
            }
            Text {
                anchors.centerIn: parent
                text: "AMR"
                color: "white"
                font.pixelSize: AppMetrics.caption
                font.bold: true
            }
        }

        Rectangle {
            id: directionLine
            anchors.left: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            width: ring.width * 0.38
            height: 3 * AppMetrics.scale
            radius: height / 2
            color: Theme.primary
            transformOrigin: Item.Left
            rotation: -root.degrees
        }

        Rectangle {
            id: knob
            width: 30 * AppMetrics.scale
            height: width
            radius: width / 2
            color: Theme.primary
            border.color: Theme.surface
            border.width: Theme.borderWidthStrong
            x: ring.width / 2 + ring.width * 0.43 * Math.cos(root.normalizedYaw) - width / 2
            y: ring.height / 2 - ring.height * 0.43 * Math.sin(root.normalizedYaw) - height / 2
        }

        MouseArea {
            anchors.fill: parent
            enabled: root.interactive
            preventStealing: true
            onPressed: function(mouse) { root.updateFromPoint(mouse.x, mouse.y) }
            onPositionChanged: function(mouse) {
                if (pressed)
                    root.updateFromPoint(mouse.x, mouse.y)
            }
        }
    }

    Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        width: degreeText.implicitWidth + 30 * AppMetrics.scale
        height: 38 * AppMetrics.scale
        radius: height / 2
        color: Theme.primarySoft
        Text {
            id: degreeText
            anchors.centerIn: parent
            text: root.degrees + "°"
            color: Theme.primary
            font.pixelSize: AppMetrics.sectionTitle
            font.bold: true
        }
    }
}

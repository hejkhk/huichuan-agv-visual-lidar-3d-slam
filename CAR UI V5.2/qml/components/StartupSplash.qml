import QtQuick
import QtQuick.Layouts
import ".."

Rectangle {
    id: root
    signal finished()

    property bool darkMode: false
    property int durationMs: 1000 + Math.floor(Math.random() * 2001)
    property bool dismissing: false
    readonly property int exitDurationMs: Performance.smooth ? 320 : 180

    color: darkMode ? "#000000" : "#FFFFFF"
    opacity: 1

    function begin() {
        if (Performance.lowPower) {
            logoRow.opacity = 1
            titleText.opacity = 1
            spinner.opacity = 1
        } else {
            introAnimation.start()
        }
        dismissTimer.start()
    }

    Component.onCompleted: begin()

    SequentialAnimation {
        id: introAnimation
        PauseAnimation { duration: Performance.smooth ? 90 : 40 }
        ParallelAnimation {
            NumberAnimation {
                target: logoRow
                property: "opacity"
                from: 0
                to: 1
                duration: Performance.smooth ? 440 : 260
                easing.type: Easing.OutCubic
            }
            NumberAnimation {
                target: logoRow
                property: "scale"
                from: Performance.smooth ? 0.92 : 0.97
                to: 1
                duration: Performance.smooth ? 520 : 300
                easing.type: Easing.OutCubic
            }
            NumberAnimation {
                target: titleText
                property: "opacity"
                from: 0
                to: 1
                duration: Performance.smooth ? 420 : 240
            }
            NumberAnimation {
                target: spinner
                property: "opacity"
                from: 0
                to: 1
                duration: Performance.smooth ? 420 : 240
            }
        }
    }

    Timer {
        id: dismissTimer
        interval: Performance.lowPower
            ? root.durationMs
            : Math.max(680, root.durationMs - root.exitDurationMs)
        onTriggered: {
            root.dismissing = true
            if (Performance.lowPower)
                root.finished()
            else
                exitAnimation.start()
        }
    }

    NumberAnimation {
        id: exitAnimation
        target: root
        property: "opacity"
        from: 1
        to: 0
        duration: root.exitDurationMs
        easing.type: Easing.InCubic
        onFinished: root.finished()
    }

    ColumnLayout {
        anchors.centerIn: parent
        width: Math.min(parent.width * 0.82, 1100 * AppMetrics.scale)
        spacing: 26 * AppMetrics.scale

        RowLayout {
            id: logoRow
            Layout.fillWidth: true
            Layout.preferredHeight: Math.min(
                root.height * 0.42, 390 * AppMetrics.scale
            )
            spacing: 30 * AppMetrics.scale
            opacity: 0
            transformOrigin: Item.Center

            Image {
                Layout.fillWidth: true
                Layout.fillHeight: true
                source: root.darkMode
                    ? "../../assets/branding/wensihuitong-dark.png"
                    : "../../assets/branding/wensihuitong-light.png"
                fillMode: Image.PreserveAspectFit
                sourceSize.width: Math.ceil(480 * AppMetrics.scale)
                sourceSize.height: Math.ceil(480 * AppMetrics.scale)
                asynchronous: true
                cache: true
            }

            Text {
                text: "&"
                color: root.darkMode ? "#FFFFFF" : "#111827"
                font.pixelSize: Math.max(
                    34 * AppMetrics.scale, AppMetrics.title * 1.45
                )
                font.weight: Font.Light
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }

            Image {
                Layout.fillWidth: true
                Layout.fillHeight: true
                source: root.darkMode
                    ? "../../assets/branding/hongxindeli-dark.png"
                    : "../../assets/branding/hongxindeli-light.png"
                fillMode: Image.PreserveAspectFit
                sourceSize.width: Math.ceil(480 * AppMetrics.scale)
                sourceSize.height: Math.ceil(480 * AppMetrics.scale)
                asynchronous: true
                cache: true
            }
        }

        Text {
            id: titleText
            Layout.alignment: Qt.AlignHCenter
            text: "AMR 操作系统"
            color: root.darkMode ? "#FFFFFF" : "#111827"
            font.pixelSize: Math.max(
                24 * AppMetrics.scale, AppMetrics.title * 1.1
            )
            font.weight: Font.DemiBold
            font.letterSpacing: 2 * AppMetrics.scale
            opacity: 0
        }

        Item {
            id: spinner
            Layout.alignment: Qt.AlignHCenter
            Layout.preferredWidth: 42 * AppMetrics.scale
            Layout.preferredHeight: width
            opacity: 0

            Repeater {
                model: 8
                Rectangle {
                    required property int index
                    width: 6 * AppMetrics.scale
                    height: 13 * AppMetrics.scale
                    radius: width / 2
                    color: root.darkMode ? "#FFFFFF" : "#1F6FB2"
                    opacity: 0.20 + index * 0.09
                    anchors.centerIn: parent
                    transform: [
                        Rotation {
                            angle: index * 45
                            origin.x: 3 * AppMetrics.scale
                            origin.y: 21 * AppMetrics.scale
                        },
                        Translate { y: -14 * AppMetrics.scale }
                    ]
                }
            }

            RotationAnimator on rotation {
                from: 0
                to: 360
                duration: Performance.smooth ? 720 : 920
                loops: Animation.Infinite
                running: spinner.visible && !Performance.lowPower
            }
        }
    }
}

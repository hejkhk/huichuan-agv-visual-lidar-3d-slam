import QtQuick
import ".."

Item {
    id: root
    objectName: "gamepadFocusView"
    property var step: ({ focusX: 0.5, focusY: 0.5, scale: 1,
                          x: 0.05, y: 0.05, w: 0.9, h: 0.9,
                          demo: "overview" })
    property bool intro: true
    property int phase: 0
    readonly property int motionDuration: Performance.lowPower
        ? 0 : (Performance.smooth ? 760 : 600)
    readonly property bool flashOn: phase % 2 === 0
    readonly property var focusSpec: {
        var demo = String(step.demo || "")
        if (demo === "normal") {
            return phase < 8
                ? { focusX: 0.22, focusY: 0.40, x: 0.14, y: 0.28, w: 0.18, h: 0.24 }
                : { focusX: 0.565, focusY: 0.56, x: 0.49, y: 0.47, w: 0.16, h: 0.20 }
        }
        if (demo === "mapping") {
            return phase < 4
                ? { focusX: 0.68, focusY: 0.30, x: 0.64, y: 0.24, w: 0.09, h: 0.13 }
                : { focusX: 0.337, focusY: 0.56, x: 0.27, y: 0.47, w: 0.15, h: 0.20 }
        }
        return step
    }
    readonly property string highlightName: {
        var demo = String(step.demo || "")
        if (intro || demo === "overview" || demo === "complete")
            return ""
        if (demo === "dpad") {
            var directions = ["dpad-up", "dpad-down",
                              "dpad-left", "dpad-right"]
            return directions[Math.floor(phase / 2) % directions.length]
        }
        if (demo === "estop")
            return phase >= 4 ? "estop-button" : ""
        if (demo === "gear")
            return "gear-button"
        if (demo === "normal") {
            if (phase < 8) {
                var normalDirections = ["dpad-up", "dpad-down",
                                        "dpad-left", "dpad-right"]
                return normalDirections[Math.floor(phase / 2) % 4]
            }
            return "right-stick"
        }
        if (demo === "mapping")
            return phase < 4 ? "gear-button" : "left-stick"
        return ""
    }
    readonly property string highlightSource: highlightName === ""
        ? "" : "../../assets/tutorial/highlights/" + highlightName + ".png"

    Item {
        id: imageLayer
        objectName: "tutorialImageLayer"
        width: Math.min(root.width * 0.94, root.height * 1.5 * 0.94)
        height: width / 1.5
        transformOrigin: Item.TopLeft
        scale: root.intro ? 0.86 : Number(root.step.scale || 1)
        x: root.width / 2
            - Number(root.focusSpec.focusX || 0.5) * width * scale
        y: root.height / 2
            - Number(root.focusSpec.focusY || 0.5) * height * scale

        Behavior on x {
            NumberAnimation {
                duration: root.motionDuration
                easing.type: Easing.InOutCubic
            }
        }
        Behavior on y {
            NumberAnimation {
                duration: root.motionDuration
                easing.type: Easing.InOutCubic
            }
        }
        Behavior on scale {
            NumberAnimation {
                duration: root.motionDuration
                easing.type: Easing.InOutCubic
            }
        }

        Image {
            anchors.fill: parent
            source: "../../assets/tutorial/gamepad-guide-landscape.png"
            fillMode: Image.Stretch
            smooth: !Performance.lowPower
            mipmap: Performance.smooth
            sourceSize.width: Math.ceil(width * Performance.imageScale)
            sourceSize.height: Math.ceil(height * Performance.imageScale)
            cache: true
        }

        Image {
            id: buttonHighlight
            anchors.fill: parent
            source: root.highlightSource
            visible: source !== "" && root.flashOn
            opacity: 0.96
            fillMode: Image.Stretch
            smooth: !Performance.lowPower
            sourceSize.width: Math.ceil(width * Performance.imageScale)
            sourceSize.height: Math.ceil(height * Performance.imageScale)
            cache: false
        }

        Item {
            id: focusMask
            anchors.fill: parent
            visible: !root.intro
            readonly property real fx: Number(root.focusSpec.x || 0) * width
            readonly property real fy: Number(root.focusSpec.y || 0) * height
            readonly property real fw: Number(root.focusSpec.w || 1) * width
            readonly property real fh: Number(root.focusSpec.h || 1) * height
            readonly property color shade: Theme.darkMode
                ? "#99000000" : "#92FFFFFF"

            Rectangle { x: 0; y: 0; width: parent.width; height: focusMask.fy; color: focusMask.shade }
            Rectangle { x: 0; y: focusMask.fy; width: focusMask.fx; height: focusMask.fh; color: focusMask.shade }
            Rectangle {
                x: focusMask.fx + focusMask.fw; y: focusMask.fy
                width: Math.max(0, parent.width - x); height: focusMask.fh
                color: focusMask.shade
            }
            Rectangle {
                x: 0; y: focusMask.fy + focusMask.fh; width: parent.width
                height: Math.max(0, parent.height - y); color: focusMask.shade
            }
            Rectangle {
                x: focusMask.fx; y: focusMask.fy
                width: focusMask.fw; height: focusMask.fh
                radius: 14 * AppMetrics.scale / imageLayer.scale
                color: "transparent"
                border.width: Math.max(2, 4 * AppMetrics.scale / imageLayer.scale)
                border.color: Theme.primary
                opacity: Performance.lowPower ? 1 : (root.flashOn ? 1 : 0.66)
                Behavior on opacity {
                    NumberAnimation {
                        duration: Performance.lowPower ? 0
                            : (Performance.smooth ? 260 : 340)
                    }
                }
            }
        }
    }
}

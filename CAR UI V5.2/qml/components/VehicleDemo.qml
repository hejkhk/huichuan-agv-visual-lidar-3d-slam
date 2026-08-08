import QtQuick
import QtQuick.Layouts
import ".."

AppCard {
    id: root
    property string demoType: "overview"
    property int phase: 0
    readonly property bool emergency: demoType === "estop"
    readonly property bool emergencyStopped: emergency && phase >= 6
    readonly property int gearIndex: demoType === "gear"
        ? Math.min(5, Math.floor(phase / 2)) : -1
    readonly property int mappingIndex: demoType === "mapping"
        ? (phase < 7 ? 0 : 1) : -1
    readonly property int directionIndex: {
        if (demoType === "dpad")
            return Math.floor(phase / 2) % 4
        if (demoType === "normal" && phase < 8)
            return Math.floor(phase / 2) % 4
        return -1
    }
    readonly property string motionSymbol: {
        if (emergency)
            return emergencyStopped ? "!" : "↑"
        if (directionIndex >= 0)
            return ["↑", "↓", "↺", "↻"][directionIndex]
        if (demoType === "normal")
            return ["↑", "↖", "↗"][Math.floor((phase - 8) / 2) % 3]
        if (demoType === "mapping")
            return mappingIndex === 0 ? "↑" : "↑↑"
        return demoType === "overview" ? "●" : ""
    }
    readonly property int motionDuration: Performance.lowPower
        ? 0 : (Performance.smooth ? 720 : 600)

    color: emergencyStopped ? Theme.dangerSoft : Theme.surfaceMuted
    clip: true

    Rectangle {
        id: redEdge
        anchors.fill: parent
        radius: Theme.radius
        color: "transparent"
        border.width: 9 * AppMetrics.scale
        border.color: Theme.danger
        visible: root.emergencyStopped
        opacity: Performance.lowPower ? 1 : 0.45
        SequentialAnimation on opacity {
            running: redEdge.visible && Performance.decorativeAnimations
            loops: Animation.Infinite
            NumberAnimation { from: 0.25; to: 1; duration: Performance.smooth ? 360 : 480 }
            NumberAnimation { from: 1; to: 0.25; duration: Performance.smooth ? 360 : 480 }
        }
    }

    Item {
        id: vehicle
        width: 138 * AppMetrics.scale
        height: 184 * AppMetrics.scale
        x: root.emergency
            ? (root.width - width) / 2
            : AppMetrics.cardPadding * 2
        y: {
            if (!root.emergency)
                return (root.height - height) / 2
            var bottom = root.height - height - AppMetrics.cardPadding
            var middle = (root.height - height) / 2
            if (root.phase >= 4)
                return middle
            return bottom + (middle - bottom) * root.phase / 4
        }

        Behavior on y {
            NumberAnimation {
                duration: root.motionDuration
                easing.type: root.emergencyStopped
                    ? Easing.OutCubic : Easing.InOutCubic
            }
        }

        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: body.top
            anchors.bottomMargin: 10 * AppMetrics.scale
            text: root.motionSymbol
            color: root.emergencyStopped ? Theme.danger : Theme.primary
            font.pixelSize: 48 * AppMetrics.scale
            font.bold: true
            scale: Performance.lowPower ? 1 : (root.phase % 2 === 0 ? 1.14 : 0.92)
            Behavior on scale {
                NumberAnimation {
                    duration: Performance.smooth ? 280 : 360
                    easing.type: Easing.InOutCubic
                }
            }
        }

        Rectangle {
            id: body
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            width: parent.width
            height: parent.height * 0.72
            radius: 20 * AppMetrics.scale
            color: root.emergencyStopped ? Theme.danger : Theme.primary
            border.width: 4 * AppMetrics.scale
            border.color: Theme.textPrimary
        }
        Text {
            anchors.centerIn: body
            text: root.emergencyStopped ? "STOP" : "AMR"
            color: "white"
            font.pixelSize: AppMetrics.sectionTitle
            font.bold: true
        }
    }

    Text {
        visible: root.emergencyStopped
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 20 * AppMetrics.scale
        text: "!"
        color: Theme.danger
        font.pixelSize: 68 * AppMetrics.scale
        font.bold: true
        scale: root.phase % 2 === 0 ? 1.2 : 0.9
        Behavior on scale { NumberAnimation { duration: 280 } }
    }

    Column {
        id: information
        visible: !root.emergency || root.emergencyStopped
        x: root.emergency
            ? vehicle.x + vehicle.width + 70 * AppMetrics.scale
            : vehicle.x + vehicle.width + 64 * AppMetrics.scale
        width: Math.max(120, root.width - x - AppMetrics.cardPadding * 2)
        anchors.verticalCenter: parent.verticalCenter
        spacing: 15 * AppMetrics.scale

        Text {
            width: parent.width
            text: root.emergencyStopped ? I18n.t("急停已触发")
                : root.demoType === "gear" ? I18n.t("当前档位")
                : root.demoType === "mapping" ? I18n.t("建图档位演示")
                : root.demoType === "normal" ? I18n.t("普通档位动作")
                : I18n.t("车辆动作示意")
            color: root.emergencyStopped ? Theme.danger : Theme.textPrimary
            font.pixelSize: AppMetrics.sectionTitle
            font.bold: true
            wrapMode: Text.WordWrap
        }

        Row {
            visible: root.demoType === "gear"
            spacing: 13 * AppMetrics.scale
            Repeater {
                model: [
                    { color: "#32B968", name: "普通一档" },
                    { color: "#358AF3", name: "普通二档" },
                    { color: "#E8B72E", name: "普通三档" },
                    { color: "#E65757", name: "普通四档" },
                    { color: "#FFFFFF", name: "建图一档" },
                    { color: "#FFFFFF", name: "建图二档" }
                ]
                Column {
                    required property var modelData
                    required property int index
                    spacing: 6 * AppMetrics.scale
                    Rectangle {
                        anchors.horizontalCenter: parent.horizontalCenter
                        width: (index === root.gearIndex ? 54 : 36) * AppMetrics.scale
                        height: width
                        radius: width / 2
                        color: modelData.color
                        border.width: index === root.gearIndex ? 4 : 1
                        border.color: index === root.gearIndex
                            ? Theme.primary : Theme.border
                        opacity: index === 5 && root.gearIndex === 5
                            ? (root.phase % 2 === 0 ? 1 : 0.25) : 1
                        Behavior on width { NumberAnimation { duration: root.motionDuration } }
                    }
                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: I18n.t(modelData.name)
                        visible: index === root.gearIndex
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.body
                        font.bold: true
                    }
                }
            }
        }

        Row {
            visible: root.demoType === "normal"
            spacing: 12 * AppMetrics.scale
            Repeater {
                model: ["#32B968", "#358AF3", "#E8B72E", "#E65757"]
                Rectangle {
                    required property string modelData
                    required property int index
                    width: 38 * AppMetrics.scale
                    height: width
                    radius: width / 2
                    color: modelData
                    border.width: index === Math.floor(root.phase / 3) % 4 ? 4 : 1
                    border.color: index === Math.floor(root.phase / 3) % 4
                        ? Theme.primary : Theme.border
                    scale: index === Math.floor(root.phase / 3) % 4 ? 1.28 : 1
                    Behavior on scale { NumberAnimation { duration: root.motionDuration } }
                }
            }
        }

        Row {
            visible: root.demoType === "mapping"
            spacing: 18 * AppMetrics.scale
            Repeater {
                model: [
                    { title: "建图一档", detail: "白色常亮" },
                    { title: "建图二档", detail: "白色闪烁 · 速度更快" }
                ]
                Rectangle {
                    required property var modelData
                    required property int index
                    width: 176 * AppMetrics.scale
                    height: 82 * AppMetrics.scale
                    radius: Theme.radiusSmall
                    color: index === root.mappingIndex
                        ? Theme.primarySoft : Theme.surface
                    border.width: index === root.mappingIndex ? 3 : 1
                    border.color: index === root.mappingIndex
                        ? Theme.primary : Theme.border
                    opacity: index === 1 && root.mappingIndex === 1
                        ? (root.phase % 2 === 0 ? 1 : 0.45) : 1
                    Column {
                        anchors.centerIn: parent
                        spacing: 4 * AppMetrics.scale
                        Text {
                            anchors.horizontalCenter: parent.horizontalCenter
                            text: I18n.t(modelData.title)
                            color: Theme.textPrimary
                            font.pixelSize: AppMetrics.body
                            font.bold: true
                        }
                        Text {
                            anchors.horizontalCenter: parent.horizontalCenter
                            text: I18n.t(modelData.detail)
                            color: Theme.textSecondary
                            font.pixelSize: AppMetrics.small
                        }
                    }
                }
            }
        }

        Text {
            width: parent.width
            text: root.demoType === "normal"
                ? I18n.t(root.phase < 8 ? "十字键控制" : "右侧摇杆控制")
                : root.demoType === "mapping"
                    ? I18n.t(root.mappingIndex === 0
                        ? "一档使用左侧摇杆" : "二档使用左侧摇杆，速度更快")
                : I18n.t("教程仅演示，不会控制真实车辆")
            color: Theme.textSecondary
            font.pixelSize: AppMetrics.body
            font.bold: root.demoType === "normal" || root.demoType === "mapping"
            wrapMode: Text.WordWrap
        }
    }
}

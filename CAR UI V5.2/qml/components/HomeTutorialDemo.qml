import QtQuick
import QtQuick.Layouts
import ".."

Item {
    id: root
    property string demoType: "map"
    property bool active: true
    property bool loop: true
    property real progress: Performance.lowPower ? 0.62 : 0
    signal cycleFinished()

    function restart() {
        demoAnimation.stop()
        progress = Performance.lowPower ? 0.62 : 0
        if (active && !Performance.lowPower)
            demoAnimation.start()
    }

    onDemoTypeChanged: Qt.callLater(restart)
    onActiveChanged: Qt.callLater(restart)
    onLoopChanged: Qt.callLater(restart)
    Component.onCompleted: restart()

    SequentialAnimation {
        id: demoAnimation
        loops: root.loop ? Animation.Infinite : 1
        NumberAnimation {
            target: root
            property: "progress"
            from: 0
            to: 1
            duration: Performance.smooth ? 3300 : 2600
            easing.type: Easing.InOutCubic
        }
        PauseAnimation { duration: Performance.smooth ? 420 : 260 }
        ScriptAction { script: root.cycleFinished() }
        PropertyAction { target: root; property: "progress"; value: 0 }
    }

    Rectangle {
        anchors.fill: parent
        radius: Theme.radius
        color: Theme.surfaceMuted
        border.color: Theme.border
        border.width: Theme.borderWidth
        clip: true

        Rectangle {
            visible: Performance.smooth
            width: parent.width * 0.7
            height: width
            radius: width / 2
            x: -width * 0.25 + root.progress * parent.width * 0.7
            y: -height * 0.45
            color: Theme.primarySoft
            opacity: 0.34
        }
    }

    // Map goal and route demonstration.
    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding
        visible: root.demoType === "map"

        Repeater {
            model: 5
            Rectangle {
                required property int index
                x: index * parent.width / 5
                width: 1
                height: parent.height
                color: Theme.divider
            }
        }
        Repeater {
            model: 4
            Rectangle {
                required property int index
                y: index * parent.height / 4
                width: parent.width
                height: 1
                color: Theme.divider
            }
        }
        Rectangle {
            x: parent.width * 0.18
            y: parent.height * 0.63
            width: parent.width * 0.55 * Math.min(1, root.progress * 1.7)
            height: 5 * AppMetrics.scale
            radius: height / 2
            color: Theme.primary
            rotation: -18
            transformOrigin: Item.Left
        }
        Rectangle {
            width: 44 * AppMetrics.scale
            height: 28 * AppMetrics.scale
            radius: 8 * AppMetrics.scale
            color: Theme.primary
            x: parent.width * (0.16 + 0.52 * Math.min(1, root.progress * 1.15))
            y: parent.height * (0.63 - 0.17 * Math.min(1, root.progress * 1.15))
            Text { anchors.centerIn: parent; text: "AMR"; color: "white"; font.pixelSize: AppMetrics.caption; font.bold: true }
        }
        Item {
            width: 34 * AppMetrics.scale
            height: width
            x: parent.width * 0.73
            y: parent.height * 0.31
            scale: 0.82 + 0.18 * Math.sin(root.progress * Math.PI * 6)
            rotation: -Math.min(1, Math.max(0, (root.progress - 0.55) * 2.5)) * 110
            Rectangle {
                anchors.centerIn: parent
                width: 24 * AppMetrics.scale
                height: width
                radius: width / 2
                color: Theme.danger
                border.color: "white"
                border.width: 3 * AppMetrics.scale
            }
            Rectangle {
                anchors.left: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                width: parent.width * 0.75
                height: 4 * AppMetrics.scale
                radius: height / 2
                color: Theme.danger
            }
            Text {
                anchors.left: parent.right
                anchors.verticalCenter: parent.verticalCenter
                text: "▶"
                color: Theme.danger
                font.pixelSize: AppMetrics.sectionTitle
            }
        }
    }

    // Zoom, reset and map-management demonstration.
    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding
        visible: root.demoType === "map_tools"
        Rectangle {
            id: demoMap
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            width: parent.width * 0.78
            height: parent.height * 0.62
            radius: Theme.radiusSmall
            color: Theme.mapCanvas
            border.color: Theme.border
            border.width: Theme.borderWidth
            scale: 0.82 + 0.2 * Math.sin(root.progress * Math.PI)
            Text { anchors.centerIn: parent; text: I18n.t("完整地图视野"); color: Theme.textSecondary; font.pixelSize: AppMetrics.body }
        }
        RowLayout {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            spacing: AppMetrics.unit
            Repeater {
                model: ["＋", "－", "⌂", I18n.t("地图管理"), "⛶"]
                Rectangle {
                    required property var modelData
                    required property int index
                    Layout.fillWidth: true
                    height: 48 * AppMetrics.scale
                    radius: Theme.radiusSmall
                    color: index === Math.min(4, Math.floor(root.progress * 5))
                        ? Theme.primary : Theme.surface
                    border.color: Theme.primary
                    border.width: Theme.borderWidth
                    Text { anchors.centerIn: parent; text: modelData; color: parent.color === Theme.primary ? "white" : Theme.primary; font.pixelSize: AppMetrics.body; font.bold: true }
                }
            }
        }
    }

    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding * 1.4
        visible: root.demoType === "travel_status"
        ColumnLayout {
            anchors.centerIn: parent
            width: parent.width
            spacing: AppMetrics.sectionGap
            RowLayout {
                Layout.fillWidth: true
                Text {
                    Layout.fillWidth: true
                    text: root.progress < 0.33 ? I18n.t("正在行驶")
                        : root.progress < 0.58 ? I18n.t("已暂停") : I18n.t("继续行驶")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.sectionTitle
                    font.bold: true
                }
                StatusBadge { status: root.progress < 0.58 && root.progress >= 0.33 ? "WARNING" : "NORMAL" }
            }
            Rectangle {
                Layout.fillWidth: true
                height: 14 * AppMetrics.scale
                radius: height / 2
                color: Theme.disabledBackground
                Rectangle {
                    width: parent.width * Math.min(1, root.progress * 1.15)
                    height: parent.height
                    radius: height / 2
                    color: Theme.primary
                }
            }
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: AppMetrics.gap
                AppButton { text: root.progress < 0.58 && root.progress >= 0.33 ? I18n.t("继续") : I18n.t("暂停"); compact: true }
                AppButton { text: I18n.t("结束导航"); accent: Theme.danger; compact: true }
            }
        }
    }

    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding
        visible: root.demoType === "vehicle"
        RowLayout {
            anchors.fill: parent
            spacing: AppMetrics.gap
            Repeater {
                model: [I18n.t("上位机状态"), I18n.t("下位机状态"), I18n.t("运行与连接")]
                AppCard {
                    required property var modelData
                    required property int index
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: Theme.surface
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: AppMetrics.gap
                        Text { text: modelData; color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.bold: true }
                        Repeater {
                            model: 3
                            RowLayout {
                                required property int index
                                Layout.fillWidth: true
                                StatusDot { status: root.progress > (index + 1) * 0.18 ? "NORMAL" : "UNKNOWN" }
                                Rectangle { Layout.fillWidth: true; height: 8 * AppMetrics.scale; radius: height / 2; color: root.progress > (index + 1) * 0.18 ? Theme.successSoft : Theme.disabledBackground }
                            }
                        }
                        Item { Layout.fillHeight: true }
                    }
                }
            }
        }
    }

    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding
        visible: root.demoType === "navigation"
        ColumnLayout {
            anchors.fill: parent
            spacing: AppMetrics.gap
            RowLayout {
                Layout.fillWidth: true
                spacing: AppMetrics.unit
                Repeater {
                    model: [I18n.t("选择目的地"), I18n.t("开始导航"), I18n.t("到达目的地")]
                    Rectangle {
                        required property var modelData
                        required property int index
                        Layout.fillWidth: true
                        height: 48 * AppMetrics.scale
                        radius: Theme.radiusSmall
                        color: root.progress >= index * 0.34 ? Theme.primarySoft : Theme.surface
                        border.color: root.progress >= index * 0.34 ? Theme.primary : Theme.border
                        border.width: Theme.borderWidth
                        Text { anchors.centerIn: parent; text: modelData; color: Theme.textPrimary; font.pixelSize: AppMetrics.small; font.bold: true }
                    }
                }
            }
            Item {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; height: 5 * AppMetrics.scale; radius: height / 2; color: Theme.primarySoft }
                Rectangle {
                    width: 52 * AppMetrics.scale; height: 34 * AppMetrics.scale; radius: 9 * AppMetrics.scale
                    x: (parent.width - width) * root.progress; anchors.verticalCenter: parent.verticalCenter
                    color: Theme.primary
                    Text { anchors.centerIn: parent; text: "AMR"; color: "white"; font.pixelSize: AppMetrics.caption; font.bold: true }
                }
            }
        }
    }

    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding
        visible: root.demoType === "voice"
        RowLayout {
            anchors.centerIn: parent
            width: parent.width * 0.9
            spacing: AppMetrics.sectionGap
            Rectangle {
                width: 110 * AppMetrics.scale; height: width; radius: width / 2
                color: Theme.purpleSoft; border.color: Theme.purple; border.width: Theme.borderWidthStrong
                scale: 0.92 + (Performance.lowPower ? 0 : 0.08 * Math.sin(root.progress * Math.PI * 8))
                Image { anchors.centerIn: parent; width: 54 * AppMetrics.scale; height: width; source: "../../assets/icons/voice-listening.svg"; fillMode: Image.PreserveAspectFit }
            }
            ColumnLayout {
                Layout.fillWidth: true
                spacing: AppMetrics.gap
                Rectangle {
                    Layout.fillWidth: true; height: 62 * AppMetrics.scale; radius: Theme.radiusSmall
                    color: Theme.surface; border.color: Theme.purple; border.width: Theme.borderWidth
                    Text { anchors.centerIn: parent; text: I18n.t("前往前台"); color: Theme.textPrimary; font.pixelSize: AppMetrics.sectionTitle; font.bold: true }
                }
                Text {
                    Layout.fillWidth: true
                    text: root.progress < 0.52 ? I18n.t("正在听取指令") : I18n.t("指令已确认")
                    color: root.progress < 0.52 ? Theme.purple : Theme.success
                    font.pixelSize: AppMetrics.body; font.bold: true
                }
            }
        }
    }

    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding
        visible: root.demoType === "gamepad"
        RowLayout {
            anchors.centerIn: parent
            width: parent.width * 0.92
            spacing: AppMetrics.gap
            Rectangle {
                Layout.preferredWidth: 120 * AppMetrics.scale; Layout.preferredHeight: 64 * AppMetrics.scale
                radius: Theme.radiusSmall; color: root.progress < 0.5 ? Theme.primarySoft : Theme.surface
                border.color: Theme.primary; border.width: Theme.borderWidth
                Text { anchors.centerIn: parent; text: I18n.t("上位机"); color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.bold: true }
            }
            Text { text: "→"; color: Theme.success; font.pixelSize: AppMetrics.title * 1.6; opacity: 0.45 + root.progress * 0.55 }
            Image {
                Layout.fillWidth: true; Layout.preferredHeight: 150 * AppMetrics.scale
                source: Theme.darkMode ? "../../assets/decor/gamepad-lineart-dark.png" : "../../assets/decor/gamepad-lineart.png"
                fillMode: Image.PreserveAspectFit
                scale: 0.94 + (Performance.lowPower ? 0 : 0.06 * Math.sin(root.progress * Math.PI))
            }
            StatusBadge { status: root.progress > 0.5 ? "NORMAL" : "UNKNOWN"; label: I18n.t(root.progress > 0.5 ? "手柄接管" : "等待交接") }
        }
    }

    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding
        visible: root.demoType === "follow"
        Rectangle {
            anchors.fill: parent
            radius: Theme.radiusSmall
            color: Theme.mapCanvas
            border.color: Theme.border
            border.width: Theme.borderWidth
            Repeater {
                model: [0.18, 0.45, 0.72]
                Rectangle {
                    required property real modelData
                    required property int index
                    x: parent.width * modelData
                    y: parent.height * (0.24 + Math.abs(0.45 - modelData) * 0.25)
                    width: 55 * AppMetrics.scale; height: 110 * AppMetrics.scale
                    radius: Theme.radiusSmall
                    color: "transparent"
                    border.color: index === 1 && root.progress > 0.28 ? Theme.success : Theme.primary
                    border.width: index === 1 && root.progress > 0.28 ? Theme.borderWidthStrong * 2 : Theme.borderWidthStrong
                    Text { anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.top; text: I18n.t("人员") + " " + (index + 1); color: parent.border.color; font.pixelSize: AppMetrics.caption }
                }
            }
            Rectangle {
                width: 54 * AppMetrics.scale; height: 34 * AppMetrics.scale; radius: 9 * AppMetrics.scale
                x: parent.width * 0.42 + Math.sin(root.progress * Math.PI * 2) * 24 * AppMetrics.scale
                anchors.bottom: parent.bottom; anchors.bottomMargin: 18 * AppMetrics.scale
                color: Theme.primary
                Text { anchors.centerIn: parent; text: "AMR"; color: "white"; font.pixelSize: AppMetrics.caption; font.bold: true }
            }
        }
    }

    Item {
        anchors.fill: parent
        anchors.margins: AppMetrics.cardPadding * 1.5
        visible: root.demoType === "status_bar"
        Rectangle {
            anchors.left: parent.left; anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter
            height: 92 * AppMetrics.scale; radius: Theme.radiusSmall
            color: Theme.bottomBarBackground; border.color: Theme.border; border.width: Theme.borderWidth
            RowLayout {
                anchors.fill: parent; anchors.margins: AppMetrics.gap; spacing: AppMetrics.gap
                Text { text: "●  " + I18n.t("电量") + " 86%"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.bold: true }
                Item { Layout.fillWidth: true }
                Repeater {
                    model: ["‹  " + I18n.t("返回"), "⌂  " + I18n.t("主页"), "⛶"]
                    Rectangle {
                        required property var modelData
                        required property int index
                        width: 108 * AppMetrics.scale; height: 50 * AppMetrics.scale; radius: Theme.radiusSmall
                        color: index === Math.min(2, Math.floor(root.progress * 3)) ? Theme.primarySoft : Theme.surfaceMuted
                        Text { anchors.centerIn: parent; text: modelData; color: Theme.primary; font.pixelSize: AppMetrics.body; font.bold: true }
                    }
                }
                Item { Layout.fillWidth: true }
                Text { text: "09:28  ●"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.bold: true }
            }
        }
    }
}

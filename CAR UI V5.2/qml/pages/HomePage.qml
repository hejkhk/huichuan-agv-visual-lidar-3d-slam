import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"
import "../dialogs"

Page {
    id: root
    objectName: "homeControlHub"
    background: Rectangle { color: Theme.pageBackground }

    readonly property real gap: AppMetrics.gap
    readonly property real followDistance: Number(
        backend.settings.parameters?.follow_distance ?? 1.0
    )
    readonly property string voiceState: backend.snapshot.voice_state ?? "LISTENING"
    property bool gamepadControlActive: false
    property bool followControlExpanded: false
    property bool voiceControlExpanded: false
    property string followSelectedActor: backend.snapshot.follow_target || "Actor1"
    readonly property bool controlDetailExpanded:
        followControlExpanded || voiceControlExpanded

    function hostTakesControl() {
        gamepadControlActive = false
    }

    function selectedDestinationName() {
        if (backend.hasMapGoal)
            return I18n.t("地图上选择的位置")
        if (backend.selectedPointId !== "")
            return I18n.t(backend.selectedPoint.name ?? "已选择目的地")
        return I18n.t("还没有选择目的地")
    }

    function friendlyActorName(actorId) {
        if (!actorId)
            return I18n.t("尚未选择")
        var digits = String(actorId).match(/[0-9]+/)
        return digits ? I18n.t("人员") + " " + digits[0] : I18n.t("人员目标")
    }

    function currentVoiceLabel() {
        if (voiceState === "SPEAKING")
            return I18n.t("正在播报")
        if (voiceState === "READY")
            return I18n.t("可以说话")
        return I18n.t("正在听")
    }

    function currentVoiceIcon() {
        if (voiceState === "SPEAKING")
            return "../../assets/icons/voice-speaking.svg"
        if (voiceState === "READY")
            return "../../assets/icons/voice-ready.svg"
        return "../../assets/icons/voice-listening.svg"
    }

    function tutorialTarget(name) {
        if (name === "map") return homeMap
        if (name === "map_tools") return mapSection
        if (name === "travel_status") return navigationStrip
        if (name === "vehicle") return vehicleCard
        if (name === "navigation") return navigationCard
        if (name === "voice") return voiceCard
        if (name === "gamepad") return gamepadCard
        if (name === "follow") return followCard
        if (name === "status_bar") return window.tutorialStatusBar
        return dashboard
    }

    function closeTransient() {
        if (addPointDialog.visible) {
            addPointDialog.dismiss()
            return true
        }
        if (homeMappingConfirmDialog.visible) {
            homeMappingConfirmDialog.close()
            return true
        }
        if (followControlExpanded) {
            followControlExpanded = false
            return true
        }
        if (voiceControlExpanded) {
            voiceControlExpanded = false
            return true
        }
        return false
    }

    AddPointDialog { id: addPointDialog }
    MappingConfirmDialog {
        id: homeMappingConfirmDialog
        onAcceptedAction: {
            backend.startMapping()
            window.pushPage("MappingFullscreenPage.qml")
        }
        onTutorialRequested: window.pushPageWithProperties(
            "GamepadTutorialPage.qml",
            { "reopenMappingOnReturn": true }
        )
    }
    Connections {
        target: backend
        function onCurrentPoseReady(pose) {
            addPointDialog.pose = pose
            addPointDialog.open()
        }
    }

    Item {
        id: dashboard
        anchors.fill: parent
        anchors.margins: AppMetrics.margin

        AppCard {
            id: systemBrandFrame
            objectName: "systemBrandFrame"
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            height: 56 * AppMetrics.scale

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: AppMetrics.cardPadding
                anchors.rightMargin: AppMetrics.cardPadding
                spacing: AppMetrics.gap

                Text {
                    Layout.fillWidth: true
                    text: I18n.t("深圳文思汇通有限公司AMR操作系统")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.title
                    font.weight: Font.Bold
                    elide: Text.ElideRight
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }

        Item {
            id: leftPanel
            objectName: "homeInformationPanel"
            anchors.left: parent.left
            anchors.top: systemBrandFrame.bottom
            anchors.topMargin: root.gap
            anchors.bottom: parent.bottom
            width: (parent.width - root.gap) * (AppMetrics.compact ? 0.50 : 0.48)

            Item {
                id: mapSection
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                height: (parent.height - root.gap) * 0.63

                RvizPlaceholder {
                    id: homeMap
                    objectName: "homeMap"
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: navigationStrip.top
                    anchors.bottomMargin: root.gap
                    interactiveGoalSelection: true
                    mapManagementVisible: true
                    createMapVisible: true
                    onMapGoalSelected: function(worldX, worldY) {
                        backend.selectMapGoal(worldX, worldY)
                    }
                    onResetRequested: backend.clearNavigationSelection()
                    onFullscreenRequested: {
                        backend.rvizAction("fullscreen")
                        window.pushPage("RvizFullscreenPage.qml")
                    }
                    onCreateMapRequested: homeMappingConfirmDialog.open()
                    onMapManagementRequested:
                        window.pushPage("MapSelectorPage.qml")
                }

                AppCard {
                    id: navigationStrip
                    objectName: "homeTravelStatusStrip"
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom
                    height: 54 * AppMetrics.scale
                    radius: 8 * AppMetrics.scale

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 6 * AppMetrics.scale
                        spacing: 6 * AppMetrics.scale

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 2 * AppMetrics.scale
                            RowLayout {
                                Layout.fillWidth: true
                                Text {
                                    Layout.fillWidth: true
                                    text: I18n.t("行驶状态") + "："
                                        + I18n.t(backend.snapshot.navigation_message
                                            ?? "请先选择要去的地方")
                                    color: Theme.textPrimary
                                    font.pixelSize: AppMetrics.body
                                    font.weight: Font.DemiBold
                                    elide: Text.ElideRight
                                }
                                Text {
                                    text: (backend.snapshot.navigation_progress ?? 0) + "%"
                                    color: Theme.primary
                                    font.pixelSize: AppMetrics.body
                                    font.weight: Font.DemiBold
                                }
                            }
                            ProgressBar {
                                id: navigationProgress
                                Layout.fillWidth: true
                                from: 0
                                to: 100
                                value: backend.snapshot.navigation_progress ?? 0
                                background: Rectangle {
                                    implicitHeight: 7 * AppMetrics.scale
                                    radius: height / 2
                                    color: Theme.surfaceMuted
                                    border.color: Theme.border
                                    border.width: Theme.borderWidth
                                }
                                contentItem: Item {
                                    implicitHeight: 7 * AppMetrics.scale
                                    Rectangle {
                                        width: navigationProgress.visualPosition * parent.width
                                        height: parent.height
                                        radius: height / 2
                                        color: Theme.primary
                                    }
                                }
                            }
                        }

                        AppButton {
                            compact: true
                            text: I18n.t(backend.navigationControls.pauseText ?? "暂停")
                            enabled: backend.navigationControls.pauseEnabled ?? false
                            accent: Theme.primary
                            onClicked: {
                                root.hostTakesControl()
                                backend.togglePauseNavigation()
                            }
                        }
                        AppButton {
                            compact: true
                            text: I18n.t("结束导航")
                            enabled: backend.navigationControls.cancelEnabled ?? false
                            accent: Theme.danger
                            onClicked: {
                                root.hostTakesControl()
                                backend.cancelNavigation()
                            }
                        }
                    }
                }
            }

            AppCard {
                id: vehicleCard
                objectName: "vehicleOverviewCard"
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: mapSection.bottom
                anchors.topMargin: root.gap
                anchors.bottom: parent.bottom
                clip: true

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: AppMetrics.cardPadding
                    spacing: AppMetrics.unit

                    SectionHeader {
                        Layout.fillWidth: true
                        title: I18n.t("车辆状态")
                        subtitle: AppMetrics.compact ? "" : I18n.t("只显示驾驶时需要关注的信息")
                        AppButton {
                            compact: true
                            text: I18n.t("详细状态")
                            outlined: true
                            onClicked: window.pushPage("RobotStatusPage.qml")
                        }
                        AppButton {
                            compact: true
                            text: I18n.t("设置")
                            onClicked: window.pushPage("SettingsPage.qml")
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: AppMetrics.gap

                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            spacing: 0
                            Text {
                                Layout.fillWidth: true
                                Layout.bottomMargin: AppMetrics.unit * 0.35
                                text: I18n.t("上位机状态")
                                color: Theme.textPrimary
                                font.pixelSize: AppMetrics.body
                                font.weight: Font.DemiBold
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("主机负载")
                                value: Number(backend.snapshot.cpu_percent ?? 0).toFixed(0) + "%"
                                status: (backend.snapshot.cpu_percent ?? 0) < 85 ? "NORMAL" : "WARNING"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("内存使用")
                                value: Number(backend.snapshot.memory_percent ?? 0).toFixed(0) + "%"
                                status: (backend.snapshot.memory_percent ?? 0) < 85 ? "NORMAL" : "WARNING"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("设备温度")
                                value: Number(backend.snapshot.cpu_temperature ?? 0).toFixed(0) + "°C"
                                status: (backend.snapshot.cpu_temperature ?? 0) < 80 ? "NORMAL" : "WARNING"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("语音模块")
                                value: I18n.t((backend.snapshot.voice_module_status ?? "UNKNOWN") === "NORMAL"
                                    ? "工作正常" : "等待连接")
                                status: backend.snapshot.voice_module_status ?? "UNKNOWN"
                                showDivider: false
                            }
                        }

                        Rectangle { Layout.fillHeight: true; width: Theme.borderWidth; color: Theme.divider }

                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            spacing: 0
                            Text {
                                Layout.fillWidth: true
                                Layout.bottomMargin: AppMetrics.unit * 0.35
                                text: I18n.t("下位机状态")
                                color: Theme.textPrimary
                                font.pixelSize: AppMetrics.body
                                font.weight: Font.DemiBold
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("剩余电量")
                                value: Number(backend.snapshot.battery_percent ?? 0).toFixed(0) + "%"
                                status: (backend.snapshot.battery_percent ?? 0) >= 20 ? "NORMAL" : "WARNING"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("电池电压")
                                value: Number(backend.snapshot.battery_voltage ?? 0).toFixed(1) + " V"
                                status: (backend.snapshot.battery_voltage ?? 0) > 0 ? "NORMAL" : "WARNING"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("充电")
                                value: I18n.t(backend.snapshot.charging_status ?? "未在充电")
                                status: backend.snapshot.charging ? "NORMAL" : "UNKNOWN"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("编码器")
                                value: I18n.t((backend.snapshot.encoder_status ?? "UNKNOWN") === "NORMAL"
                                    ? "工作正常" : "等待连接")
                                status: backend.snapshot.encoder_status ?? "UNKNOWN"
                                showDivider: false
                            }
                        }

                        Rectangle { Layout.fillHeight: true; width: Theme.borderWidth; color: Theme.divider }

                        ColumnLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            spacing: 0
                            Text {
                                Layout.fillWidth: true
                                Layout.bottomMargin: AppMetrics.unit * 0.35
                                text: I18n.t("运行与连接")
                                color: Theme.textPrimary
                                font.pixelSize: AppMetrics.body
                                font.weight: Font.DemiBold
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("移动速度")
                                value: Number(backend.snapshot.vx ?? 0).toFixed(2) + " m/s"
                                status: "NORMAL"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("雷达")
                                value: I18n.t((backend.snapshot.lidar_status ?? "UNKNOWN") === "NORMAL"
                                    ? "工作正常" : "等待连接")
                                status: backend.snapshot.lidar_status ?? "UNKNOWN"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("控制系统")
                                value: I18n.t(backend.snapshot.ros_connected ? "已连接" : "等待连接")
                                status: backend.snapshot.ros_connected ? "NORMAL" : "WARNING"
                            }
                            DataRow {
                                Layout.fillWidth: true
                                label: I18n.t("网络连接")
                                value: I18n.t(backend.snapshot.network_connected ? "已连接" : "等待连接")
                                status: backend.snapshot.network_connected ? "NORMAL" : "WARNING"
                                showDivider: false
                            }
                        }
                    }
                }
            }
        }

        Item {
            id: rightPanel
            objectName: "homeControlPanel"
            anchors.left: leftPanel.right
            anchors.leftMargin: root.gap
            anchors.right: parent.right
            anchors.top: leftPanel.top
            anchors.bottom: parent.bottom

            AppCard {
                id: controlModeFrame
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.bottom: parent.bottom

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: AppMetrics.cardPadding
                    spacing: AppMetrics.gap

                    RowLayout {
                        visible: !root.controlDetailExpanded
                        Layout.fillWidth: true
                        Layout.preferredHeight: 30 * AppMetrics.scale
                        spacing: 6 * AppMetrics.scale
                        Text {
                            text: I18n.t("控制模式")
                            color: Theme.textPrimary
                            font.pixelSize: AppMetrics.sectionTitle
                            font.weight: Font.DemiBold
                            verticalAlignment: Text.AlignVCenter
                        }
                        Item { Layout.fillWidth: true }
                        Image {
                            Layout.preferredWidth: 160
                            Layout.preferredHeight: 60
                            z: 2
                            source: Theme.darkMode
                                ? "../../assets/branding/wensihuitong-dark.png"
                                : "../../assets/branding/wensihuitong-light.png"
                            fillMode: Image.PreserveAspectFit
                            cache: true
                        }
                        Text {
                            text: "&"
                            color: Theme.textPrimary
                            font.pixelSize: AppMetrics.body
                            font.weight: Font.DemiBold
                            verticalAlignment: Text.AlignVCenter
                        }
                        Image {
                            Layout.preferredWidth: 160
                            Layout.preferredHeight: 60
                            z: 2
                            source: Theme.darkMode
                                ? "../../assets/branding/hongxindeli-dark.png"
                                : "../../assets/branding/hongxindeli-light.png"
                            fillMode: Image.PreserveAspectFit
                            cache: true
                        }
                    }

                    Item {
                        id: modeGrid
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }
                }
            }

            Item {
                id: primaryControls
                parent: modeGrid
                objectName: "primaryControlRow"
                visible: !root.controlDetailExpanded
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                height: (parent.height - root.gap) / 2

                AppCard {
                    id: navigationCard
                    objectName: "navigationControlCard"
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    width: (parent.width - root.gap) / 2

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: AppMetrics.cardPadding
                        spacing: AppMetrics.unit

                        SectionHeader {
                            Layout.fillWidth: true
                            title: I18n.t("导航控制")
                            subtitle: AppMetrics.compact ? "" : I18n.t("选择目的地，然后开始行驶")
                            StatusBadge {
                                status: backend.snapshot.ros_connected ? "NORMAL" : "DISCONNECTED"
                                label: I18n.t(backend.snapshot.ros_connected
                                    ? "控制系统正常" : "控制系统未连接")
                            }
                        }

                        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.divider }

                        DataRow {
                            Layout.fillWidth: true
                            label: I18n.t("要去的地方")
                            value: root.selectedDestinationName()
                            status: (backend.hasMapGoal || backend.selectedPointId !== "")
                                ? "NORMAL" : "UNKNOWN"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: I18n.t("最近目的地")
                            color: Theme.textSecondary
                            font.pixelSize: AppMetrics.small
                            font.weight: Font.DemiBold
                        }

                        Repeater {
                            // 首页只保留最近使用的一个目的地，完整列表在
                            // “管理目的地”中查看，避免等高卡片内信息拥挤。
                            model: backend.recentPoints.slice(0, 1)
                            Button {
                                required property var modelData
                                Layout.fillWidth: true
                                implicitHeight: 42 * AppMetrics.scale
                                hoverEnabled: true
                                background: Rectangle {
                                    radius: Theme.radiusSmall * AppMetrics.scale
                                    color: backend.selectedPointId === modelData.id
                                        ? Theme.primarySoft : parent.down
                                            ? Theme.surfaceMuted : Theme.surface
                                    border.color: backend.selectedPointId === modelData.id
                                        ? Theme.primary : Theme.divider
                                    border.width: backend.selectedPointId === modelData.id
                                        ? Theme.borderWidthStrong : Theme.borderWidth
                                }
                                contentItem: RowLayout {
                                    spacing: AppMetrics.unit
                                    StatusDot { status: modelData.is_charging_point ? "NORMAL" : "UNKNOWN" }
                                    Text {
                                        Layout.fillWidth: true
                                        text: I18n.t(modelData.name)
                                        color: backend.selectedPointId === modelData.id
                                            ? Theme.primary : Theme.textPrimary
                                        font.pixelSize: AppMetrics.body
                                        font.weight: Font.DemiBold
                                        elide: Text.ElideRight
                                    }
                                }
                                onClicked: backend.selectPoint(modelData.id)
                            }
                        }

                        Text {
                            visible: backend.recentPoints.length === 0
                            Layout.fillWidth: true
                            text: I18n.t("还没有最近使用的目的地")
                            color: Theme.textMuted
                            font.pixelSize: AppMetrics.small
                            horizontalAlignment: Text.AlignHCenter
                        }

                        PrimaryButton {
                            objectName: "homeStartNavigationButton"
                            Layout.fillWidth: true
                            text: I18n.t("开始导航")
                            enabled: backend.navigationControls.startEnabled ?? false
                            onClicked: {
                                root.hostTakesControl()
                                backend.startSelectedNavigation()
                            }
                        }

                        GridLayout {
                            Layout.fillWidth: true
                            columns: 2
                            columnSpacing: AppMetrics.unit
                            rowSpacing: AppMetrics.unit

                            AppButton {
                                Layout.fillWidth: true
                                compact: true
                                text: I18n.t("返回充电座")
                                accent: Theme.success
                                onClicked: {
                                    root.hostTakesControl()
                                    backend.startCharging()
                                }
                            }
                            SecondaryButton {
                                Layout.fillWidth: true
                                compact: true
                                text: I18n.t("保存这里")
                                onClicked: {
                                    if (backend.hasMapGoal) {
                                        addPointDialog.pose = backend.mapGoal
                                        addPointDialog.open()
                                    } else {
                                        backend.requestCurrentPose()
                                    }
                                }
                            }
                            SecondaryButton {
                                Layout.fillWidth: true
                                Layout.columnSpan: 2
                                compact: true
                                text: I18n.t("管理目的地")
                                onClicked:
                                    window.pushPage("PointManagerPage.qml")
                            }
                        }
                        Item { Layout.fillHeight: true }
                    }
                }

                AppCard {
                    id: voiceCard
                    objectName: "voiceControlCard"
                    anchors.left: navigationCard.right
                    anchors.leftMargin: root.gap
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: AppMetrics.cardPadding
                        spacing: AppMetrics.unit

                        SectionHeader {
                            Layout.fillWidth: true
                            title: I18n.t("语音控制")
                            subtitle: AppMetrics.compact ? "" : I18n.t("对机器人说出要执行的操作")
                            AppSwitch {
                                checked: backend.snapshot.voice_control_enabled ?? false
                                onToggled: {
                                    root.hostTakesControl()
                                    backend.setVoiceEnabled(checked)
                                }
                            }
                        }
                        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.divider }

                        DataRow {
                            Layout.fillWidth: true
                            label: I18n.t("当前使用语音的人")
                            value: I18n.t(backend.snapshot.speaker_name ?? "暂未识别")
                            status: (backend.snapshot.voice_control_enabled ?? false)
                                ? "NORMAL" : "UNKNOWN"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: I18n.t("语音状态")
                            color: Theme.textSecondary
                            font.pixelSize: AppMetrics.small
                            font.weight: Font.DemiBold
                        }
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 48
                            radius: Theme.radiusSmall
                            color: Theme.purpleSoft
                            border.color: Theme.purple
                            border.width: Theme.borderWidth
                            RowLayout {
                                anchors.fill: parent
                                anchors.leftMargin: AppMetrics.gap
                                anchors.rightMargin: AppMetrics.gap
                                spacing: AppMetrics.unit
                                Image {
                                    Layout.preferredWidth: 25
                                    Layout.preferredHeight: 25
                                    source: root.currentVoiceIcon()
                                    fillMode: Image.PreserveAspectFit
                                }
                                Text {
                                    Layout.fillWidth: true
                                    text: root.currentVoiceLabel()
                                    color: Theme.purple
                                    font.pixelSize: AppMetrics.body
                                    font.weight: Font.DemiBold
                                }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Text {
                                Layout.fillWidth: true
                                text: I18n.t("允许陌生人使用语音控制")
                                color: Theme.textSecondary
                                font.pixelSize: AppMetrics.small
                                elide: Text.ElideRight
                            }
                            AppSwitch {
                                checked: backend.settings.unknown_voice_allowed ?? true
                                onToggled: {
                                    root.hostTakesControl()
                                    backend.setUnknownVoiceAllowed(checked)
                                }
                            }
                        }

                        Item { Layout.fillHeight: true }

                        PrimaryButton {
                            Layout.fillWidth: true
                            text: I18n.t("进入语音控制")
                            accent: Theme.purple
                            onClicked: {
                                root.hostTakesControl()
                                root.voiceControlExpanded = true
                            }
                        }
                    }
                }
            }

            Item {
                id: secondaryControls
                parent: modeGrid
                objectName: "secondaryControlRow"
                visible: !root.controlDetailExpanded
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.verticalCenter
                anchors.topMargin: root.gap / 2
                anchors.bottom: parent.bottom

                AppCard {
                    id: gamepadCard
                    objectName: "gamepadControlCard"
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    width: (parent.width - root.gap) / 2

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: AppMetrics.cardPadding
                        spacing: AppMetrics.unit
                        SectionHeader {
                            Layout.fillWidth: true
                            title: I18n.t("手柄控制")
                            subtitle: AppMetrics.compact ? "" : I18n.t("把车辆控制权交给手柄")
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            StatusDot { status: root.gamepadControlActive ? "NORMAL" : "UNKNOWN" }
                            Text {
                                Layout.fillWidth: true
                                text: I18n.t(root.gamepadControlActive
                                    ? "手柄接管" : "上位机接管")
                                color: root.gamepadControlActive
                                    ? Theme.success : Theme.textPrimary
                                font.pixelSize: AppMetrics.sectionTitle
                                font.weight: Font.DemiBold
                            }
                        }
                        AppButton {
                            objectName: "releaseControlToGamepadButton"
                            Layout.fillWidth: true
                            text: I18n.t(root.gamepadControlActive
                                ? "手柄已接管" : "交还控制权由手柄控制")
                            enabled: !root.gamepadControlActive
                            accent: Theme.success
                            onClicked: {
                                backend.releaseControlToGamepad()
                                root.gamepadControlActive = true
                            }
                        }
                        SecondaryButton {
                            Layout.fillWidth: true
                            text: I18n.t("手柄教程")
                            onClicked:
                                window.pushPage("GamepadTutorialPage.qml")
                        }
                    }

                    Image {
                        objectName: "homeGamepadLineArt"
                        anchors.right: parent.right
                        anchors.rightMargin: 8 * AppMetrics.scale
                        anchors.top: parent.top
                        anchors.topMargin: 8 * AppMetrics.scale
                        width: 285 * AppMetrics.scale
                        height: 138 * AppMetrics.scale
                        source: Theme.darkMode
                            ? "../../assets/decor/gamepad-lineart-dark.png"
                            : "../../assets/decor/gamepad-lineart.png"
                        sourceSize.width: 380
                        sourceSize.height: 234
                        fillMode: Image.PreserveAspectFit
                        smooth: true
                        mipmap: false
                    }
                }

                AppCard {
                    id: followCard
                    objectName: "followControlCard"
                    anchors.left: gamepadCard.right
                    anchors.leftMargin: root.gap
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: AppMetrics.cardPadding
                        spacing: AppMetrics.unit
                        SectionHeader {
                            Layout.fillWidth: true
                            title: I18n.t("视觉控制")
                            subtitle: AppMetrics.compact ? "" : I18n.t("识别人并自动跟随")
                            Image {
                                objectName: "homeCameraLineArt"
                                Layout.preferredWidth: 190 * AppMetrics.scale
                                Layout.preferredHeight: 58 * AppMetrics.scale
                                source: Theme.darkMode
                                    ? "../../assets/decor/camera-lineart-dark.png"
                                    : "../../assets/decor/camera-lineart.png"
                                sourceSize.width: 380
                                sourceSize.height: 116
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                                mipmap: false
                            }
                            AppSwitch {
                                checked: backend.snapshot.visual_follow_enabled ?? false
                                onToggled: {
                                    root.hostTakesControl()
                                    backend.setFollowEnabled(checked)
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: AppMetrics.unit
                            Text {
                                Layout.fillWidth: true
                                text: I18n.t("识别到") + " "
                                    + (backend.snapshot.detected_actors?.length ?? 0)
                                    + " " + I18n.t("人")
                                color: Theme.textSecondary
                                font.pixelSize: AppMetrics.small
                            }
                            Text {
                                text: I18n.t("正在跟随") + " "
                                    + root.friendlyActorName(backend.snapshot.follow_target)
                                color: Theme.textPrimary
                                font.pixelSize: AppMetrics.small
                                font.weight: Font.DemiBold
                                elide: Text.ElideRight
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            Text {
                                Layout.fillWidth: true
                                text: I18n.t("跟随距离")
                                color: Theme.textSecondary
                                font.pixelSize: AppMetrics.small
                            }
                            Text {
                                text: homeFollowDistanceSlider.value.toFixed(1) + " m"
                                color: Theme.textPrimary
                                font.pixelSize: AppMetrics.small
                                font.weight: Font.DemiBold
                            }
                        }
                        AppSlider {
                            id: homeFollowDistanceSlider
                            objectName: "homeFollowDistanceSlider"
                            Layout.fillWidth: true
                            from: 0.5
                            to: 10.0
                            stepSize: 0.1
                            value: root.followDistance
                            onPressedChanged: if (!pressed) {
                                root.hostTakesControl()
                                backend.setParameter(
                                    "follow_distance", Number(value.toFixed(1))
                                )
                            }
                        }
                        PrimaryButton {
                            objectName: "homeStartFollowingButton"
                            Layout.fillWidth: true
                            text: I18n.t("开始跟随")
                            enabled: backend.snapshot.visual_follow_enabled ?? false
                            onClicked: {
                                root.hostTakesControl()
                                root.followSelectedActor = backend.snapshot.follow_target
                                    || root.followSelectedActor || "Actor1"
                                backend.startFollowing(root.followSelectedActor)
                            }
                        }
                        SecondaryButton {
                            Layout.fillWidth: true
                            text: I18n.t("全屏控制")
                            onClicked: {
                                root.hostTakesControl()
                                root.followSelectedActor = backend.snapshot.follow_target || "Actor1"
                                root.followControlExpanded = true
                            }
                        }
                    }
                }
            }

            Loader {
                id: embeddedFollowLoader
                parent: modeGrid
                objectName: "embeddedFollowLoader"
                anchors.fill: parent
                active: root.followControlExpanded
                visible: active
                sourceComponent: Component {
                    FollowPage {
                        embedded: true
                        selectedActor: root.followSelectedActor
                        onSelectedActorChanged: root.followSelectedActor = selectedActor
                        onExitRequested: root.followControlExpanded = false
                    }
                }
            }

            Loader {
                id: embeddedVoiceLoader
                parent: modeGrid
                objectName: "embeddedVoiceLoader"
                anchors.fill: parent
                active: root.voiceControlExpanded
                visible: active
                sourceComponent: Component {
                    VoiceControlPage {
                        embedded: true
                        onExitRequested: root.voiceControlExpanded = false
                    }
                }
            }
        }
    }
}

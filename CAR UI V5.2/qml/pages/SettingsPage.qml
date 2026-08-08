import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"
import "../dialogs"

Page {
    id: root
    objectName: "settingsPage"
    background: Rectangle { color: Theme.pageBackground }
    property int activePanel: 0
    property bool wifiLoaded: false
    property int pendingVolume: -1
    property bool developerUnlocked: false

    Component.onCompleted: {
        if (!wifiLoaded) {
            backend.refreshWifi()
            wifiLoaded = true
        }
    }

    WifiPasswordDialog { id: wifiPasswordDialog }
    DeveloperPasswordDialog {
        id: developerPasswordDialog
        onUnlocked: {
            root.developerUnlocked = true
            root.activePanel = 8
            backend.refreshDeveloperData()
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: AppMetrics.margin
        spacing: AppMetrics.sectionGap

        PageHeader {
            Layout.fillWidth: true
            title: I18n.t("设置")
            subtitle: I18n.t("系统连接、运行参数与界面偏好")
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: AppMetrics.sectionGap

            AppCard {
                Layout.preferredWidth: 300 * AppMetrics.scale
                Layout.fillHeight: true

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10 * AppMetrics.scale
                    spacing: 6 * AppMetrics.scale

                    Repeater {
                        model: [
                            { title: I18n.t("Wi-Fi 设置"), subtitle: I18n.t("无线网络连接") },
                            { title: I18n.t("音量设置"), subtitle: I18n.t("扬声器输出") },
                            { title: I18n.t("参数设置"), subtitle: I18n.t("机器人运行参数") },
                            { title: I18n.t("OTA 和语言"), subtitle: I18n.t("版本与界面语言") },
                            { title: I18n.t("外观设置"), subtitle: I18n.t("主题、字体与边框") },
                            { title: I18n.t("我的小车"), subtitle: I18n.t("设备与制造信息") },
                            { title: I18n.t("历史数据"), subtitle: I18n.t("趋势图表与数据记录") },
                            { title: I18n.t("告警日志"), subtitle: I18n.t("系统事件与异常记录") },
                            { title: I18n.t("开发者模式"), subtitle: I18n.t("ROS 2 状态与通信设置") }
                        ]

                        Button {
                            required property var modelData
                            required property int index
                            Layout.fillWidth: true
                            Layout.preferredHeight: 62 * AppMetrics.scale
                            onClicked: {
                                if (index === 8 && !root.developerUnlocked) {
                                    developerPasswordDialog.begin()
                                    return
                                }
                                root.activePanel = index
                                if (index === 0) backend.refreshWifi()
                                else if (index === 1) backend.refreshSystemVolume()
                                else if (index === 5) backend.refreshSystemInfo()
                                else if (index === 8) backend.refreshDeveloperData()
                            }
                            background: Rectangle {
                                radius: Theme.radiusSmall * AppMetrics.scale
                                color: root.activePanel === index ? Theme.primarySoft : "transparent"
                                border.color: root.activePanel === index ? Theme.primary : "transparent"
                                border.width: root.activePanel === index
                                    ? Theme.borderWidth : 0
                            }
                            contentItem: ColumnLayout {
                                spacing: 2
                                Text { text: modelData.title; color: root.activePanel === index ? Theme.primary : Theme.textPrimary; font.pixelSize: AppMetrics.body; font.weight: Font.DemiBold }
                                Text { text: modelData.subtitle; color: Theme.textMuted; font.pixelSize: AppMetrics.caption }
                            }
                        }
                    }
                    Item { Layout.fillHeight: true }
                }
            }

            AppCard {
                Layout.fillWidth: true
                Layout.fillHeight: true

                StackLayout {
                    anchors.fill: parent
                    anchors.margins: AppMetrics.cardPadding
                    currentIndex: root.activePanel

                    ColumnLayout {
                        spacing: AppMetrics.gap
                        SectionHeader {
                            Layout.fillWidth: true
                            title: I18n.t("可用 Wi-Fi")
                            subtitle: I18n.t("选择网络并建立连接")
                            AppButton { text: I18n.t("刷新"); outlined: true; compact: true; onClicked: backend.refreshWifi() }
                        }
                        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.divider }
                        ListView {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            clip: true
                            model: backend.wifiNetworks
                            spacing: 4 * AppMetrics.scale
                            delegate: Item {
                                required property var modelData
                                width: ListView.view.width
                                height: 66 * AppMetrics.scale
                                RowLayout {
                                    anchors.fill: parent
                                    anchors.leftMargin: AppMetrics.unit
                                    anchors.rightMargin: AppMetrics.unit
                                    spacing: AppMetrics.gap
                                    StatusDot { status: modelData.connected ? "NORMAL" : "UNKNOWN" }
                                    ColumnLayout {
                                        Layout.fillWidth: true
                                        spacing: 2
                                        Text { text: modelData.ssid; color: Theme.textPrimary; font.pixelSize: AppMetrics.body; font.weight: modelData.connected ? Font.DemiBold : Font.Normal; elide: Text.ElideRight }
                                        Text { text: modelData.connected ? I18n.t("已连接") : modelData.secured ? I18n.t("需要密码") : I18n.t("开放"); color: modelData.connected ? Theme.success : Theme.textMuted; font.pixelSize: AppMetrics.caption }
                                    }
                                    Text { text: modelData.strength + "%"; color: Theme.textSecondary; font.pixelSize: AppMetrics.small }
                                    AppButton {
                                        text: modelData.connected ? I18n.t("断开") : I18n.t("连接")
                                        compact: true
                                        outlined: !modelData.connected
                                        accent: modelData.connected ? Theme.danger : Theme.primary
                                        onClicked: {
                                            if (modelData.connected) backend.connectWifi(modelData.ssid, "")
                                            else if (modelData.secured) {
                                                wifiPasswordDialog.targetSsid = modelData.ssid
                                                wifiPasswordDialog.open()
                                            } else backend.connectWifi(modelData.ssid, "")
                                        }
                                    }
                                }
                                Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: Theme.divider }
                            }
                        }
                    }

                    ColumnLayout {
                        spacing: AppMetrics.sectionGap
                        SectionHeader { Layout.fillWidth: true; title: I18n.t("扬声器音量"); subtitle: I18n.t("调整语音播报和系统提示音") }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("当前音量")
                            description: I18n.t("松开滑块后立即生效")
                            value: Math.round(volume.value) + "%"
                        }
                        AppSlider {
                            id: volume
                            objectName: "systemVolumeSlider"
                            Layout.fillWidth: true
                            from: 0; to: 100; stepSize: 1
                            Component.onCompleted: value = Number(backend.settings.volume ?? 68)
                            onPressedChanged: {
                                if (!pressed) {
                                    root.pendingVolume = Math.round(value)
                                    backend.setVolume(root.pendingVolume)
                                }
                            }
                        }
                        Connections {
                            target: backend
                            function onDataChanged() {
                                if (volume.pressed)
                                    return
                                var actual = Math.round(Number(backend.settings.volume ?? 68))
                                if (root.pendingVolume < 0 || actual === root.pendingVolume) {
                                    volume.value = actual
                                    root.pendingVolume = -1
                                }
                            }
                        }
                        Item { Layout.fillHeight: true }
                    }

                    ColumnLayout {
                        spacing: AppMetrics.sectionGap
                        SectionHeader { Layout.fillWidth: true; title: I18n.t("运行参数（Mock）"); subtitle: I18n.t("仅调整现有机器人参数") }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("最大转向速度")
                            description: I18n.t("限制机器人旋转响应")
                            SpinBox { id: speed; from: 1; to: 10; value: 5; onValueModified: backend.setParameter("max_speed", value / 10) }
                        }
                        Item { Layout.fillHeight: true }
                    }

                    ColumnLayout {
                        spacing: AppMetrics.sectionGap
                        SectionHeader { Layout.fillWidth: true; title: I18n.t("OTA 和语言"); subtitle: I18n.t("软件版本与显示语言") }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("当前版本")
                            description: I18n.t("已是最新版本")
                            value: "1.4.2"
                            StatusBadge { status: "NORMAL" }
                        }
                        AppButton { Layout.fillWidth: true; text: I18n.t("检查并模拟升级"); accent: Theme.primary; onClicked: backend.startOta() }
                        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.divider }
                        Text { text: I18n.t("界面语言"); color: Theme.textPrimary; font.pixelSize: AppMetrics.sectionTitle; font.weight: Font.DemiBold }
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: AppMetrics.gap
                            AppButton { Layout.fillWidth: true; text: "中文"; accent: Theme.primary; outlined: backend.language !== "zh"; onClicked: backend.setLanguage("zh") }
                            AppButton { Layout.fillWidth: true; text: "English"; accent: Theme.primary; outlined: backend.language !== "en"; onClicked: backend.setLanguage("en") }
                            AppButton { Layout.fillWidth: true; text: "Русский"; accent: Theme.primary; outlined: backend.language !== "ru"; onClicked: backend.setLanguage("ru") }
                        }
                        Text { text: I18n.t("切换后立即生效"); color: Theme.textMuted; font.pixelSize: AppMetrics.caption }
                        Item { Layout.fillHeight: true }
                    }

                    ColumnLayout {
                        spacing: AppMetrics.sectionGap
                        SectionHeader { Layout.fillWidth: true; title: I18n.t("外观设置"); subtitle: I18n.t("主题切换在当前运行周期内生效") }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("配色方案")
                            description: I18n.t("仅改变界面颜色，不改变功能")
                            SegmentedControl {
                                implicitWidth: 520 * AppMetrics.scale
                                options: [
                                    I18n.t("工业蓝"),
                                    I18n.t("石墨青"),
                                    I18n.t("深空紫"),
                                    I18n.t("钛金橙")
                                ]
                                currentIndex: Theme.colorScheme
                                onSelected: function(index) {
                                    window.setColorScheme(index)
                                }
                            }
                        }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("字体大小")
                            description: I18n.t("统一调整全界面文字")
                            SegmentedControl {
                                objectName: "fontSizeModeControl"
                                implicitWidth: 360
                                options: [I18n.t("小"), I18n.t("标准"), I18n.t("大")]
                                currentIndex: AppMetrics.fontSizeMode
                                onSelected: function(index) {
                                    window.setFontSizeMode(index)
                                }
                            }
                        }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("边框粗细")
                            description: I18n.t("统一调整卡片和控件描边")
                            SegmentedControl {
                                objectName: "borderModeControl"
                                implicitWidth: 360
                                options: [I18n.t("细"), I18n.t("中"), I18n.t("粗")]
                                currentIndex: Theme.borderMode
                                onSelected: function(index) {
                                    window.setBorderMode(index)
                                }
                            }
                        }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("界面主题")
                            description: I18n.t("默认使用亮色模式")
                            SegmentedControl {
                                options: [I18n.t("亮色"), I18n.t("暗色")]
                                currentIndex: Theme.darkMode ? 1 : 0
                                onSelected: function(index) {
                                    window.setDarkMode(index === 1)
                                }
                            }
                        }
                        SettingRow {
                            Layout.fillWidth: true
                            title: I18n.t("性能模式")
                            description: I18n.t("调整动画和界面刷新策略")
                            SegmentedControl {
                                objectName: "performanceModeControl"
                                implicitWidth: 420 * AppMetrics.scale
                                options: [
                                    I18n.t("低性能模式"),
                                    I18n.t("普通模式"),
                                    I18n.t("流畅模式")
                                ]
                                currentIndex: Performance.mode
                                onSelected: function(index) {
                                    window.setPerformanceMode(index)
                                }
                            }
                        }
                        Text {
                            Layout.fillWidth: true
                            text: I18n.t("流畅模式可能占用更多算力和内存")
                            color: Theme.textMuted
                            font.pixelSize: AppMetrics.caption
                            horizontalAlignment: Text.AlignRight
                        }
                        AppCard {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 150 * AppMetrics.scale
                            color: Theme.surfaceMuted
                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: AppMetrics.cardPadding
                                Text { text: I18n.t("主题预览"); color: Theme.textPrimary; font.pixelSize: AppMetrics.sectionTitle; font.weight: Font.DemiBold }
                                Text {
                                    text: [
                                        I18n.t("工业蓝控制台"),
                                        I18n.t("石墨青控制台"),
                                        I18n.t("深空紫控制台"),
                                        I18n.t("钛金橙控制台")
                                    ][Theme.colorScheme] + " · "
                                        + I18n.t(Theme.darkMode ? "暗色" : "亮色")
                                    color: Theme.textSecondary
                                    font.pixelSize: AppMetrics.body
                                }
                                RowLayout {
                                    spacing: AppMetrics.unit
                                    StatusBadge { status: "NORMAL" }
                                    StatusBadge { status: "WARNING" }
                                    StatusBadge { status: "ERROR" }
                                }
                            }
                        }
                        Item { Layout.fillHeight: true }
                    }

                    MyRobotPanel {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }

                    HistoryDataPanel {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }

                    AlertLogPanel {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }

                    DeveloperPanel {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }
                }
            }
        }
    }
}

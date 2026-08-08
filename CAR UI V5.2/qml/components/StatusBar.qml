import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."

Rectangle {
    id: root
    signal backRequested()
    signal homeRequested()
    signal fullscreenRequested()
    property bool fullScreen: false
    color: Theme.bottomBarBackground
    border.color: Theme.border
    border.width: Theme.borderWidth
    implicitHeight: AppMetrics.statusHeight

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: AppMetrics.margin
        anchors.rightMargin: AppMetrics.margin
        spacing: AppMetrics.gap

        RowLayout {
            spacing: AppMetrics.gap
            StatusDot { status: (backend.snapshot.battery_percent ?? 0) > 20 ? "NORMAL" : "WARNING" }
            Text { text: I18n.t("电量") + " " + (backend.snapshot.battery_percent ?? "--") + "%"; color: Theme.textPrimary; font.pixelSize: AppMetrics.small; font.weight: Font.DemiBold }
            Rectangle { width: 1; height: 24 * AppMetrics.scale; color: Theme.divider }
            Text { text: I18n.t("续航") + " " + (backend.snapshot.remaining_range_km ?? "--") + " km"; color: Theme.textSecondary; font.pixelSize: AppMetrics.small }
            Text { visible: root.width >= 1200; text: I18n.t(backend.snapshot.bluetooth_connected ? "蓝牙 已连接" : "蓝牙 断开"); color: backend.snapshot.bluetooth_connected ? Theme.info : Theme.textMuted; font.pixelSize: AppMetrics.small }
            Text { visible: root.width >= 1450; text: I18n.t("控制系统") + " " + I18n.t(backend.snapshot.system_status ?? "连接中"); color: Theme.success; font.pixelSize: AppMetrics.small }
        }

        Item { Layout.fillWidth: true }

        Rectangle {
            implicitWidth: 294 * AppMetrics.scale
            implicitHeight: 52 * AppMetrics.scale
            radius: 11 * AppMetrics.scale
            color: Theme.surfaceMuted
            border.color: Theme.border
            border.width: Theme.borderWidth
            RowLayout {
                anchors.fill: parent
                anchors.margins: 4 * AppMetrics.scale
                spacing: 4 * AppMetrics.scale
                Button {
                    id: backButton
                    Layout.fillWidth: true; Layout.fillHeight: true
                    background: Rectangle { color: backButton.down ? Theme.primarySoft : "transparent"; radius: 8 * AppMetrics.scale }
                    contentItem: RowLayout {
                        spacing: 6 * AppMetrics.scale
                        ThemedNavIcon {
                            Layout.preferredWidth: 22 * AppMetrics.scale
                            Layout.preferredHeight: 22 * AppMetrics.scale
                            iconName: "back"
                            iconColor: Theme.textPrimary
                        }
                        Text { text: I18n.t("返回"); color: Theme.textPrimary; font.pixelSize: AppMetrics.small; font.weight: Font.DemiBold }
                    }
                    onClicked: root.backRequested()
                }
                Button {
                    id: homeButton
                    Layout.fillWidth: true; Layout.fillHeight: true
                    background: Rectangle {
                        color: Theme.primarySoft
                        radius: 8 * AppMetrics.scale
                        border.color: Theme.primary
                        border.width: Theme.borderWidth
                    }
                    contentItem: RowLayout {
                        spacing: 6 * AppMetrics.scale
                        ThemedNavIcon {
                            Layout.preferredWidth: 22 * AppMetrics.scale
                            Layout.preferredHeight: 22 * AppMetrics.scale
                            iconName: "home"
                            iconColor: Theme.primary
                        }
                        Text { text: I18n.t("主页"); color: Theme.primary; font.pixelSize: AppMetrics.small; font.weight: Font.DemiBold }
                    }
                    onClicked: root.homeRequested()
                }
                Button {
                    id: fullscreenButton
                    Layout.fillWidth: true; Layout.fillHeight: true
                    background: Rectangle { color: fullscreenButton.down ? Theme.primarySoft : "transparent"; radius: 8 * AppMetrics.scale }
                    contentItem: ThemedNavIcon {
                        iconName: root.fullScreen ? "restore" : "fullscreen"
                        iconColor: Theme.textPrimary
                    }
                    onClicked: root.fullscreenRequested()
                }
            }
        }

        Item { Layout.fillWidth: true }

        RowLayout {
            spacing: AppMetrics.gap
            Text { text: Qt.formatTime(new Date(), "hh:mm"); color: Theme.textPrimary; font.pixelSize: AppMetrics.sectionTitle; font.weight: Font.DemiBold; Timer { interval: Performance.lowPower ? 30000 : 1000; running: true; repeat: true; onTriggered: parent.text = Qt.formatTime(new Date(), "hh:mm") } }
            StatusDot { status: backend.snapshot.network_connected ? "NORMAL" : "DISCONNECTED" }
            Text { visible: root.width >= 1180; text: I18n.t(backend.snapshot.network_connected ? "网络正常" : "网络断开"); color: backend.snapshot.network_connected ? Theme.success : Theme.textMuted; font.pixelSize: AppMetrics.small }
            Text { visible: root.width >= 1550; text: "↑ " + (backend.snapshot.upload_kbps ?? 0) + " kb/s  ↓ " + (backend.snapshot.download_kbps ?? 0) + " kb/s"; color: Theme.textMuted; font.pixelSize: AppMetrics.caption }
        }
    }
}

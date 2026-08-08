import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

Page {
    background: Rectangle { color: Theme.pageBackground }
    function fmt3(v) { var n = Number(v || 0); return n.toFixed(3) }
    ColumnLayout { anchors.fill: parent; anchors.margins: AppMetrics.cardPadding; spacing: AppMetrics.gap
        PageHeader { Layout.fillWidth: true; title: I18n.t("车辆详细状态"); subtitle: I18n.t("状态会自动更新"); StatusBadge { status: backend.snapshot.ros_connected ? "NORMAL" : "WARNING" } }
        AppCard {
            Layout.fillWidth: true
            Layout.preferredHeight: 112 * AppMetrics.scale
            RowLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                spacing: AppMetrics.gap
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 3 * AppMetrics.scale
                    Text {
                        text: "SLAM / Nav2：" + (backend.snapshot.slam_message ?? "未运行")
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.body
                        font.weight: Font.DemiBold
                    }
                    Text {
                        text: "重定位：" + (backend.snapshot.localization_ready ? "已完成" : (backend.snapshot.localization_state ?? "inactive"))
                            + ((backend.snapshot.localization_detail ?? "") ? " · " + backend.snapshot.localization_detail : "")
                        color: backend.snapshot.localization_ready ? Theme.success : Theme.textSecondary
                        font.pixelSize: AppMetrics.caption
                        elide: Text.ElideRight
                        Layout.fillWidth: true
                    }
                    Text {
                        text: "脱困：" + (backend.snapshot.recovery_stage ?? "tracking")
                            + ((backend.snapshot.recovery_reason ?? "") ? " · " + backend.snapshot.recovery_reason : "")
                            + " · 次数 " + (backend.snapshot.recovery_count ?? 0)
                        color: (backend.snapshot.recovery_stage ?? "tracking") === "tracking" ? Theme.textMuted : Theme.warning
                        font.pixelSize: AppMetrics.caption
                        elide: Text.ElideRight
                        Layout.fillWidth: true
                    }
                }
                PrimaryButton {
                    text: "启动 SLAM 导航"
                    enabled: !(backend.snapshot.slam_running ?? false) && !backend.busy
                    onClicked: backend.startSlamNavigation()
                }
                DangerButton {
                    text: "停止整个系统"
                    enabled: (backend.snapshot.slam_running ?? false) && !backend.busy
                    onClicked: backend.stopSlamSystem()
                }
            }
        }
        GridLayout {
        Layout.fillWidth: true
        Layout.fillHeight: true
        columns: width >= 1200 || AppMetrics.compact ? 2 : 1
        columnSpacing: AppMetrics.gap
        rowSpacing: AppMetrics.gap
        AppCard { Layout.fillWidth: true; Layout.fillHeight: true
            ColumnLayout { anchors.fill: parent; anchors.margins: AppMetrics.cardPadding; spacing: AppMetrics.gap
                SectionHeader { Layout.fillWidth: true; title: I18n.t("主机运行情况") }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("主机负载") + "  " + (backend.snapshot.cpu_percent ?? 0) + "%"; status: "NORMAL" }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("内存使用") + "  " + (backend.snapshot.memory_percent ?? 0) + "%"; status: backend.snapshot.memory_percent > 80 ? "WARNING" : "NORMAL" }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("设备温度") + "  " + (backend.snapshot.cpu_temperature ?? 0) + "°C"; status: backend.snapshot.cpu_temperature > 70 ? "WARNING" : "NORMAL" }
                Item { Layout.fillHeight: true }
                Text { text: I18n.t("状态会自动更新"); color: Theme.textMuted; font.pixelSize: AppMetrics.small }
            }
        }
        AppCard { Layout.fillWidth: true; Layout.fillHeight: true
            ColumnLayout { anchors.fill: parent; anchors.margins: AppMetrics.cardPadding; spacing: AppMetrics.gap
                SectionHeader { Layout.fillWidth: true; title: I18n.t("设备与电池") }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("编码器"); status: backend.snapshot.encoder_status ?? "UNKNOWN" }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("激光雷达"); status: backend.snapshot.lidar_status ?? "UNKNOWN" }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("语音模块"); status: backend.snapshot.voice_module_status ?? "UNKNOWN" }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("电池电压") + "  " + (backend.snapshot.battery_voltage ?? 0) + "V"; status: "NORMAL" }
                StatusIndicator { Layout.fillWidth: true; label: I18n.t("充电状态") + "  " + I18n.t(backend.snapshot.charging_status ?? "未在充电"); status: backend.snapshot.charging ? "NORMAL" : "WARNING" }
                Item { Layout.fillHeight: true }
            }
        }
        AppCard {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.columnSpan: AppMetrics.compact ? 2 : 1
            ColumnLayout { anchors.fill: parent; anchors.margins: AppMetrics.cardPadding; spacing: AppMetrics.gap
                SectionHeader { Layout.fillWidth: true; title: I18n.t("车辆运动情况") }
                GridLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    columns: AppMetrics.compact ? 3 : 1
                    columnSpacing: AppMetrics.sectionGap
                    rowSpacing: AppMetrics.unit
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: AppMetrics.unit
                        Text { text: I18n.t("前后速度") + "：" + fmt3(backend.snapshot.vx) + " m/s"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Text { text: I18n.t("横向速度") + "：" + fmt3(backend.snapshot.vy) + " m/s"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Text { text: I18n.t("转向速度") + "：" + fmt3(backend.snapshot.wz) + " rad/s"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Item { Layout.fillHeight: true }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: AppMetrics.unit
                        Text { text: I18n.t("加速度"); font.pixelSize: AppMetrics.sectionTitle; font.bold: true; color: Theme.textPrimary }
                        Text { text: I18n.t("前后") + "：" + fmt3(backend.snapshot.ax) + " g"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Text { text: I18n.t("左右") + "：" + fmt3(backend.snapshot.ay) + " g"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Text { text: I18n.t("上下") + "：" + fmt3(backend.snapshot.az) + " g"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Item { Layout.fillHeight: true }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: AppMetrics.unit
                        Text { text: I18n.t("转动情况"); font.pixelSize: AppMetrics.sectionTitle; font.bold: true; color: Theme.textPrimary }
                        Text { text: I18n.t("前后翻转") + "：" + fmt3(backend.snapshot.gx) + " °/s"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Text { text: I18n.t("左右倾斜") + "：" + fmt3(backend.snapshot.gy) + " °/s"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Text { text: I18n.t("转向") + "：" + fmt3(backend.snapshot.gz) + " °/s"; color: Theme.textPrimary; font.pixelSize: AppMetrics.body }
                        Item { Layout.fillHeight: true }
                    }
                }
                Item { Layout.fillHeight: true }
            }
        }
        }
    }
}

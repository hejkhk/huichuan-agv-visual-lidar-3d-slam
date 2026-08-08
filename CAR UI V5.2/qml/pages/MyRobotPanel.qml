import QtQuick
import QtQuick.Layouts
import ".."
import "../components"

ColumnLayout {
    id: root
    spacing: AppMetrics.gap

    SectionHeader {
        Layout.fillWidth: true
        title: I18n.t("我的小车")
        subtitle: I18n.t("设备、系统与制造信息")
        AppButton {
            compact: true
            outlined: true
            text: I18n.t("刷新")
            onClicked: backend.refreshSystemInfo()
        }
    }

    AppCard {
        Layout.fillWidth: true
        Layout.preferredHeight: 230
        color: Theme.surfaceMuted
        RowLayout {
            anchors.fill: parent
            anchors.margins: AppMetrics.cardPadding
            spacing: AppMetrics.sectionGap
            Image {
                Layout.preferredWidth: 330
                Layout.fillHeight: true
                source: "../../assets/vehicle.png"
                fillMode: Image.PreserveAspectFit
                cache: true
                smooth: true
            }
            ColumnLayout {
                Layout.fillWidth: true
                Text {
                    text: I18n.t("智能 AMR 小车")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.title
                    font.weight: Font.DemiBold
                }
                Text {
                    text: I18n.t("专业自主移动机器人操作平台")
                    color: Theme.textSecondary
                    font.pixelSize: AppMetrics.cardTitle
                }
                StatusBadge {
                    status: backend.snapshot.ros_connected ? "NORMAL" : "WARNING"
                    label: I18n.t(backend.snapshot.ros_connected
                        ? "控制系统正常" : "控制系统未连接")
                }
                Item { Layout.fillHeight: true }
                Text {
                    text: "UI " + (backend.systemInfo.ui_version ?? "V4.2")
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.small
                }
            }
        }
    }

    GridLayout {
        Layout.fillWidth: true
        columns: 3
        columnSpacing: AppMetrics.gap
        rowSpacing: AppMetrics.gap

        AppCard {
            Layout.fillWidth: true
            Layout.preferredHeight: 264
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                SectionHeader { title: I18n.t("硬件平台") }
                DataRow { Layout.fillWidth: true; label: I18n.t("上位机"); value: backend.systemInfo.host_model ?? "--" }
                DataRow { Layout.fillWidth: true; label: I18n.t("设备温度"); value: Number(backend.snapshot.cpu_temperature ?? 0).toFixed(0) + "°C" }
                DataRow { Layout.fillWidth: true; label: I18n.t("内存使用"); value: Number(backend.snapshot.memory_percent ?? 0).toFixed(0) + "%"; showDivider: false }
            }
        }
        AppCard {
            Layout.fillWidth: true
            Layout.preferredHeight: 264
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                SectionHeader { title: I18n.t("存储空间") }
                DataRow { Layout.fillWidth: true; label: I18n.t("总容量"); value: (backend.systemInfo.storage_total_gb ?? "--") + " GB" }
                DataRow { Layout.fillWidth: true; label: I18n.t("已使用"); value: (backend.systemInfo.storage_used_gb ?? "--") + " GB" }
                DataRow { Layout.fillWidth: true; label: I18n.t("可用空间"); value: (backend.systemInfo.storage_free_gb ?? "--") + " GB"; showDivider: false }
            }
        }
        AppCard {
            Layout.fillWidth: true
            Layout.preferredHeight: 264
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                SectionHeader { title: I18n.t("软件与地图") }
                DataRow { Layout.fillWidth: true; label: I18n.t("操作系统"); value: backend.systemInfo.os_name ?? "--" }
                DataRow { Layout.fillWidth: true; label: "ROS 2"; value: backend.systemInfo.ros_distro ?? "--" }
                DataRow { Layout.fillWidth: true; label: I18n.t("当前地图"); value: backend.currentMap.name ?? I18n.t("等待地图") }
                DataRow { Layout.fillWidth: true; label: I18n.t("地图数量"); value: backend.maps.length + ""; showDivider: false }
            }
        }
    }

    AppCard {
        Layout.fillWidth: true
        Layout.fillHeight: true
        RowLayout {
            anchors.fill: parent
            anchors.margins: AppMetrics.cardPadding
            spacing: AppMetrics.sectionGap
            ColumnLayout {
                Layout.preferredWidth: 270
                Text { text: I18n.t("联合制造"); color: Theme.textPrimary; font.pixelSize: AppMetrics.cardTitle; font.weight: Font.DemiBold }
                Text { text: I18n.t("深圳文思汇通科技有限公司"); color: Theme.textSecondary; font.pixelSize: AppMetrics.small }
                Text { text: I18n.t("深圳市洪昕德立科技有限公司"); color: Theme.textSecondary; font.pixelSize: AppMetrics.small }
            }
            Image {
                Layout.fillWidth: true
                Layout.fillHeight: true
                source: Theme.darkMode
                    ? "../../assets/branding/wensihuitong-dark.png"
                    : "../../assets/branding/wensihuitong-light.png"
                fillMode: Image.PreserveAspectFit
                cache: true
            }
            Text {
                text: "&"
                color: Theme.textPrimary
                font.pixelSize: AppMetrics.title
                font.weight: Font.DemiBold
            }
            Image {
                Layout.fillWidth: true
                Layout.fillHeight: true
                source: Theme.darkMode
                    ? "../../assets/branding/hongxindeli-dark.png"
                    : "../../assets/branding/hongxindeli-light.png"
                fillMode: Image.PreserveAspectFit
                cache: true
            }
        }
    }
}

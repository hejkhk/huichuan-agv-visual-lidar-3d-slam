import QtQuick
import QtQuick.Layouts
import ".."

Item {
    id: root
    property var mapData: ({})
    property bool emphasized: false

    AppCard {
        anchors.fill: parent
        color: root.emphasized ? Theme.surfaceElevated : Theme.surface
        border.color: root.emphasized ? Theme.primary : Theme.border
        border.width: root.emphasized ? Theme.borderWidthStrong : Theme.borderWidth

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: root.emphasized
                ? AppMetrics.cardPadding : AppMetrics.gap
            spacing: AppMetrics.unit

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: Theme.radiusSmall * AppMetrics.scale
                color: Theme.mapCanvas
                clip: true

                Image {
                    id: preview
                    anchors.fill: parent
                    anchors.margins: AppMetrics.unit
                    source: root.visible
                        ? (root.mapData.cache_preview_url ?? root.mapData.cache_pgm_url ?? "")
                        : ""
                    sourceSize.width: Math.max(
                        1,
                        root.emphasized ? parent.width * 1.25 * Performance.imageScale : parent.width * Performance.imageScale
                    )
                    sourceSize.height: Math.max(
                        1,
                        root.emphasized ? parent.height * 1.25 * Performance.imageScale : parent.height * Performance.imageScale
                    )
                    asynchronous: true
                    cache: Performance.imageCache
                    fillMode: Image.PreserveAspectFit
                    smooth: !Performance.lowPower
                }

                Text {
                    anchors.centerIn: parent
                    visible: preview.status === Image.Error
                    text: I18n.t("地图预览无法读取")
                    color: Theme.danger
                    font.pixelSize: AppMetrics.small
                }
            }

            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(
                    AppMetrics.touch,
                    titleText.implicitHeight + createdText.implicitHeight
                        + 5 * AppMetrics.scale
                )

                Column {
                    anchors.centerIn: parent
                    spacing: 3 * AppMetrics.scale
                    width: currentBadge.visible
                        ? parent.width - currentBadge.width - AppMetrics.gap
                        : parent.width
                    Text {
                        id: titleText
                        width: parent.width
                        text: root.mapData.name ?? ""
                        color: Theme.textPrimary
                        font.pixelSize: root.emphasized
                            ? AppMetrics.sectionTitle : AppMetrics.body
                        font.weight: Font.DemiBold
                        horizontalAlignment: Text.AlignHCenter
                        elide: Text.ElideRight
                    }
                    Text {
                        id: createdText
                        width: parent.width
                        text: I18n.t("创建时间") + "："
                            + (root.mapData.created_time_text ?? "--")
                        color: Theme.textMuted
                        font.pixelSize: AppMetrics.caption
                        horizontalAlignment: Text.AlignHCenter
                        elide: Text.ElideRight
                    }
                }

                StatusBadge {
                    id: currentBadge
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    visible: root.emphasized && (root.mapData.is_current ?? false)
                    status: "NORMAL"
                    label: I18n.t("当前地图")
                }
            }
        }
    }
}

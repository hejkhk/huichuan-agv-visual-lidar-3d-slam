import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"
import "../dialogs"

Page {
    id: root
    background: Rectangle { color: Theme.pageBackground }

    function closeTransient() {
        if (saveMapDialog.visible) {
            saveMapDialog.close()
            return true
        }
        return false
    }

    SaveMapDialog { id: saveMapDialog }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: AppMetrics.margin
        spacing: AppMetrics.gap

        PageHeader {
            Layout.fillWidth: true
            title: I18n.t("建图模式")
            subtitle: I18n.t("地图与建图控制")
            StatusBadge {
                status: backend.snapshot.mapping_active ? "NORMAL" : "UNKNOWN"
            }
        }

        RvizPlaceholder {
            objectName: "mappingFullscreenMap"
            Layout.fillWidth: true
            Layout.fillHeight: true
            fullscreenPage: true
            interactiveGoalSelection: false
            onFullscreenRequested: window.goBack()
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: AppMetrics.primaryTouch
            spacing: AppMetrics.gap

            DangerButton {
                Layout.fillWidth: true
                text: I18n.t("停止建图")
                onClicked: {
                    backend.stopMapping()
                    window.goBack()
                }
            }
            Item { Layout.fillWidth: true }
            PrimaryButton {
                Layout.fillWidth: true
                accent: Theme.success
                text: I18n.t("保存地图")
                onClicked: saveMapDialog.open()
            }
        }
    }
}

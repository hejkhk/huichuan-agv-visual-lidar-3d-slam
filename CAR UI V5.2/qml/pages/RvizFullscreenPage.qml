import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

Page {
    id: root
    readonly property bool navigationActive:
        backend.navigationControls.cancelEnabled ?? false

    background: Rectangle { color: Theme.pageBackground }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: AppMetrics.margin
        spacing: AppMetrics.gap

        PageHeader {
            Layout.fillWidth: true
            title: I18n.t("全屏地图")
            subtitle: I18n.t("在地图上选择目的地并控制导航")
        }

        RvizPlaceholder {
            objectName: "fullscreenMap"
            Layout.fillWidth: true
            Layout.fillHeight: true
            fullscreenPage: true
            interactiveGoalSelection: true
            navigationToggleVisible: true
            navigationActive: root.navigationActive
            navigationStartEnabled: backend.navigationControls.startEnabled ?? false
            onMapGoalSelected: function(worldX, worldY) {
                backend.selectMapGoal(worldX, worldY)
            }
            onResetRequested: backend.clearNavigationSelection()
            onNavigationToggleRequested: {
                if (root.navigationActive)
                    backend.cancelNavigation()
                else
                    backend.startSelectedNavigation()
            }
            onFullscreenRequested: window.goBack()
        }

    }
}

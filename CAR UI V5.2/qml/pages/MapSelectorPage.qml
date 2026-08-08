import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"
import "../dialogs"

Page {
    id: root
    objectName: "mapSelectorPage"
    background: Rectangle { color: Theme.pageBackground }
    property var selectedMap: carousel.currentMap
    property var useAvailability: ({ allowed: false, reason: "" })
    property var renameAvailability: ({ allowed: false, reason: "" })
    property var deleteAvailability: ({ allowed: false, reason: "" })

    function openMappingDialog() {
        mappingConfirmDialog.open()
    }

    function closeTransient() {
        if (renameDialog.visible) {
            renameDialog.close()
            return true
        }
        if (deleteDialog.visible) {
            deleteDialog.close()
            return true
        }
        if (mappingConfirmDialog.visible) {
            mappingConfirmDialog.close()
            return true
        }
        return false
    }

    function updateAvailability() {
        if (!selectedMap || !selectedMap.id) {
            useAvailability = { allowed: false, reason: "" }
            renameAvailability = { allowed: false, reason: "" }
            deleteAvailability = { allowed: false, reason: "" }
            return
        }
        useAvailability = backend.mapActionAvailability(selectedMap.id, "use")
        renameAvailability = backend.mapActionAvailability(selectedMap.id, "rename")
        deleteAvailability = backend.mapActionAvailability(selectedMap.id, "delete")
    }

    function preserveSelection() {
        var id = selectedMap && selectedMap.id ? selectedMap.id : ""
        if (!id || backend.maps.length === 0) {
            carousel.currentIndex = backend.maps.length > 0 ? 0 : -1
            return
        }
        for (var i = 0; i < backend.maps.length; ++i) {
            if (backend.maps[i].id === id) {
                carousel.currentIndex = i
                return
            }
        }
        carousel.currentIndex = Math.min(
            Math.max(0, carousel.currentIndex),
            backend.maps.length - 1
        )
    }

    function focusCurrentMap() {
        var currentId = backend.currentMap.id ?? ""
        if (!currentId)
            return
        for (var i = 0; i < backend.maps.length; ++i) {
            if (backend.maps[i].id === currentId) {
                carousel.currentIndex = i
                return
            }
        }
    }

    Component.onCompleted: {
        backend.refreshMaps()
        updateAvailability()
        Qt.callLater(focusCurrentMap)
    }
    onSelectedMapChanged: updateAvailability()

    Connections {
        target: backend
        function onMapsChanged() {
            root.preserveSelection()
            root.updateAvailability()
        }
        function onSnapshotChanged() {
            root.updateAvailability()
        }
        function onMapOperationFinished(action, success, mapId) {
            root.preserveSelection()
            root.updateAvailability()
            if (action === "use" && success)
                window.goHome()
        }
    }

    RenameMapDialog { id: renameDialog }
    MappingConfirmDialog {
        id: mappingConfirmDialog
        onAcceptedAction: {
            backend.startMapping()
            window.pushPage("MappingFullscreenPage.qml")
        }
        onTutorialRequested: window.pushPageWithProperties(
            "GamepadTutorialPage.qml",
            { "reopenMappingOnReturn": true }
        )
    }
    ConfirmDialog {
        id: deleteDialog
        title: I18n.t("删除地图")
        message: I18n.t("确认删除地图") + "“"
            + (root.selectedMap.name ?? "") + "”？\n"
            + I18n.t("地图删除后无法恢复。")
        onAcceptedAction: backend.deleteMap(root.selectedMap.id)
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: AppMetrics.margin
        spacing: AppMetrics.gap

        PageHeader {
            Layout.fillWidth: true
            title: I18n.t("地图选择与管理")
            subtitle: I18n.t("选择机器人要使用的地图")
            RowLayout {
                spacing: AppMetrics.unit
                StatusBadge {
                    status: backend.mapOperationState.status === "ERROR"
                        ? "ERROR"
                        : backend.mapOperationState.status === "IDLE"
                            ? "NORMAL" : "WARNING"
                    label: I18n.t(backend.mapOperationState.status === "SYNCING"
                        ? "正在整理地图"
                        : backend.mapOperationState.status === "ERROR"
                            ? "地图准备失败" : "地图已准备好")
                }
                SecondaryButton {
                    compact: true
                    text: I18n.t("重新检查")
                    enabled: !backend.busy
                    onClicked: backend.refreshMaps()
                }
                PrimaryButton {
                    compact: true
                    text: I18n.t("创建新地图")
                    onClicked: mappingConfirmDialog.open()
                }
            }
        }

        AppCard {
            Layout.fillWidth: true
            Layout.fillHeight: true

            EmptyState {
                anchors.centerIn: parent
                visible: backend.maps.length === 0
                    && backend.mapOperationState.status !== "SYNCING"
                title: I18n.t("还没有可用的地图")
                description: I18n.t("请先创建或导入一张完整地图")
            }

            EmptyState {
                anchors.centerIn: parent
                visible: backend.maps.length === 0
                    && backend.mapOperationState.status === "SYNCING"
                title: I18n.t("正在整理地图")
                description: I18n.t("地图准备完成后会自动显示")
            }

            RowLayout {
                anchors.fill: parent
                anchors.margins: AppMetrics.cardPadding
                spacing: AppMetrics.gap
                visible: backend.maps.length > 0

                Item {
                    Layout.preferredWidth: AppMetrics.primaryTouch
                    Layout.fillHeight: true
                    AppIconButton {
                        anchors.centerIn: parent
                        text: "←"
                        accessibleName: I18n.t("上一张地图")
                        enabled: !carousel.animating
                            && backend.maps.length > 1
                        onClicked: carousel.previous()
                    }
                }

                MapCarousel {
                    id: carousel
                    objectName: "mapCarousel"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    maps: backend.maps
                    onSelectionChanged: function(mapData) {
                        root.selectedMap = mapData
                        root.updateAvailability()
                    }
                }

                Item {
                    Layout.preferredWidth: AppMetrics.primaryTouch
                    Layout.fillHeight: true
                    AppIconButton {
                        anchors.centerIn: parent
                        text: "→"
                        accessibleName: I18n.t("下一张地图")
                        enabled: !carousel.animating
                            && backend.maps.length > 1
                        onClicked: carousel.next()
                    }
                }
            }
        }

        Text {
            Layout.fillWidth: true
            visible: backend.mapErrors.length > 0
            text: backend.mapErrors.length > 0
                ? I18n.t("地图异常") + "："
                    + (backend.mapErrors[0].name ?? "") + " "
                    + I18n.t(backend.mapErrors[0].error ?? "")
                : ""
            color: Theme.danger
            font.pixelSize: AppMetrics.small
            elide: Text.ElideRight
        }

        Text {
            Layout.fillWidth: true
            visible: backend.maps.length > 0
                && (!useAvailability.allowed
                    || !renameAvailability.allowed
                    || !deleteAvailability.allowed)
            text: useAvailability.reason
                || renameAvailability.reason
                || deleteAvailability.reason
            color: Theme.warning
            font.pixelSize: AppMetrics.small
            horizontalAlignment: Text.AlignHCenter
            elide: Text.ElideRight
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: AppMetrics.primaryTouch
            Layout.minimumHeight: AppMetrics.primaryTouch
            Layout.maximumWidth: 980 * AppMetrics.scale
            Layout.maximumHeight: AppMetrics.primaryTouch
            Layout.alignment: Qt.AlignHCenter
            spacing: AppMetrics.gap
            visible: backend.maps.length > 0

            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: AppMetrics.primaryTouch
                DangerButton {
                    anchors.fill: parent
                    text: I18n.t("删除")
                    enabled: deleteAvailability.allowed && !backend.busy
                    onClicked: deleteDialog.open()
                }
                TapHandler {
                    enabled: !deleteAvailability.allowed
                    onTapped: backend.showMapBlockedReason(
                        root.selectedMap.id, "delete"
                    )
                }
            }
            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: AppMetrics.primaryTouch
                SecondaryButton {
                    anchors.fill: parent
                    text: I18n.t("重命名")
                    enabled: renameAvailability.allowed && !backend.busy
                    onClicked: {
                        renameDialog.mapId = root.selectedMap.id
                        renameDialog.currentName = root.selectedMap.name
                        renameDialog.open()
                    }
                }
                TapHandler {
                    enabled: !renameAvailability.allowed
                    onTapped: backend.showMapBlockedReason(
                        root.selectedMap.id, "rename"
                    )
                }
            }
            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: AppMetrics.primaryTouch
                PrimaryButton {
                    anchors.fill: parent
                    text: backend.mapOperationState.status === "LOADING_MAP"
                        ? I18n.t("正在加载地图") : I18n.t("使用地图")
                    busy: backend.mapOperationState.status === "LOADING_MAP"
                    enabled: useAvailability.allowed && !backend.busy
                    onClicked: backend.useMap(root.selectedMap.id)
                }
                TapHandler {
                    enabled: !useAvailability.allowed
                    onTapped: backend.showMapBlockedReason(
                        root.selectedMap.id, "use"
                    )
                }
            }
        }
    }
}

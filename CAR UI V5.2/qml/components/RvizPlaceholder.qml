import QtQuick
import QtQuick.Layouts
import ".."
import "../dialogs"

AppCard {
    id: root
    signal fullscreenRequested()
    signal mapManagementRequested()
    signal createMapRequested()
    signal mapGoalSelected(real worldX, real worldY)
    signal resetRequested()
    signal navigationToggleRequested()
    property bool navigationToggleVisible: false
    property bool navigationActive: false
    property bool navigationStartEnabled: false
    property bool interactiveGoalSelection: false
    property bool fullscreenPage: false
    property bool mapManagementVisible: false
    property bool createMapVisible: false
    property bool headingUp: false
    property real zoomLevel: 1.0
    property real panX: 0
    property real panY: 0
    readonly property real hostCpuPercent:
        Number(backend.snapshot.cpu_percent ?? 0)
    // High load pauses only continuous gestures. Goal taps intentionally
    // remain available at every CPU level as a safety-critical operation.
    readonly property bool mapGesturesAllowed: hostCpuPercent <= 70
    readonly property bool hasMap: backend.snapshot.map_available ?? false
    readonly property bool goalSelectionAllowed:
        interactiveGoalSelection && hasMap
    readonly property real mapMetersWide: (backend.snapshot.map_width ?? 0) * (backend.snapshot.map_resolution ?? 0)
    readonly property real mapMetersHigh: (backend.snapshot.map_height ?? 0) * (backend.snapshot.map_resolution ?? 0)

    function pixelX(worldX) {
        if (mapMetersWide <= 0) return 0
        return (worldX - (backend.snapshot.map_origin_x ?? 0)) / mapMetersWide * mapLayer.width
    }
    function pixelY(worldY) {
        if (mapMetersHigh <= 0) return 0
        return mapLayer.height - (worldY - (backend.snapshot.map_origin_y ?? 0)) / mapMetersHigh * mapLayer.height
    }
    function worldX(pixel) {
        if (mapLayer.width <= 0) return backend.snapshot.map_origin_x ?? 0
        return (backend.snapshot.map_origin_x ?? 0) + pixel / mapLayer.width * mapMetersWide
    }
    function worldY(pixel) {
        if (mapLayer.height <= 0) return backend.snapshot.map_origin_y ?? 0
        return (backend.snapshot.map_origin_y ?? 0) + (mapLayer.height - pixel) / mapLayer.height * mapMetersHigh
    }
    function clampZoom(value) {
        return Math.max(0.5, Math.min(6.0, value))
    }
    function transformedOffset(localX, localY, scaleValue) {
        var angle = mapLayer.rotation * Math.PI / 180
        var dx = localX - mapLayer.width / 2
        var dy = localY - mapLayer.height / 2
        return Qt.point(
            scaleValue * (dx * Math.cos(angle) - dy * Math.sin(angle)),
            scaleValue * (dx * Math.sin(angle) + dy * Math.cos(angle))
        )
    }
    function setZoomAt(value, viewportX, viewportY) {
        var nextZoom = clampZoom(value)
        if (headingUp) {
            zoomLevel = nextZoom
            updateHeadingTransform()
            return
        }
        var local = mapLayer.mapFromItem(viewport, viewportX, viewportY)
        var offset = transformedOffset(local.x, local.y, nextZoom)
        zoomLevel = nextZoom
        panX = viewportX - viewport.width / 2 - offset.x
        panY = viewportY - viewport.height / 2 - offset.y
    }
    function updateHeadingTransform() {
        if (!headingUp || !hasMap
                || !(backend.snapshot.pose_available ?? false))
            return
        var pose = backend.snapshot.current_pose ?? ({ x: 0, y: 0, yaw: 0 })
        var offset = transformedOffset(pixelX(pose.x), pixelY(pose.y), zoomLevel)
        panX = -offset.x
        panY = -offset.y
    }
    function resetView() {
        headingUp = false
        zoomLevel = 1.0
        panX = 0
        panY = 0
    }
    onHeadingUpChanged: {
        if (headingUp)
            updateHeadingTransform()
        else {
            panX = 0
            panY = 0
        }
    }
    onZoomLevelChanged: if (headingUp) updateHeadingTransform()

    color: Theme.surfaceElevated
    clip: true

    HeadingDialog { id: mapHeadingDialog }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: AppMetrics.gap
        spacing: AppMetrics.unit

        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.fillWidth: true
                text: I18n.t(root.hasMap ? "地图 · 已准备好" : "地图 · 等待控制系统响应")
                color: Theme.textPrimary
                font.pixelSize: AppMetrics.body
                font.bold: true
            }
            PrimaryButton {
                objectName: "mapCreateButton"
                visible: root.createMapVisible
                text: I18n.t("创建新地图")
                compact: true
                onClicked: root.createMapRequested()
            }
            AppButton {
                objectName: "mapManagementButton"
                visible: root.mapManagementVisible
                text: I18n.t("地图管理")
                implicitWidth: 146 * AppMetrics.scale
                implicitHeight: 40 * AppMetrics.scale
                outlined: true
                onClicked: root.mapManagementRequested()
            }
            AppButton {
                text: I18n.t(root.fullscreenPage ? "退出全屏" : "全屏")
                implicitWidth: 170 * AppMetrics.scale
                implicitHeight: 40 * AppMetrics.scale
                accent: Theme.primary
                onClicked: root.fullscreenRequested()
            }
        }

        Rectangle {
            id: viewport
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: 10 * AppMetrics.scale
            color: Theme.mapCanvas
            clip: true

            Item {
                id: fittedMap
                anchors.centerIn: parent
                clip: false
                width: root.mapMetersWide > 0 && root.mapMetersHigh > 0
                    ? Math.min(parent.width, parent.height * root.mapMetersWide / root.mapMetersHigh)
                    : parent.width
                height: root.mapMetersWide > 0 && root.mapMetersHigh > 0
                    ? Math.min(parent.height, parent.width * root.mapMetersHigh / root.mapMetersWide)
                    : parent.height

                Item {
                    id: mapLayer
                    width: parent.width
                    height: parent.height
                    x: root.panX
                    y: root.panY
                    clip: false
                    scale: root.zoomLevel
                    rotation: root.headingUp
                        ? (backend.snapshot.current_pose?.yaw ?? 0)
                            * 180 / Math.PI + 90
                        : 0

                    Image {
                        anchors.fill: parent
                        visible: root.hasMap && root.visible
                        source: root.visible ? backend.mapImageSource : ""
                        fillMode: Image.Stretch
                        smooth: false
                        cache: true
                        asynchronous: true
                    }

                    Timer {
                        id: overlayThrottle
                        interval: Performance.mapPaintInterval; repeat: false
                        onTriggered: overlay.requestPaint()
                    }

                    Canvas {
                        id: overlay
                        anchors.fill: parent
                        visible: root.hasMap && root.visible
                        renderStrategy: Canvas.Threaded
                        property var renderSnapshot: ({ laser_points: [], path_points: [] })

                        Component.onCompleted: renderSnapshot = backend.snapshot
                        onVisibleChanged: {
                            if (visible) {
                                renderSnapshot = backend.snapshot
                                requestPaint()
                            }
                        }

                        onPaint: {
                            var snap = renderSnapshot
                            var path = snap.path_points ?? []
                            var scan = snap.laser_points ?? []

                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)

                            if (path.length > 1) {
                                ctx.beginPath()
                                ctx.lineWidth = Math.max(2, 4 * AppMetrics.scale / root.zoomLevel)
                                ctx.strokeStyle = "#1c8cff"
                                ctx.moveTo(root.pixelX(path[0][0]), root.pixelY(path[0][1]))
                                for (var i = 1; i < path.length; ++i)
                                    ctx.lineTo(root.pixelX(path[i][0]), root.pixelY(path[i][1]))
                                ctx.stroke()
                            }

                            if (scan.length > 0) {
                                ctx.fillStyle = "#ef3f52"
                                var dot = Math.max(1.5, 2.5 * AppMetrics.scale / root.zoomLevel)
                                var rx = snap.current_pose?.x ?? 0
                                var ry = snap.current_pose?.y ?? 0
                                ctx.beginPath()
                                for (var j = 0; j < scan.length; ++j) {
                                    // 180° rotation around robot position to fix localization yaw offset
                                    var px = root.pixelX(rx - (scan[j][0] - rx))
                                    var py = root.pixelY(ry - (scan[j][1] - ry))
                                    ctx.rect(px - dot / 2, py - dot / 2, dot, dot)
                                }
                                ctx.fill()
                            }
                        }

                        Connections {
                            enabled: root.visible && root.hasMap
                            target: backend
                            function onMapOverlayChanged() {
                                overlay.renderSnapshot = backend.snapshot
                                overlayThrottle.restart()
                            }
                        }
                    }

                    Item {
                        id: robotMarker
                        visible: root.hasMap && (backend.snapshot.pose_available ?? false)
                        x: root.pixelX(backend.snapshot.current_pose?.x ?? 0) - width / 2
                        y: root.pixelY(backend.snapshot.current_pose?.y ?? 0) - height / 2
                        width: 34 * AppMetrics.scale / root.zoomLevel
                        height: width
                        rotation: -(backend.snapshot.current_pose?.yaw ?? 0) * 180 / Math.PI + 180

                        Rectangle {
                            anchors.fill: parent
                            radius: 7 * AppMetrics.scale / root.zoomLevel
                            color: Theme.primary
                            border.width: 4 * AppMetrics.scale / root.zoomLevel
                            border.color: "white"
                        }
                        Rectangle {
                            width: parent.width * 0.55
                            height: Math.max(2, 5 * AppMetrics.scale / root.zoomLevel)
                            radius: height / 2
                            color: "white"
                            anchors.left: parent.horizontalCenter
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }

                    Item {
                        id: goalMarker
                        readonly property var goal: (backend.hasMapGoal ?? false)
                            ? backend.mapGoal
                            : backend.selectedPoint
                        readonly property bool temporary: backend.hasMapGoal ?? false
                        visible: root.hasMap && root.visible
                            && ((backend.hasMapGoal ?? false) || (backend.selectedPointId ?? "") !== "")
                        z: 5
                        x: root.pixelX(goal?.x ?? 0) - width / 2
                        y: root.pixelY(goal?.y ?? 0) - height / 2
                        width: 70 * AppMetrics.scale / root.zoomLevel
                        height: width

                        Rectangle {
                            anchors.centerIn: parent
                            width: 32 * AppMetrics.scale / root.zoomLevel
                            height: width
                            radius: width / 2
                            color: goalMarker.temporary ? Theme.danger : Theme.warning
                            border.width: 3 * AppMetrics.scale / root.zoomLevel
                            border.color: "white"
                        }

                        Item {
                            anchors.centerIn: parent
                            width: parent.width
                            height: parent.height
                            rotation: -((((Number(goalMarker.goal?.yaw ?? 0) * 180 / Math.PI) % 360) + 360) % 360)
                            Rectangle {
                                anchors.left: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                width: parent.width * 0.42
                                height: 5 * AppMetrics.scale / root.zoomLevel
                                radius: height / 2
                                color: goalMarker.temporary ? Theme.danger : Theme.warning
                            }
                            Text {
                                anchors.right: parent.right
                                anchors.verticalCenter: parent.verticalCenter
                                text: "▶"
                                color: goalMarker.temporary ? Theme.danger : Theme.warning
                                font.pixelSize: 23 * AppMetrics.scale / root.zoomLevel
                                font.bold: true
                            }
                        }
                    }

                }
            }

            TapHandler {
                id: mapGoalTap
                enabled: root.goalSelectionAllowed
                acceptedButtons: Qt.LeftButton
                gesturePolicy: TapHandler.DragThreshold
                onTapped: function(eventPoint, _button) {
                    var point = mapLayer.mapFromItem(
                        viewport, eventPoint.position.x, eventPoint.position.y
                    )
                    if (point.x >= 0 && point.x <= mapLayer.width
                            && point.y >= 0 && point.y <= mapLayer.height)
                        root.mapGoalSelected(
                            root.worldX(point.x), root.worldY(point.y)
                        )
                }
            }

            DragHandler {
                id: mapDrag
                enabled: root.mapGesturesAllowed && root.hasMap
                target: null
                acceptedButtons: Qt.LeftButton
                property real startingPanX: 0
                property real startingPanY: 0
                onActiveChanged: {
                    if (active) {
                        if (root.headingUp)
                            root.headingUp = false
                        startingPanX = root.panX
                        startingPanY = root.panY
                    }
                }
                onActiveTranslationChanged: {
                    if (!active)
                        return
                    root.panX = startingPanX + activeTranslation.x
                    root.panY = startingPanY + activeTranslation.y
                }
            }

            WheelHandler {
                id: mapWheel
                enabled: root.mapGesturesAllowed && root.hasMap
                acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
                blocking: true
                onWheel: function(event) {
                    var factor = event.angleDelta.y > 0 ? 1.14 : 1 / 1.14
                    root.setZoomAt(
                        root.zoomLevel * factor,
                        event.x,
                        event.y
                    )
                    event.accepted = true
                }
            }

            PinchHandler {
                id: mapPinch
                enabled: root.mapGesturesAllowed && root.hasMap
                target: null
                minimumScale: 0.5
                maximumScale: 6.0
                property real startingZoom: 1
                property real startingPanX: 0
                property real startingPanY: 0
                onActiveChanged: {
                    if (active) {
                        startingZoom = root.zoomLevel
                        startingPanX = root.panX
                        startingPanY = root.panY
                    } else if (root.headingUp) {
                        root.updateHeadingTransform()
                    }
                }
                onActiveScaleChanged: {
                    root.zoomLevel = root.clampZoom(
                        startingZoom * activeScale
                    )
                    if (!root.headingUp) {
                        root.panX = startingPanX + activeTranslation.x
                        root.panY = startingPanY + activeTranslation.y
                    }
                }
            }

            HoverHandler {
                cursorShape: mapDrag.active
                    ? Qt.ClosedHandCursor : Qt.CrossCursor
            }

            Rectangle {
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.topMargin: AppMetrics.unit
                z: 30
                visible: root.hasMap && !root.mapGesturesAllowed
                width: loadNotice.implicitWidth + 28 * AppMetrics.scale
                height: loadNotice.implicitHeight + 14 * AppMetrics.scale
                radius: height / 2
                color: Theme.warning
                opacity: 0.94
                Text {
                    id: loadNotice
                    anchors.centerIn: parent
                    text: I18n.t("设备负载较高，已暂停拖动和缩放；仍可点击目标")
                    color: "white"
                    font.pixelSize: AppMetrics.small
                    font.weight: Font.DemiBold
                }
            }

            Column {
                anchors.centerIn: parent
                visible: !root.hasMap
                spacing: 8 * AppMetrics.scale
                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: I18n.t("正在获取地图")
                    color: Theme.textSecondary
                    font.pixelSize: AppMetrics.body
                    font.bold: true
                }
                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: I18n.t("控制系统尚未准备好，请稍候")
                    color: Theme.textMuted
                    font.pixelSize: AppMetrics.small
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            AppButton {
                text: I18n.t("放大")
                outlined: true
                onClicked: root.setZoomAt(
                    root.zoomLevel * 1.25,
                    viewport.width / 2, viewport.height / 2
                )
            }
            AppButton {
                text: I18n.t("缩小")
                outlined: true
                onClicked: root.setZoomAt(
                    root.zoomLevel / 1.25,
                    viewport.width / 2, viewport.height / 2
                )
            }
            AppButton {
                text: I18n.t("回到全图")
                accent: Theme.textMuted
                onClicked: {
                    root.resetView()
                    root.resetRequested()
                }
            }
            AppButton {
                objectName: "editMapGoalHeadingButton"
                visible: root.interactiveGoalSelection
                text: I18n.t("编辑目标朝向")
                outlined: true
                enabled: backend.hasMapGoal ?? false
                onClicked: mapHeadingDialog.openForMapGoal()
            }
            AppButton {
                objectName: "cancelMapGoalSelectionButton"
                visible: root.interactiveGoalSelection
                text: I18n.t("取消导航点选择")
                outlined: true
                accent: Theme.danger
                enabled: backend.hasMapGoal ?? false
                onClicked: backend.clearNavigationSelection()
            }
            RowLayout {
                visible: root.fullscreenPage
                spacing: AppMetrics.unit
                AppSwitch {
                    checked: root.headingUp
                    onToggled: root.headingUp = checked
                }
                Text {
                    text: I18n.t("车头朝上并自动居中")
                    color: Theme.textSecondary
                    font.pixelSize: AppMetrics.small
                }
            }
            AppButton {
                objectName: "fullscreenNavigationButton"
                visible: root.navigationToggleVisible
                text: I18n.t(root.navigationActive ? "结束导航" : "开始导航")
                enabled: root.navigationActive || root.navigationStartEnabled
                accent: root.navigationActive ? Theme.danger : Theme.primary
                onClicked: root.navigationToggleRequested()
            }
            Item { Layout.fillWidth: true }
            Text {
                visible: (backend.hasMapGoal ?? false) || (backend.selectedPointId ?? "") !== ""
                readonly property var displayedGoal: (backend.hasMapGoal ?? false)
                    ? backend.mapGoal
                    : backend.selectedPoint
                text: I18n.t("已选择地图上的位置") + "  ➜  "
                    + Math.round((((Number(displayedGoal?.yaw ?? 0) * 180 / Math.PI) % 360) + 360) % 360)
                    + "°"
                color: (backend.hasMapGoal ?? false) ? Theme.danger : Theme.warning
                font.pixelSize: AppMetrics.small
            }
            Text {
                text: I18n.t(root.hasMap ? "地图已准备好" : "地图服务未连接")
                color: root.hasMap ? Theme.success : Theme.textMuted
                font.pixelSize: AppMetrics.small
                elide: Text.ElideRight
            }
        }
    }

    Connections {
        target: backend
        function onSnapshotChanged() {
            if (root.headingUp)
                root.updateHeadingTransform()
        }
    }
}

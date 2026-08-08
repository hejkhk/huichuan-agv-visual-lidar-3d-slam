import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"
import "../dialogs"

Page {
    id: root
    property int pageIndex: 0
    property int pageSize: 5
    property var renameTarget: ({})
    property var deleteTarget: ({})
    property var headingTarget: ({})

    function closeTransient() {
        if (deleteDialog.visible) {
            deleteDialog.close()
            return true
        }
        if (renameDialog.visible) {
            renameDialog.close()
            return true
        }
        if (headingDialog.visible) {
            headingDialog.close()
            return true
        }
        if (addDialog.visible) {
            addDialog.dismiss()
            return true
        }
        return false
    }

    background: Rectangle { color: Theme.pageBackground }

    AddPointDialog { id: addDialog }
    RenamePointDialog { id: renameDialog; point: root.renameTarget }
    HeadingDialog { id: headingDialog }
    ConfirmDialog {
        id: deleteDialog
        title: I18n.t("删除目标点")
        message: deleteTarget.is_charging_point
            ? I18n.t("这是当前充电点，删除后将无法一键回充。确认删除？")
            : I18n.t("确认删除") + " “" + I18n.t(deleteTarget.name ?? "") + "”?"
        onAcceptedAction: backend.deletePoint(deleteTarget.id)
    }
    Connections {
        target: backend
        function onCurrentPoseReady(pose) {
            addDialog.pose = pose
            addDialog.open()
        }
    }

    PageHeader {
        id: pointManagerHeader
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.topMargin: AppMetrics.margin
        anchors.leftMargin: AppMetrics.margin
        anchors.rightMargin: AppMetrics.margin
        title: I18n.t("导航管理")
        subtitle: I18n.t("管理目的地")
        AppButton {
            text: I18n.t("新增目标点")
            accent: Theme.primary
            onClicked: backend.requestCurrentPose()
        }
    }

    RowLayout {
        id: managerLayout
        anchors.fill: parent
        anchors.leftMargin: AppMetrics.margin
        anchors.rightMargin: AppMetrics.margin
        anchors.topMargin: AppMetrics.margin + pointManagerHeader.implicitHeight
            + AppMetrics.gap
        anchors.bottomMargin: AppMetrics.margin
        spacing: AppMetrics.gap

        ColumnLayout {
            Layout.fillHeight: true
            Layout.preferredWidth: (managerLayout.width - managerLayout.spacing) * 0.36
            Layout.minimumWidth: (managerLayout.width - managerLayout.spacing) * 0.34
            Layout.maximumWidth: (managerLayout.width - managerLayout.spacing) * 0.38
            spacing: AppMetrics.unit

            ListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 7 * AppMetrics.scale
                clip: true
                model: backend.points.slice(root.pageIndex * root.pageSize, root.pageIndex * root.pageSize + root.pageSize)

                delegate: PointListItem {
                    required property var modelData
                    width: ListView.view.width
                    height: 66 * AppMetrics.scale
                    point: modelData
                    onRenameRequested: {
                        root.renameTarget = modelData
                        renameDialog.open()
                    }
                    onHeadingRequested: {
                        root.headingTarget = modelData
                        headingDialog.openForPoint(modelData)
                    }
                    onDeleteRequested: {
                        root.deleteTarget = modelData
                        deleteDialog.open()
                    }
                    onAddRequested: backend.addRoutePoint(modelData.id)
                }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: AppMetrics.unit

                AppButton {
                    text: I18n.t("上一页")
                    implicitWidth: 68 * AppMetrics.scale
                    outlined: true
                    enabled: root.pageIndex > 0
                    onClicked: root.pageIndex--
                }
                Rectangle {
                    implicitWidth: 42 * AppMetrics.scale
                    implicitHeight: AppMetrics.touch
                    radius: 9 * AppMetrics.scale
                    color: Theme.surface
                    border.color: Theme.border
                    border.width: Theme.borderWidth
                    Text {
                        anchors.centerIn: parent
                        text: root.pageIndex + 1
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.body
                        font.bold: true
                    }
                }
                AppButton {
                    text: I18n.t("下一页")
                    implicitWidth: 68 * AppMetrics.scale
                    outlined: true
                    enabled: (root.pageIndex + 1) * root.pageSize < backend.points.length
                    onClicked: root.pageIndex++
                }

                Item { Layout.fillWidth: true }

                Text {
                    text: I18n.t("多选点") + "\n" + I18n.t("按顺序导航")
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.small
                    horizontalAlignment: Text.AlignRight
                }
                AppSwitch { id: orderedSwitch; checked: true }
            }
        }

        ColumnLayout {
            Layout.fillHeight: true
            Layout.preferredWidth: (managerLayout.width - managerLayout.spacing) * 0.64
            Layout.minimumWidth: (managerLayout.width - managerLayout.spacing) * 0.62
            Layout.maximumWidth: (managerLayout.width - managerLayout.spacing) * 0.66
            spacing: AppMetrics.unit

            RvizPlaceholder {
                Layout.fillWidth: true
                Layout.preferredHeight: root.height * 0.60
                onFullscreenRequested: {
                    backend.rvizAction("fullscreen")
                    window.pushPage("RvizFullscreenPage.qml")
                }
            }

            AppCard {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: Theme.radius * AppMetrics.scale

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 8 * AppMetrics.scale
                    spacing: AppMetrics.unit

                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 6 * AppMetrics.scale

                        RowLayout {
                            Layout.fillWidth: true
                            Text {
                                Layout.fillWidth: true
                                text: I18n.t("多路径点导航")
                                color: Theme.textPrimary
                                font.pixelSize: AppMetrics.title
                                font.bold: true
                            }
                            Text {
                                text: backend.routePoints.length + " " + I18n.t("个点")
                                color: Theme.textMuted
                                font.pixelSize: AppMetrics.small
                            }
                        }

                        GridView {
                            id: routeGrid
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            clip: true
                            model: backend.routePoints
                            cellWidth: width / 2
                            cellHeight: 43 * AppMetrics.scale

                            delegate: Item {
                                required property var modelData
                                required property int index
                                width: routeGrid.cellWidth
                                height: routeGrid.cellHeight

                                Rectangle {
                                    anchors.fill: parent
                                    anchors.margins: 3 * AppMetrics.scale
                                    radius: 10 * AppMetrics.scale
                                    color: Theme.surfaceMuted
                                    border.color: Theme.border
                                    border.width: Theme.borderWidth

                                    RowLayout {
                                        anchors.fill: parent
                                        anchors.margins: 6 * AppMetrics.scale
                                        spacing: 6 * AppMetrics.scale

                                        Rectangle {
                                            width: 28 * AppMetrics.scale
                                            height: width
                                            radius: width / 2
                                            color: Theme.primarySoft
                                            Text {
                                                anchors.centerIn: parent
                                                text: index + 1
                                                color: Theme.primary
                                                font.pixelSize: AppMetrics.small
                                                font.bold: true
                                            }
                                        }
                                        Text {
                                            Layout.fillWidth: true
                                            text: I18n.t(modelData.name)
                                            color: Theme.textPrimary
                                            font.pixelSize: AppMetrics.body
                                            elide: Text.ElideRight
                                        }
                                        Text {
                                            readonly property int degrees:
                                                Math.round((((Number(modelData.yaw ?? 0) * 180 / Math.PI) % 360) + 360) % 360)
                                            text: "➜"
                                            rotation: -degrees
                                            color: Theme.primary
                                            font.pixelSize: AppMetrics.body
                                            font.bold: true
                                        }
                                        Text {
                                            readonly property int degrees:
                                                Math.round((((Number(modelData.yaw ?? 0) * 180 / Math.PI) % 360) + 360) % 360)
                                            text: degrees + "°"
                                            color: Theme.textMuted
                                            font.pixelSize: AppMetrics.caption
                                        }
                                        AppButton {
                                            text: I18n.t("移除")
                                            implicitWidth: 70 * AppMetrics.scale
                                            implicitHeight: 31 * AppMetrics.scale
                                            accent: Theme.danger
                                            onClicked: backend.removeRoutePoint(modelData.id)
                                        }
                                    }
                                }
                            }

                            Text {
                                anchors.centerIn: parent
                                visible: backend.routePoints.length === 0
                                text: I18n.t("请从左侧加入路径点")
                                color: Theme.textMuted
                                font.pixelSize: AppMetrics.body
                            }
                        }
                    }

                    ColumnLayout {
                        Layout.preferredWidth: 112 * AppMetrics.scale
                        Layout.fillHeight: true
                        spacing: 6 * AppMetrics.scale

                        Item { Layout.fillHeight: true }

                        AppButton {
                            Layout.fillWidth: true
                            Layout.preferredHeight: AppMetrics.touch
                            implicitWidth: 0
                            text: I18n.t("清空路径")
                            outlined: true
                            accent: Theme.danger
                            enabled: backend.routePoints.length > 0
                            onClicked: backend.clearRoute()
                        }
                        AppButton {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 58 * AppMetrics.scale
                            implicitWidth: 0
                            text: I18n.t("开始")
                            accent: Theme.success
                            enabled: backend.routePoints.length > 0
                            onClicked: {
                                backend.startRoute(orderedSwitch.checked)
                                window.goHome()
                            }
                        }
                    }
                }
            }
        }
    }
}

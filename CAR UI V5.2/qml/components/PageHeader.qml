import QtQuick
import QtQuick.Layouts
import ".."

Item {
    id: root
    property string title: ""
    property string subtitle: ""
    property bool showCloseButton: true
    property var closeAction: null
    property string closeObjectName: "pageCloseButton"
    default property alias actions: actionSlot.data
    implicitHeight: 52 * AppMetrics.scale

    RowLayout {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.right: closeButton.visible ? closeButton.left : parent.right
        anchors.rightMargin: closeButton.visible ? AppMetrics.gap : 0
        spacing: AppMetrics.gap

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 1
            Text { Layout.fillWidth: true; text: root.title; color: Theme.textPrimary; font.pixelSize: AppMetrics.title; font.weight: Font.DemiBold; elide: Text.ElideRight }
            Text { Layout.fillWidth: true; visible: root.subtitle.length > 0; text: root.subtitle; color: Theme.textMuted; font.pixelSize: AppMetrics.caption; elide: Text.ElideRight }
        }
        RowLayout { id: actionSlot; spacing: AppMetrics.unit }
    }

    AppIconButton {
        id: closeButton
        objectName: root.closeObjectName
        anchors.top: parent.top
        anchors.right: parent.right
        visible: root.showCloseButton
        text: "×"
        accessibleName: I18n.t("关闭页面")
        implicitWidth: 48
        implicitHeight: 48
        outlined: true
        font.pixelSize: AppMetrics.sectionTitle
        onClicked: {
            if (root.closeAction)
                root.closeAction()
            else
                window.goBack()
        }
    }
}

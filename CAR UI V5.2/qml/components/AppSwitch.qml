import QtQuick
import QtQuick.Controls
import ".."

Switch {
    id: control
    implicitWidth: 62 * AppMetrics.scale
    implicitHeight: 36 * AppMetrics.scale
    focusPolicy: Qt.StrongFocus

    indicator: Rectangle {
        implicitWidth: 58 * AppMetrics.scale
        implicitHeight: 32 * AppMetrics.scale
        radius: height / 2
        color: !control.enabled ? Theme.disabledBackground : control.checked ? Theme.success : Theme.surfaceMuted
        border.color: control.checked ? Theme.success : Theme.border
        border.width: control.activeFocus ? Theme.borderWidthStrong : Theme.borderWidth

        Rectangle {
            width: 24 * AppMetrics.scale
            height: width
            radius: width / 2
            y: (parent.height - height) / 2
            x: control.checked ? parent.width - width - 4 * AppMetrics.scale : 4 * AppMetrics.scale
            color: control.enabled ? "#FFFFFF" : Theme.disabledText
            Behavior on x { NumberAnimation { duration: Performance.shortDuration; easing.type: Easing.OutCubic } }
        }
    }
}

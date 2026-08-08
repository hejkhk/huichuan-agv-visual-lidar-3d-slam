import QtQuick
import QtQuick.Controls
import ".."

Slider {
    id: control
    implicitHeight: AppMetrics.touch

    background: Rectangle {
        x: control.leftPadding
        y: control.topPadding + control.availableHeight / 2 - height / 2
        width: control.availableWidth
        height: 8 * AppMetrics.scale
        radius: height / 2
        color: Theme.surfaceMuted
        border.color: Theme.border
        border.width: Theme.borderWidth
        Rectangle {
            width: control.visualPosition * parent.width
            height: parent.height
            radius: parent.radius
            color: control.enabled ? Theme.primary : Theme.disabledText
        }
    }

    handle: Rectangle {
        x: control.leftPadding + control.visualPosition * (control.availableWidth - width)
        y: control.topPadding + control.availableHeight / 2 - height / 2
        width: 26 * AppMetrics.scale
        height: width
        radius: width / 2
        color: Theme.surfaceElevated
        border.color: control.enabled ? Theme.primary : Theme.disabledText
        border.width: Theme.borderWidthStrong
    }
}

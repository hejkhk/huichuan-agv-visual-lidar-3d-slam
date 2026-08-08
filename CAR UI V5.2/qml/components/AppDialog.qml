import QtQuick
import QtQuick.Controls
import ".."

Popup {
    id: root
    modal: true
    dim: true
    padding: AppMetrics.cardPadding
    closePolicy: Popup.CloseOnEscape
    enter: Transition {
        ParallelAnimation {
            NumberAnimation { property: "opacity"; from: Performance.lowPower ? 1 : 0; to: 1; duration: Performance.shortDuration }
            NumberAnimation { property: "scale"; from: Performance.smooth ? 0.92 : 1; to: 1; duration: Performance.shortDuration; easing.type: Easing.OutBack }
        }
    }
    exit: Transition {
        ParallelAnimation {
            NumberAnimation { property: "opacity"; from: 1; to: Performance.lowPower ? 1 : 0; duration: Performance.shortDuration }
            NumberAnimation { property: "scale"; from: 1; to: Performance.smooth ? 0.96 : 1; duration: Performance.shortDuration }
        }
    }
    Overlay.modal: Rectangle { color: Theme.overlay }
    background: Rectangle {
        color: Theme.surfaceElevated
        radius: Theme.radius * AppMetrics.scale
        border.color: Theme.border
        border.width: Theme.borderWidth
    }
}

import QtQuick
import QtQuick.Controls
import ".."

/*
 * Window-level input dialog.
 *
 * Qt Quick Controls Popup/Dialog items live in Overlay.overlay.  On some
 * Qt/Wayland combinations that overlay is composited above InputPanel and
 * hides the on-screen keyboard.  This component deliberately stays in the
 * normal window scene so the application-wide InputPanel can remain above it.
 *
 * The public open()/close()/visible API mirrors Dialog, keeping page call
 * sites and business actions unchanged.
 */
Item {
    id: root

    default property alias contentData: contentHost.data
    property real dialogWidth: 720 * AppMetrics.scale
    property real dialogHeight: 320 * AppMetrics.scale
    property real padding: AppMetrics.cardPadding

    signal opened()
    signal closed()

    anchors.fill: parent
    visible: false
    z: 1000

    function open() {
        if (visible)
            return
        window.inputDialogActive = true
        window.keyboardRequested = false
        visible = true
        opened()
        // Focus after the newly visible scene item has completed a polish
        // pass; otherwise the input method can miss the first focus request.
        Qt.callLater(function() { root.opened() })
    }

    function close() {
        if (!visible)
            return
        window.dismissKeyboard()
        window.inputDialogActive = false
        visible = false
        closed()
    }

    Rectangle {
        anchors.fill: parent
        color: Theme.overlay

        TapHandler {
            // Consume taps outside the card; input dialogs close explicitly.
        }
    }

    Rectangle {
        id: card
        width: Math.min(root.dialogWidth, root.width - 2 * AppMetrics.margin)
        height: Math.min(root.dialogHeight,
                         root.height - window.inputPanelHeight - 2 * AppMetrics.margin)
        x: (root.width - width) / 2
        y: Math.max(AppMetrics.margin,
                    (root.height - window.inputPanelHeight - height) / 2)
        radius: Theme.radius
        color: Theme.surface
        border.width: Theme.borderWidth
        border.color: Theme.border

        Item {
            id: contentHost
            anchors.fill: parent
            anchors.margins: root.padding
        }
    }
}

import QtQuick
import ".."

AppButton {
    id: root
    objectName: "keyboardToggleButton"

    required property var inputItem

    // Use the requested state, not InputPanel.visible. Pressing this button
    // moves focus away from the TextField before onClicked runs, which makes
    // InputPanel.visible briefly false and would invert the action.
    text: I18n.t(window.keyboardRequested ? "隐藏键盘" : "显示键盘")
    implicitWidth: 124 * AppMetrics.scale
    outlined: !window.keyboardRequested
    accent: Theme.primary

    function toggleKeyboard() {
        if (window.keyboardRequested)
            window.dismissKeyboard()
        else
            window.showKeyboardFor(inputItem)
    }

    onClicked: toggleKeyboard()
}

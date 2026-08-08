import QtQuick
import QtQuick.VirtualKeyboard
import ".."

InputPanel {
    id: root
    objectName: "applicationVirtualKeyboard"
    property bool allowed: false
    width: parent ? parent.width : 0
    x: 0
    y: parent ? parent.height - height : 0
    z: 2000
    visible: allowed && active

    // The built-in Qt style exposes palette properties. Binding them to the
    // application Theme keeps all four palettes and their light/dark variants
    // visually consistent without replacing the proven Pinyin input engine.
    Binding {
        target: root.keyboard.style
        property: "keyboardBackgroundColor"
        value: Theme.darkMode ? Theme.surfaceMuted : Theme.pageBackground
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "normalKeyBackgroundColor"
        // The built-in function-key SVGs are white, so light themes use the
        // palette's restrained secondary tone instead of near-black or white.
        value: Theme.darkMode ? Theme.surfaceElevated : Theme.textSecondary
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "highlightedKeyBackgroundColor"
        value: Theme.primary
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "keyTextColor"
        value: Theme.darkMode ? Theme.textPrimary : "#FFFFFF"
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "keySmallTextColor"
        value: Theme.darkMode ? Theme.textSecondary : "#FFFFFF"
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "selectionListBackgroundColor"
        value: Theme.surfaceElevated
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "selectionListTextColor"
        value: Theme.textPrimary
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "selectionListSeparatorColor"
        value: Theme.divider
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "secondaryColor"
        value: Theme.primary
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "secondaryLightColor"
        value: Theme.primaryHover
        when: root.keyboard.style
    }
    Binding {
        target: root.keyboard.style
        property: "secondaryDarkColor"
        value: Theme.primaryPressed
        when: root.keyboard.style
    }
}

pragma Singleton
import QtQuick

QtObject {
    property real scale: 1
    property int fontSizeMode: 1
    readonly property real fontScale: [0.8, 1.0, 1.2][
        Math.max(0, Math.min(2, fontSizeMode))
    ]
    readonly property bool compact: scale < 0.82
    readonly property real unit: 8 * scale
    readonly property real gap: 14 * scale
    readonly property real sectionGap: 18 * scale
    readonly property real margin: 20 * scale
    readonly property real cardPadding: 18 * scale
    // Large displays retain the 48 px touch floor. On 1024x600 panels the
    // whole HMI is physically smaller, so using the same floor causes text,
    // buttons and cards to overlap. Compact mode scales the complete design
    // system together while keeping controls comfortably tappable.
    readonly property real touch: compact
        ? Math.max(34, 50 * scale) : Math.max(48, 50 * scale)
    readonly property real primaryTouch: compact
        ? Math.max(38, 58 * scale) : Math.max(56, 58 * scale)
    // The production HMI is designed and verified at 1920×1080. Font
    // hierarchy is independent from geometry so operators can enlarge text
    // without turning every card and touch target into a different layout.
    readonly property real body: 16 * fontScale
    readonly property real small: 14 * fontScale
    readonly property real caption: 13 * fontScale
    readonly property real cardTitle: 18 * fontScale
    readonly property real sectionTitle: 20 * fontScale
    readonly property real title: 26 * fontScale
    readonly property real statusHeight: compact
        ? Math.max(54, 82 * scale) : Math.max(78, 82 * scale)
}

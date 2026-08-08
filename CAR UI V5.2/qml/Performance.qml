pragma Singleton
import QtQuick

QtObject {
    id: root

    // 0: low-power, 1: normal (the approved current behavior), 2: smooth.
    property int mode: 1

    readonly property bool lowPower: mode === 0
    readonly property bool normal: mode === 1
    readonly property bool smooth: mode === 2
    readonly property bool decorativeAnimations: !lowPower
    readonly property bool enhancedAnimations: smooth

    readonly property int instantDuration: lowPower ? 0 : (smooth ? 140 : 90)
    readonly property int shortDuration: lowPower ? 0 : (smooth ? 190 : 110)
    readonly property int mediumDuration: lowPower ? 0 : (smooth ? 380 : 300)
    readonly property int pageDuration: lowPower ? 0 : (smooth ? 360 : 250)
    readonly property int pulseDuration: smooth ? 760 : 1000
    readonly property int mapPaintInterval: lowPower ? 450 : (smooth ? 100 : 200)
    readonly property real imageScale: lowPower ? 0.65 : (smooth ? 1.25 : 1.0)
    readonly property bool imageCache: smooth
    readonly property int backendPollInterval: lowPower ? 1500 : (smooth ? 400 : 750)
}

pragma Singleton
import QtQuick

QtObject {
    id: root

    // 0 Industrial blue, 1 Graphite teal, 2 Deep-space violet,
    // 3 Titanium amber. Every palette has a light and dark variant.
    property int colorScheme: 0
    property bool darkMode: false
    property int borderMode: 0
    readonly property int borderWidth: [1, 2, 4][
        Math.max(0, Math.min(2, borderMode))
    ]
    readonly property int borderWidthStrong: borderWidth * 2
    readonly property int borderWidthHeavy: borderWidth * 4

    readonly property var lightPalettes: [
        {
            pageBackground: "#EEF3F7", surface: "#FFFFFF",
            surfaceElevated: "#FFFFFF", surfaceMuted: "#F5F8FA",
            border: "#D9E3EA", divider: "#E7EDF2",
            textPrimary: "#162936", textSecondary: "#526673",
            textMuted: "#7A8C98", primary: "#2378D4",
            primaryHover: "#3186E2", primaryPressed: "#185FAE",
            primarySoft: "#EAF3FD", mapToolbarBackground: "#20313C",
            mapCanvas: "#DCE6EB", bottomBarBackground: "#FFFFFF"
        },
        {
            pageBackground: "#EDF4F3", surface: "#FFFFFF",
            surfaceElevated: "#FFFFFF", surfaceMuted: "#F2F7F6",
            border: "#D4E2DF", divider: "#E2ECEA",
            textPrimary: "#17302E", textSecondary: "#526B68",
            textMuted: "#78908D", primary: "#16847A",
            primaryHover: "#23998E", primaryPressed: "#106960",
            primarySoft: "#E2F4F1", mapToolbarBackground: "#1D3533",
            mapCanvas: "#D8E6E3", bottomBarBackground: "#FFFFFF"
        },
        {
            pageBackground: "#F2F0F7", surface: "#FFFFFF",
            surfaceElevated: "#FFFFFF", surfaceMuted: "#F7F5FA",
            border: "#E1DCEC", divider: "#ECE8F2",
            textPrimary: "#2B2538", textSecondary: "#625A72",
            textMuted: "#8A8298", primary: "#7057C7",
            primaryHover: "#8268D7", primaryPressed: "#5841A7",
            primarySoft: "#F0ECFB", mapToolbarBackground: "#302A40",
            mapCanvas: "#E3DEEC", bottomBarBackground: "#FFFFFF"
        },
        {
            pageBackground: "#F5F2EC", surface: "#FFFFFF",
            surfaceElevated: "#FFFFFF", surfaceMuted: "#FAF7F2",
            border: "#E7DDCF", divider: "#EFE7DC",
            textPrimary: "#332A20", textSecondary: "#6E6254",
            textMuted: "#958979", primary: "#B96D17",
            primaryHover: "#CC7B20", primaryPressed: "#945511",
            primarySoft: "#FAECD8", mapToolbarBackground: "#3A3024",
            mapCanvas: "#E9E0D3", bottomBarBackground: "#FFFFFF"
        }
    ]

    readonly property var darkPalettes: [
        {
            pageBackground: "#07131D", surface: "#0C1D29",
            surfaceElevated: "#102633", surfaceMuted: "#0A1924",
            border: "#244354", divider: "#1B3545",
            textPrimary: "#F2F7FA", textSecondary: "#B9C8D1",
            textMuted: "#7E94A1", primary: "#2F8CFF",
            primaryHover: "#4A9CFF", primaryPressed: "#1B6BC4",
            primarySoft: "#123B5F", mapToolbarBackground: "#091923",
            mapCanvas: "#102430", bottomBarBackground: "#091722"
        },
        {
            pageBackground: "#061514", surface: "#0B2422",
            surfaceElevated: "#102D2A", surfaceMuted: "#081D1B",
            border: "#28514B", divider: "#1D403C",
            textPrimary: "#EFFAF8", textSecondary: "#B5D0CC",
            textMuted: "#789B96", primary: "#2FC4B3",
            primaryHover: "#47D2C2", primaryPressed: "#1C9588",
            primarySoft: "#123F3A", mapToolbarBackground: "#071C1A",
            mapCanvas: "#0F2A27", bottomBarBackground: "#071B19"
        },
        {
            pageBackground: "#100D1B", surface: "#1A1628",
            surfaceElevated: "#211B32", surfaceMuted: "#151120",
            border: "#463A61", divider: "#362C4D",
            textPrimary: "#F7F3FF", textSecondary: "#CEC4E0",
            textMuted: "#978BAB", primary: "#9B7CF5",
            primaryHover: "#AE92FF", primaryPressed: "#7657D1",
            primarySoft: "#35275E", mapToolbarBackground: "#151020",
            mapCanvas: "#211A31", bottomBarBackground: "#15111F"
        },
        {
            pageBackground: "#17120C", surface: "#251D13",
            surfaceElevated: "#302518", surfaceMuted: "#1E170F",
            border: "#59432B", divider: "#453522",
            textPrimary: "#FFF8EE", textSecondary: "#D7C5AE",
            textMuted: "#A28D73", primary: "#E59A3A",
            primaryHover: "#F2AC50", primaryPressed: "#B87424",
            primarySoft: "#563819", mapToolbarBackground: "#1C150D",
            mapCanvas: "#2A2116", bottomBarBackground: "#1B150E"
        }
    ]

    readonly property var palette: {
        var palettes = darkMode ? darkPalettes : lightPalettes
        return palettes[Math.max(0, Math.min(palettes.length - 1, colorScheme))]
    }

    readonly property color pageBackground: palette.pageBackground
    readonly property color surface: palette.surface
    readonly property color surfaceElevated: palette.surfaceElevated
    readonly property color surfaceMuted: palette.surfaceMuted
    readonly property color border: palette.border
    readonly property color divider: palette.divider
    readonly property color textPrimary: palette.textPrimary
    readonly property color textSecondary: palette.textSecondary
    readonly property color textMuted: palette.textMuted
    readonly property color primary: palette.primary
    readonly property color primaryHover: palette.primaryHover
    readonly property color primaryPressed: palette.primaryPressed
    readonly property color primarySoft: palette.primarySoft
    readonly property color mapToolbarBackground: palette.mapToolbarBackground
    readonly property color mapCanvas: palette.mapCanvas
    readonly property color bottomBarBackground: palette.bottomBarBackground

    // Semantic colors stay stable across palettes.
    readonly property color success: darkMode ? "#3DC66D" : "#279E52"
    readonly property color successSoft: darkMode ? "#123C2A" : "#E7F6EC"
    readonly property color warning: darkMode ? "#F2A23A" : "#D98215"
    readonly property color warningSoft: darkMode ? "#493117" : "#FFF3DF"
    readonly property color danger: darkMode ? "#F05A67" : "#D84956"
    readonly property color dangerSoft: darkMode ? "#4A2028" : "#FCECEE"
    readonly property color info: darkMode ? "#48A8F8" : "#287CC4"
    readonly property color purple: darkMode ? "#A77BFF" : "#7554C9"
    readonly property color purpleSoft: darkMode ? "#332653" : "#F1EDFB"
    readonly property color disabledBackground: darkMode ? "#273945" : "#DDE5EA"
    readonly property color disabledText: darkMode ? "#71838E" : "#95A3AD"
    readonly property color shadowColor: darkMode ? "#00000000" : "#180F2533"
    readonly property color overlay: darkMode ? "#A6000000" : "#660D1B24"
    readonly property int radius: 14
    readonly property int radiusSmall: 9

    // Compatibility aliases used by older shared components.
    readonly property color background: pageBackground
    readonly property color surfaceAlt: surfaceMuted
    readonly property color text: textPrimary
    readonly property color muted: textMuted
    readonly property color disabled: disabledBackground
}

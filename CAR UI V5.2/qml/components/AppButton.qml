import QtQuick
import QtQuick.Controls
import ".."

Button {
    id: control
    property color accent: Theme.primary
    property bool outlined: false
    property bool busy: false
    property bool compact: false

    implicitHeight: compact ? 40 * AppMetrics.scale : AppMetrics.touch
    implicitWidth: compact ? 92 * AppMetrics.scale : 116 * AppMetrics.scale
    font.pixelSize: AppMetrics.body
    font.weight: Font.DemiBold
    hoverEnabled: true
    focusPolicy: Qt.StrongFocus

    background: Rectangle {
        radius: Theme.radiusSmall * AppMetrics.scale
        color: {
            if (!control.enabled) return Theme.disabledBackground
            if (control.outlined) {
                if (control.down) return Qt.rgba(control.accent.r, control.accent.g, control.accent.b, 0.16)
                if (control.hovered) return Qt.rgba(control.accent.r, control.accent.g, control.accent.b, 0.09)
                return "transparent"
            }
            if (control.down) return Qt.darker(control.accent, 1.18)
            if (control.hovered) return Qt.lighter(control.accent, 1.08)
            return control.accent
        }
        border.color: !control.enabled ? Theme.disabledBackground : control.activeFocus ? Theme.info : control.outlined ? control.accent : "transparent"
        border.width: control.activeFocus
            ? Theme.borderWidthStrong : control.outlined ? Theme.borderWidth : 0
        Behavior on color { ColorAnimation { duration: Performance.shortDuration } }
    }

    contentItem: Text {
        text: control.busy ? I18n.t("处理中") : control.text
        color: !control.enabled ? Theme.disabledText : control.outlined ? control.accent : "white"
        font: control.font
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }
}

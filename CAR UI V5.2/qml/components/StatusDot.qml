import QtQuick
import ".."

Rectangle {
    property string status: "NORMAL"
    width: 10 * AppMetrics.scale
    height: width
    radius: width / 2
    color: status === "NORMAL" ? Theme.success
        : status === "WARNING" ? Theme.warning
        : status === "ERROR" ? Theme.danger
        : status === "DISCONNECTED" ? Theme.disabledText
        : Theme.textMuted
}

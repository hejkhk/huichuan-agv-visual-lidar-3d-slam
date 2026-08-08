import QtQuick
import ".."

AppButton {
    implicitWidth: AppMetrics.touch
    property string accessibleName: text
    Accessible.name: accessibleName
    compact: true
}

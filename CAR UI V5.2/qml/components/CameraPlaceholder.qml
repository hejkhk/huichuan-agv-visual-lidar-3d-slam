import QtQuick
import ".."

Rectangle {
    id: root
    property string selectedActor: ""
    function friendlyActorName(actorId) {
        var raw = String(actorId || "")
        var match = raw.match(/Actor\s*(\d+)/i)
        return match ? I18n.t("人员") + match[1] : (raw || I18n.t("尚未选择"))
    }
    signal actorSelected(string actorId)
    color: Theme.mapToolbarBackground; radius: Theme.radius; clip: true
    Text { anchors.centerIn: parent; text: I18n.t("摄像头检测画面 · Mock"); color: Theme.textMuted; font.pixelSize: AppMetrics.body }
    Repeater { model: backend.snapshot.detected_actors ?? []
        Rectangle { required property var modelData; x: modelData.x*root.width; y: modelData.y*root.height; width: 94*AppMetrics.scale; height: 160*AppMetrics.scale; color: "transparent"; border.width: root.selectedActor===modelData.id ? 5 : 2; border.color: root.selectedActor===modelData.id ? Theme.success : Theme.primary; radius: 8
            Text { anchors.bottom: parent.top; text: root.friendlyActorName(modelData.id) + " · " + modelData.distance + "m"; color: Theme.textPrimary; font.pixelSize: AppMetrics.small }
            MouseArea { anchors.fill: parent; onClicked: root.actorSelected(modelData.id) }
        }
    }
}

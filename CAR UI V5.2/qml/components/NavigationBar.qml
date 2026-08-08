import QtQuick
import QtQuick.Layouts
import ".."

RowLayout {
    signal openSettings(); signal openStatus(); signal openFollow(); signal openVoice()
    spacing: AppMetrics.gap
    LauncherTile { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; text: I18n.t("设置"); iconSource: "../../assets/icons/settings.svg"; accent: "#8c929a"; onClicked: openSettings() }
    LauncherTile { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; text: I18n.t("车体状态"); iconSource: "../../assets/icons/status.svg"; accent: "#1597cf"; onClicked: openStatus() }
    LauncherTile { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; text: I18n.t("人员跟随"); iconSource: "../../assets/icons/camera.svg"; accent: "#60717c"; onClicked: openFollow() }
    LauncherTile { Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 1; text: I18n.t("语音控制"); iconSource: "../../assets/icons/microphone.svg"; accent: "#bd197a"; onClicked: openVoice() }
}

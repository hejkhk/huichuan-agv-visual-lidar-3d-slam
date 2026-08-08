import QtQuick
import QtQuick.Layouts
import ".."

RowLayout {
    id: root
    signal previous()
    signal next()
    signal skip()
    signal restart()

    property int currentIndex: 0
    property int count: 1
    readonly property bool lastStep: currentIndex >= count - 1

    spacing: AppMetrics.gap

    SecondaryButton {
        text: I18n.t("跳过教程")
        onClicked: root.skip()
    }
    SecondaryButton {
        visible: root.lastStep
        text: I18n.t("重新观看")
        onClicked: root.restart()
    }
    Item { Layout.fillWidth: true }
    Text {
        text: (root.currentIndex + 1) + " / " + root.count
        color: Theme.textSecondary
        font.pixelSize: AppMetrics.body
        font.bold: true
    }
    Item { Layout.fillWidth: true }
    SecondaryButton {
        text: I18n.t("上一步")
        enabled: root.currentIndex > 0
        onClicked: root.previous()
    }
    PrimaryButton {
        text: I18n.t(root.lastStep ? "完成并返回" : "下一步")
        onClicked: root.next()
    }
}

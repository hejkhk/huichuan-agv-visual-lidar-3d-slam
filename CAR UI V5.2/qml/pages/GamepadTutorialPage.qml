import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."
import "../components"

Page {
    id: root
    objectName: "gamepadTutorialPage"
    property bool reopenMappingOnReturn: false
    property int stepIndex: 0
    property int stepTick: 0
    property bool introActive: true
    property bool manualNavigation: false
    readonly property var step: tutorialModel.steps[stepIndex]
    readonly property bool lastStep:
        stepIndex >= tutorialModel.steps.length - 1

    background: Rectangle { color: Theme.pageBackground }

    function finishTutorial() {
        stepTimer.stop()
        autoAdvanceTimer.stop()
        window.returnFromGamepadTutorial(reopenMappingOnReturn)
    }
    function cancelAutomaticPaging() {
        manualNavigation = true
        autoAdvanceTimer.stop()
    }
    function startStepAnimation() {
        stepTimer.stop()
        autoAdvanceTimer.stop()
        stepTick = 0
        if (!introActive && Number(step.animationTicks || 0) > 0)
            stepTimer.start()
    }
    function advance(manual) {
        if (manual)
            cancelAutomaticPaging()
        if (lastStep) {
            if (manual)
                finishTutorial()
            return
        }
        stepTimer.stop()
        autoAdvanceTimer.stop()
        stepIndex += 1
    }
    function goPrevious() {
        cancelAutomaticPaging()
        if (stepIndex > 0) {
            stepTimer.stop()
            stepIndex -= 1
        }
    }

    Component.onCompleted: introTimer.start()
    onStepIndexChanged: Qt.callLater(startStepAnimation)

    GamepadTutorialModel { id: tutorialModel }

    Timer {
        id: introTimer
        interval: Performance.lowPower ? 250 : 1250
        onTriggered: {
            root.introActive = false
            root.startStepAnimation()
        }
    }
    Timer {
        id: stepTimer
        interval: Performance.lowPower ? 700
            : (Performance.smooth ? 780 : 900)
        repeat: true
        onTriggered: {
            root.stepTick += 1
            if (root.stepTick >= Number(root.step.animationTicks || 0)) {
                stop()
                if (!root.manualNavigation && !root.lastStep)
                    autoAdvanceTimer.restart()
            }
        }
    }
    Timer {
        id: autoAdvanceTimer
        interval: 1000
        repeat: false
        onTriggered: root.advance(false)
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: AppMetrics.margin
        spacing: AppMetrics.gap

        PageHeader {
            Layout.fillWidth: true
            title: I18n.t("手柄控制教程")
            subtitle: root.introActive
                ? I18n.t("认识车辆控制手柄")
                : I18n.t("按步骤了解安全操作方法")
            StatusBadge {
                status: root.manualNavigation ? "UNKNOWN" : "NORMAL"
                label: I18n.t(root.manualNavigation
                    ? "手动翻页" : "自动演示")
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: AppMetrics.gap

            AppCard {
                Layout.preferredWidth: root.width * 0.55
                Layout.fillHeight: true
                clip: true

                GamepadFocusView {
                    anchors.fill: parent
                    anchors.margins: AppMetrics.gap
                    step: root.step
                    intro: root.introActive
                    phase: root.stepTick
                }
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: AppMetrics.gap
                opacity: root.introActive ? 0 : 1
                Behavior on opacity {
                    NumberAnimation {
                        duration: Performance.lowPower ? 0
                            : (Performance.smooth ? 760 : 600)
                    }
                }

                Text {
                    Layout.fillWidth: true
                    text: I18n.t(root.step.title)
                    color: Theme.textPrimary
                    font.pixelSize: AppMetrics.title * 1.22
                    font.bold: true
                    wrapMode: Text.WordWrap
                }
                Text {
                    Layout.fillWidth: true
                    text: I18n.t(root.step.description)
                    color: Theme.textSecondary
                    font.pixelSize: AppMetrics.compact
                        ? AppMetrics.body : AppMetrics.sectionTitle
                    lineHeight: 1.25
                    wrapMode: Text.WordWrap
                }
                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: warningText.implicitHeight
                        + AppMetrics.cardPadding * 2
                    radius: Theme.radiusSmall
                    color: root.step.id === "estop"
                        ? Theme.dangerSoft : Theme.warningSoft
                    Text {
                        id: warningText
                        anchors.fill: parent
                        anchors.margins: AppMetrics.cardPadding
                        text: I18n.t(root.step.warning)
                        color: root.step.id === "estop"
                            ? Theme.danger : Theme.warning
                        font.pixelSize: AppMetrics.compact
                            ? AppMetrics.small : AppMetrics.body
                        font.bold: true
                        wrapMode: Text.WordWrap
                    }
                }
                VehicleDemo {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    demoType: root.step.demo
                    phase: root.stepTick
                }
            }
        }

        TutorialNavigationBar {
            Layout.fillWidth: true
            currentIndex: root.stepIndex
            count: tutorialModel.steps.length
            onPrevious: root.goPrevious()
            onNext: root.advance(true)
            onSkip: {
                root.cancelAutomaticPaging()
                root.finishTutorial()
            }
            onRestart: {
                root.cancelAutomaticPaging()
                root.stepIndex = 0
                root.stepTick = 0
                root.introActive = true
                introTimer.restart()
            }
        }
    }
}

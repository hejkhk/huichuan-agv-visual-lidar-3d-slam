import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ".."

Item {
    id: root
    objectName: "homeTutorialOverlay"
    required property var homeItem
    property int stepIndex: 0
    property bool manualNavigation: false
    property bool transitioning: false
    property int pendingStepIndex: 0
    property int transitionDirection: 1
    property bool pendingFinish: false
    readonly property var step: tutorialModel.steps[stepIndex]
    readonly property bool lastStep: stepIndex >= tutorialModel.steps.length - 1
    readonly property var targetItem: homeItem && homeItem.tutorialTarget
        ? homeItem.tutorialTarget(step.target) : null
    readonly property point targetPosition: targetItem
        ? targetItem.mapToItem(root, 0, 0) : Qt.point(0, 0)
    signal finished()

    function stopAutomaticPaging() {
        manualNavigation = true
        stepDurationTimer.stop()
        autoAdvanceTimer.stop()
    }

    function restartStep() {
        stepDurationTimer.stop()
        autoAdvanceTimer.stop()
        Qt.callLater(function() {
            focusSource.scheduleUpdate()
            tutorialDemo.restart()
            if (!Performance.lowPower)
                focusIntro.restart()
            if (!manualNavigation && Performance.lowPower)
                stepDurationTimer.restart()
        })
    }

    function transitionTo(nextIndex, direction, finishAfter) {
        if (transitioning)
            return
        stepDurationTimer.stop()
        autoAdvanceTimer.stop()
        pendingStepIndex = nextIndex
        transitionDirection = direction
        pendingFinish = finishAfter
        if (Performance.lowPower) {
            if (finishAfter)
                finished()
            else
                stepIndex = nextIndex
            return
        }
        pageTransition.restart()
    }

    function nextStep(manual) {
        if (manual)
            stopAutomaticPaging()
        transitionTo(lastStep ? stepIndex : stepIndex + 1, 1, lastStep)
    }

    function previousStep() {
        stopAutomaticPaging()
        if (stepIndex > 0)
            transitionTo(stepIndex - 1, -1, false)
    }

    Component.onCompleted: restartStep()
    onStepIndexChanged: restartStep()

    HomeTutorialModel { id: tutorialModel }

    Timer {
        id: stepDurationTimer
        interval: Number(root.step.duration || 3800)
        repeat: false
        onTriggered: if (!root.manualNavigation)
            autoAdvanceTimer.restart()
    }
    Timer {
        id: autoAdvanceTimer
        interval: 500
        repeat: false
        onTriggered: root.nextStep(false)
    }

    SequentialAnimation {
        id: pageTransition
        ScriptAction { script: root.transitioning = true }
        ParallelAnimation {
            NumberAnimation {
                target: pageTranslate; property: "x"
                to: -root.transitionDirection * tutorialPageContent.width
                    * (Performance.smooth ? 0.16 : 0.08)
                duration: Performance.smooth ? 260 : 170
                easing.type: Easing.InCubic
            }
            NumberAnimation {
                target: tutorialPageContent; property: "opacity"
                to: 0; duration: Performance.smooth ? 230 : 150
            }
            NumberAnimation {
                target: tutorialPageContent; property: "scale"
                to: Performance.smooth ? 0.92 : 0.97
                duration: Performance.smooth ? 260 : 170
                easing.type: Easing.InCubic
            }
            NumberAnimation {
                target: tutorialPageContent; property: "rotation"
                to: -root.transitionDirection * (Performance.smooth ? 1.5 : 0.5)
                duration: Performance.smooth ? 260 : 170
            }
        }
        ScriptAction {
            script: {
                if (root.pendingFinish) {
                    root.finished()
                } else {
                    root.stepIndex = root.pendingStepIndex
                    pageTranslate.x = root.transitionDirection
                        * tutorialPageContent.width
                        * (Performance.smooth ? 0.18 : 0.09)
                    tutorialPageContent.scale = Performance.smooth ? 0.90 : 0.97
                    tutorialPageContent.rotation = root.transitionDirection
                        * (Performance.smooth ? 1.8 : 0.5)
                    tutorialPageContent.opacity = 0
                }
            }
        }
        PauseAnimation { duration: Performance.smooth ? 35 : 12 }
        ParallelAnimation {
            NumberAnimation {
                target: pageTranslate; property: "x"
                to: 0; duration: Performance.smooth ? 390 : 210
                easing.type: Easing.OutCubic
            }
            NumberAnimation {
                target: tutorialPageContent; property: "opacity"
                to: 1; duration: Performance.smooth ? 300 : 190
                easing.type: Easing.OutCubic
            }
            NumberAnimation {
                target: tutorialPageContent; property: "scale"
                to: 1; duration: Performance.smooth ? 430 : 220
                easing.type: Performance.smooth ? Easing.OutBack : Easing.OutCubic
            }
            NumberAnimation {
                target: tutorialPageContent; property: "rotation"
                to: 0; duration: Performance.smooth ? 390 : 210
                easing.type: Easing.OutCubic
            }
        }
        ScriptAction { script: root.transitioning = false }
    }

    Rectangle {
        anchors.fill: parent
        color: Performance.lowPower ? "#E80A1720" : "#B80A1720"
        opacity: Performance.smooth ? 0.72 : 0.82
    }

    // The actual home control remains visible as a location cue. The guide
    // never forwards pointer events to it.
    Rectangle {
        visible: root.targetItem !== null
        x: Math.max(6, root.targetPosition.x - 8 * AppMetrics.scale)
        y: Math.max(6, root.targetPosition.y - 8 * AppMetrics.scale)
        width: Math.min(root.width - x - 6,
                        (root.targetItem ? root.targetItem.width : 0)
                        + 16 * AppMetrics.scale)
        height: Math.min(root.height - y - 6,
                         (root.targetItem ? root.targetItem.height : 0)
                         + 16 * AppMetrics.scale)
        radius: Theme.radius
        color: "transparent"
        border.color: Theme.primary
        border.width: Performance.smooth
            ? Theme.borderWidthStrong * 2 : Theme.borderWidthStrong
        opacity: 0.65 + (Performance.smooth
            ? 0.25 * Math.sin(focusPulseState.progress * Math.PI * 2) : 0)
    }

    QtObject {
        id: focusPulseState
        property real progress: 0
    }
    NumberAnimation {
        id: focusPulse
        target: focusPulseState
        property: "progress"
        from: 0
        to: 1
        loops: Animation.Infinite
        duration: 1100
        running: Performance.smooth && root.visible
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 30 * AppMetrics.scale
        spacing: AppMetrics.gap

        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.gap
            Text {
                Layout.fillWidth: true
                text: I18n.t("首页快速教程")
                color: "white"
                font.pixelSize: AppMetrics.title
                font.bold: true
            }
            StatusBadge {
                status: root.manualNavigation ? "UNKNOWN" : "NORMAL"
                label: I18n.t(root.manualNavigation ? "手动翻页" : "自动演示")
            }
            SecondaryButton {
                text: I18n.t("退出教程")
                onClicked: root.finished()
            }
        }

        RowLayout {
            id: tutorialPageContent
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: AppMetrics.sectionGap
            transform: Translate { id: pageTranslate; x: 0 }

            AppCard {
                Layout.preferredWidth: root.width * 0.56
                Layout.fillHeight: true
                color: Theme.surfaceElevated
                clip: true

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: AppMetrics.cardPadding
                    spacing: AppMetrics.gap

                    RowLayout {
                        Layout.fillWidth: true
                        Text {
                            Layout.fillWidth: true
                            text: I18n.t("当前介绍区域")
                            color: Theme.textSecondary
                            font.pixelSize: AppMetrics.body
                            font.weight: Font.DemiBold
                        }
                        StatusBadge { status: "INFO"; label: I18n.t(root.step.objectLabel) }
                    }

                    Item {
                        id: previewArea
                        Layout.fillWidth: true
                        Layout.fillHeight: true

                        Rectangle {
                            anchors.fill: parent
                            radius: Theme.radius
                            color: Theme.pageBackground
                            border.color: Theme.border
                            border.width: Theme.borderWidth
                        }

                        Item {
                            id: focusPreview
                            anchors.centerIn: parent
                            readonly property real sourceAspect: root.targetItem
                                && root.targetItem.height > 0
                                ? root.targetItem.width / root.targetItem.height : 1.6
                            width: Math.min(previewArea.width * 0.90,
                                            previewArea.height * 0.78 * sourceAspect)
                            height: Math.max(28 * AppMetrics.scale,
                                             width / sourceAspect)
                            scale: 1

                            NumberAnimation {
                                id: focusIntro
                                target: focusPreview
                                property: "scale"
                                from: Performance.smooth ? 0.92 : 0.97
                                to: 1
                                duration: Performance.smooth ? 520 : 230
                                easing.type: Performance.smooth
                                    ? Easing.OutBack : Easing.OutCubic
                            }

                            ShaderEffectSource {
                                id: focusSource
                                anchors.fill: parent
                                sourceItem: root.targetItem
                                sourceRect: root.targetItem
                                    ? Qt.rect(0, 0, root.targetItem.width, root.targetItem.height)
                                    : Qt.rect(0, 0, 1, 1)
                                textureSize: Qt.size(
                                    Math.min(1024, Math.max(1, width * 1.5)),
                                    Math.min(768, Math.max(1, height * 1.5)))
                                live: Performance.smooth
                                recursive: true
                                smooth: true
                                hideSource: false
                            }

                            Rectangle {
                                anchors.fill: parent
                                anchors.margins: -8 * AppMetrics.scale
                                radius: Theme.radius
                                color: "transparent"
                                border.color: Theme.primary
                                border.width: Theme.borderWidthStrong * 2
                            }
                            Rectangle {
                                visible: Performance.smooth
                                anchors.fill: parent
                                anchors.margins: -18 * AppMetrics.scale
                                radius: Theme.radius
                                color: "transparent"
                                border.color: Theme.primarySoft
                                border.width: 8 * AppMetrics.scale
                                opacity: 0.32
                            }
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        text: I18n.t("蓝色高光表示当前讲解的操作区域。")
                        color: Theme.textMuted
                        font.pixelSize: AppMetrics.caption
                        horizontalAlignment: Text.AlignHCenter
                    }
                }
            }

            AppCard {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: Theme.surfaceElevated

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: AppMetrics.cardPadding
                    spacing: AppMetrics.gap

                    HomeTutorialDemo {
                        id: tutorialDemo
                        Layout.fillWidth: true
                        Layout.preferredHeight: parent.height * 0.46
                        demoType: root.step.demo
                        active: root.visible
                        loop: root.manualNavigation
                        onCycleFinished: {
                            if (!root.manualNavigation && !root.transitioning)
                                autoAdvanceTimer.restart()
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        text: I18n.t(root.step.title)
                        color: Theme.textPrimary
                        font.pixelSize: AppMetrics.title * 1.08
                        font.bold: true
                        wrapMode: Text.WordWrap
                    }
                    Text {
                        Layout.fillWidth: true
                        text: I18n.t(root.step.description)
                        color: Theme.textSecondary
                        font.pixelSize: AppMetrics.sectionTitle
                        lineHeight: 1.2
                        wrapMode: Text.WordWrap
                    }
                    Rectangle { Layout.fillWidth: true; height: 1; color: Theme.divider }
                    Repeater {
                        model: root.step.actions
                        RowLayout {
                            required property var modelData
                            required property int index
                            Layout.fillWidth: true
                            spacing: AppMetrics.gap
                            Rectangle {
                                width: 30 * AppMetrics.scale
                                height: width
                                radius: width / 2
                                color: Theme.primarySoft
                                Text { anchors.centerIn: parent; text: index + 1; color: Theme.primary; font.pixelSize: AppMetrics.small; font.bold: true }
                            }
                            Text {
                                Layout.fillWidth: true
                                text: I18n.t(modelData)
                                color: Theme.textPrimary
                                font.pixelSize: AppMetrics.body
                                wrapMode: Text.WordWrap
                            }
                        }
                    }
                    Rectangle {
                        Layout.fillWidth: true
                        implicitHeight: noteText.implicitHeight + AppMetrics.gap * 2
                        radius: Theme.radiusSmall
                        color: Theme.warningSoft
                        Text {
                            id: noteText
                            anchors.fill: parent
                            anchors.margins: AppMetrics.gap
                            text: I18n.t(root.step.note)
                            color: Theme.warning
                            font.pixelSize: AppMetrics.small
                            font.weight: Font.DemiBold
                            wrapMode: Text.WordWrap
                        }
                    }
                    Item { Layout.fillHeight: true }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: AppMetrics.gap
            SecondaryButton {
                text: I18n.t("退出教程")
                onClicked: root.finished()
            }
            Item { Layout.fillWidth: true }
            Text {
                text: (root.stepIndex + 1) + " / " + tutorialModel.steps.length
                color: "white"
                font.pixelSize: AppMetrics.body
                font.bold: true
            }
            Item { Layout.fillWidth: true }
            SecondaryButton {
                text: I18n.t("上一步")
                enabled: root.stepIndex > 0 && !root.transitioning
                onClicked: root.previousStep()
            }
            PrimaryButton {
                text: I18n.t(root.lastStep ? "完成教程" : "下一步")
                enabled: !root.transitioning
                onClicked: root.nextStep(true)
            }
        }
    }

    TapHandler {
        // Consume all pointer input. The guide is visual only and cannot
        // activate the real home controls behind it.
    }
}

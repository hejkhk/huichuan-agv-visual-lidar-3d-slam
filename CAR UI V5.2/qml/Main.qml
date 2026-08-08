import QtQuick
import QtQuick.Controls
import QtQuick.Effects
import QtQuick.Layouts
import QtQuick.VirtualKeyboard.Settings
import QtCore
import "."
import "components"
import "pages"

ApplicationWindow {
    id: window
    visible: true
    width: 1920; height: 1080
    minimumWidth: 1280; minimumHeight: 720
    title: I18n.t("机器人车载触控屏")
    color: Theme.pageBackground
    property bool delayedBusy: false
    property bool _fullscreenDone: false
    property bool inputDialogActive: false
    property bool keyboardRequested: false
    readonly property bool keyboardVisible: appKeyboard.visible
    readonly property real inputPanelHeight: appKeyboard.visible
        ? appKeyboard.height : 0
    property int activePerformanceMode: Performance.mode
    property bool startupVisible: true
    readonly property bool tutorialActive: homeTutorialLoader.active
    readonly property Item tutorialStatusBar: statusBar

    Settings {
        id: uiPreferences
        category: "appearance"
        property bool darkMode: false
        property int colorScheme: 0
        property int fontSizeMode: 1
        property int borderMode: 0
    }


    Timer {
        id: fullscreenTimer
        interval: 200
        repeat: false
        onTriggered: {
            if (!window._fullscreenDone) {
                window._fullscreenDone = true
                window.showFullScreen()
            }
        }
    }

    Component.onCompleted: {
        AppMetrics.scale = 1
        Theme.darkMode = uiPreferences.darkMode
        Theme.colorScheme = Math.max(
            0, Math.min(3, Number(uiPreferences.colorScheme))
        )
        AppMetrics.fontSizeMode = Math.max(
            0, Math.min(2, Number(uiPreferences.fontSizeMode))
        )
        Theme.borderMode = Math.max(
            0, Math.min(2, Number(uiPreferences.borderMode))
        )
        setPerformanceMode(backend.settings.performance_mode ?? 1)
        VirtualKeyboardSettings.activeLocales = ["zh_CN", "en_US"]
        VirtualKeyboardSettings.locale = backend.language === "zh" ? "zh_CN" : "en_US"
        if (typeof autoFullscreen !== "undefined" && autoFullscreen)
            fullscreenTimer.start()
    }

    function pushPage(source) {
        stack.push("pages/" + source)
    }
    function pushPageWithProperties(source, properties) {
        stack.push("pages/" + source, properties)
    }
    function returnFromGamepadTutorial(reopenMapping) {
        goBack()
        if (reopenMapping) {
            Qt.callLater(function() {
                var item = stack.currentItem
                if (item && item.openMappingDialog)
                    item.openMappingDialog()
            })
        }
    }
    function setDarkMode(enabled) {
        Theme.darkMode = enabled
        uiPreferences.darkMode = enabled
    }
    function setColorScheme(index) {
        Theme.colorScheme = Math.max(0, Math.min(3, index))
        uiPreferences.colorScheme = Theme.colorScheme
    }
    function setFontSizeMode(index) {
        AppMetrics.fontSizeMode = Math.max(0, Math.min(2, Number(index)))
        uiPreferences.fontSizeMode = AppMetrics.fontSizeMode
    }
    function setBorderMode(index) {
        Theme.borderMode = Math.max(0, Math.min(2, Number(index)))
        uiPreferences.borderMode = Theme.borderMode
    }
    function setPerformanceMode(index) {
        var nextMode = Math.max(0, Math.min(2, Number(index)))
        Performance.mode = nextMode
        backend.setPerformanceMode(nextMode)
    }
    property int activeColorScheme: Theme.colorScheme
    property bool activeDarkMode: Theme.darkMode
    property int activeFontSizeMode: AppMetrics.fontSizeMode
    property int activeBorderMode: Theme.borderMode
    function setCurrentPagePanel(index) {
        if (stack.currentItem && stack.currentItem.activePanel !== undefined)
            stack.currentItem.activePanel = index
    }
    function startHomeTutorial() {
        homeTutorialPrompt.close()
        goHome()
        Qt.callLater(function() {
            homeTutorialLoader.active = false
            homeTutorialLoader.active = true
        })
    }
    function offerHomeTutorialAfterStartup() {
        if (backend.settings.show_home_tutorial_on_startup ?? false)
            homeTutorialPrompt.open()
    }
    function closeTransient() {
        if (homeTutorialLoader.active) {
            homeTutorialLoader.active = false
            return true
        }
        var item = stack.currentItem
        return item && item.closeTransient ? item.closeTransient() : false
    }
    function dismissKeyboard() {
        keyboardRequested = false
        keyboardFocusSink.forceActiveFocus()
        Qt.inputMethod.hide()
        Qt.callLater(function() { Qt.inputMethod.hide() })
    }
    function showKeyboardFor(inputItem) {
        keyboardRequested = true
        if (inputItem)
            inputItem.forceActiveFocus()
        Qt.inputMethod.show()
        Qt.callLater(function() {
            if (inputItem)
                inputItem.forceActiveFocus()
            Qt.inputMethod.show()
        })
    }
    function goBack() {
        if (closeTransient()) {
            dismissKeyboard()
            return
        }
        dismissKeyboard()
        if (stack.depth > 1)
            stack.pop()
    }
    function goHome() {
        closeTransient()
        dismissKeyboard()
        while (stack.depth > 1)
            stack.pop()
    }
    function toggleApplicationFullscreen() {
        if (window.visibility === Window.FullScreen)
            window.showNormal()
        else
            window.showFullScreen()
    }

    Item {
        id: applicationSurface
        anchors.fill: parent
        layer.enabled: window.tutorialActive && Performance.smooth
        layer.smooth: true
        layer.effect: MultiEffect {
            blurEnabled: true
            blur: 0.62
            blurMax: 32
            saturation: -0.18
        }

        StackView {
            id: stack
            objectName: "pageStack"
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: statusBar.top
            initialItem: HomePage {}

            pushEnter: Transition {
            ParallelAnimation {
                NumberAnimation {
                    property: "x"
                    from: Performance.smooth ? window.width * 0.08 : window.width * 0.035
                    to: 0
                    duration: Performance.pageDuration
                    easing.type: Easing.OutCubic
                }
                NumberAnimation {
                    property: "opacity"
                    from: Performance.lowPower ? 1 : 0.55
                    to: 1
                    duration: Performance.pageDuration
                    easing.type: Easing.OutCubic
                }
                NumberAnimation {
                    property: "scale"
                    from: Performance.smooth ? 0.975 : 1
                    to: 1
                    duration: Performance.pageDuration
                    easing.type: Easing.OutCubic
                }
            }
        }
            pushExit: Transition {
            NumberAnimation {
                property: "opacity"
                from: 1
                to: Performance.smooth ? 0.78 : 1
                duration: Performance.pageDuration
            }
        }
            popEnter: Transition {
            NumberAnimation {
                property: "opacity"
                from: Performance.lowPower ? 1 : 0.72
                to: 1
                duration: Performance.pageDuration
                easing.type: Easing.OutCubic
            }
        }
            popExit: Transition {
            ParallelAnimation {
                NumberAnimation {
                    property: "x"
                    from: 0
                    to: Performance.smooth ? window.width * 0.08 : window.width * 0.035
                    duration: Performance.pageDuration
                    easing.type: Easing.InCubic
                }
                NumberAnimation {
                    property: "opacity"
                    from: 1
                    to: Performance.lowPower ? 1 : 0
                    duration: Performance.pageDuration
                    easing.type: Easing.InCubic
                }
            }
            }
        }

        BottomStatusBar {
            id: statusBar
            objectName: "statusBar"
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            height: AppMetrics.statusHeight
            fullScreen: window.visibility === Window.FullScreen
            onBackRequested: window.goBack()
            onHomeRequested: window.goHome()
            onFullscreenRequested: window.toggleApplicationFullscreen()
        }
    }

    Timer {
        id: busyDelay
        interval: 220
        onTriggered: window.delayedBusy = backend.busy
    }
    Connections {
        target: backend
        function onBusyChanged() {
            if (backend.busy)
                busyDelay.restart()
            else {
                busyDelay.stop()
                window.delayedBusy = false
            }
        }
    }
    Rectangle {
        objectName: "busyOverlay"
        visible: window.delayedBusy
        anchors.fill: parent
        color: Theme.overlay
        z: 50
        BusyIndicator { anchors.centerIn: parent; running: parent.visible }
    }
    StartupSplash {
        id: startupSplash
        objectName: "startupSplash"
        anchors.fill: parent
        z: 1000
        visible: window.startupVisible
        darkMode: uiPreferences.darkMode
        onFinished: {
            window.startupVisible = false
            Qt.callLater(window.offerHomeTutorialAfterStartup)
        }
    }

    AppDialog {
        id: homeTutorialPrompt
        objectName: "homeTutorialPrompt"
        width: Math.min(620 * AppMetrics.scale, window.width * 0.72)
        x: (window.width - width) / 2
        y: (window.height - height) / 2
        z: 950
        contentItem: ColumnLayout {
            spacing: AppMetrics.gap
            Text {
                Layout.fillWidth: true
                text: I18n.t("首页快速教程")
                color: Theme.textPrimary
                font.pixelSize: AppMetrics.title
                font.bold: true
            }
            Text {
                Layout.fillWidth: true
                text: I18n.t("是否查看首页快速教程？")
                color: Theme.textPrimary
                font.pixelSize: AppMetrics.sectionTitle
                font.weight: Font.DemiBold
                wrapMode: Text.WordWrap
            }
            Text {
                Layout.fillWidth: true
                text: I18n.t("教程将介绍地图、车辆状态和四种控制方式。")
                color: Theme.textSecondary
                font.pixelSize: AppMetrics.body
                wrapMode: Text.WordWrap
            }
            RowLayout {
                Layout.fillWidth: true
                Item { Layout.fillWidth: true }
                SecondaryButton {
                    text: I18n.t("暂不查看")
                    onClicked: homeTutorialPrompt.close()
                }
                PrimaryButton {
                    text: I18n.t("开始教程")
                    onClicked: window.startHomeTutorial()
                }
            }
        }
    }

    Loader {
        id: homeTutorialLoader
        objectName: "homeTutorialLoader"
        anchors.fill: parent
        active: false
        visible: active
        z: 900
        sourceComponent: Component {
            HomeTutorialOverlay {
                homeItem: stack.depth > 0 ? stack.get(0) : null
                onFinished: homeTutorialLoader.active = false
            }
        }
    }
    Item {
        id: keyboardFocusSink
        objectName: "keyboardFocusSink"
        width: 1
        height: 1
        visible: true
    }
    Popup {
        id: toast
        x: (window.width - width) / 2
        y: window.height - height - AppMetrics.statusHeight - AppMetrics.margin
        closePolicy: Popup.NoAutoClose
        padding: AppMetrics.gap
        z: 100
        enter: Transition {
            ParallelAnimation {
                NumberAnimation { property: "opacity"; from: Performance.lowPower ? 1 : 0; to: 1; duration: Performance.shortDuration }
                NumberAnimation { property: "scale"; from: Performance.smooth ? 0.9 : 1; to: 1; duration: Performance.shortDuration; easing.type: Easing.OutBack }
            }
        }
        exit: Transition {
            NumberAnimation { property: "opacity"; from: 1; to: Performance.lowPower ? 1 : 0; duration: Performance.shortDuration }
        }
        background: Rectangle {
            color: Theme.surfaceElevated
            radius: Theme.radiusSmall
            border.color: Theme.border
            border.width: Theme.borderWidth
        }
        contentItem: Text {
            text: I18n.t(backend.notification)
            color: Theme.textPrimary
            font.pixelSize: AppMetrics.body
        }
        Timer {
            id: toastTimer
            interval: 2800
            onTriggered: {
                toast.close()
                backend.clearNotification()
            }
        }
    }


    VirtualKeyboard {
        id: appKeyboard
        parent: window.contentItem
        width: window.width
        allowed: window.keyboardRequested
    }
    Connections {
        target: backend
        function onNotificationChanged() {
            if (backend.notification) {
                toast.open()
                toastTimer.restart()
            }
        }
    }
    Binding { target: I18n; property: "language"; value: backend.language }
}

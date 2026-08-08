import QtQuick
import ".."

Item {
    id: root
    property var maps: []
    property alias currentIndex: view.currentIndex
    readonly property var currentMap: maps.length > 0
        && view.currentIndex >= 0
        && view.currentIndex < maps.length
        ? maps[view.currentIndex] : ({})
    readonly property bool animating: transitionLock.running || view.moving
    signal selectionChanged(var mapData)

    function circularDistance(index, current, count) {
        if (count <= 0)
            return 0
        var direct = Math.abs(index - current)
        return Math.min(direct, count - direct)
    }

    function selectIndex(index) {
        if (animating || index < 0 || index >= maps.length
                || index === view.currentIndex)
            return false
        transitionLock.restart()
        view.currentIndex = index
        return true
    }

    function previous() {
        if (maps.length <= 1 || view.currentIndex < 0)
            return false
        return selectIndex((view.currentIndex + maps.length - 1) % maps.length)
    }

    function next() {
        if (maps.length <= 1 || view.currentIndex < 0)
            return false
        return selectIndex((view.currentIndex + 1) % maps.length)
    }

    onMapsChanged: {
        if (maps.length === 0)
            view.currentIndex = -1
        else if (view.currentIndex < 0)
            view.currentIndex = 0
        else if (view.currentIndex >= maps.length)
            view.currentIndex = maps.length - 1
    }

    Timer {
        id: transitionLock
        interval: Performance.lowPower ? 40 : Performance.mediumDuration + 20
        repeat: false
    }

    PathView {
        id: view
        anchors.fill: parent
        model: root.maps
        interactive: false
        pathItemCount: Math.min(3, root.maps.length)
        cacheItemCount: 0
        preferredHighlightBegin: 0.5
        preferredHighlightEnd: 0.5
        highlightRangeMode: PathView.StrictlyEnforceRange
        highlightMoveDuration: Performance.mediumDuration
        snapMode: PathView.SnapToItem

        delegate: MapCard {
            required property var modelData
            required property int index
            width: root.width * 0.46
            height: root.height * 0.92
            mapData: modelData
            emphasized: PathView.isCurrentItem
            visible: root.circularDistance(
                index, view.currentIndex, root.maps.length
            ) <= 1
            scale: PathView.cardScale
            opacity: PathView.cardOpacity
            rotation: Performance.smooth ? PathView.cardTilt : 0
            z: PathView.isCurrentItem ? 2 : 1
            Behavior on scale {
                NumberAnimation { duration: Performance.mediumDuration; easing.type: Easing.OutCubic }
            }
            Behavior on rotation {
                NumberAnimation { duration: Performance.mediumDuration; easing.type: Easing.OutCubic }
            }
            Behavior on opacity {
                NumberAnimation { duration: Performance.mediumDuration; easing.type: Easing.OutCubic }
            }
        }

        path: Path {
            startX: root.width * 0.16
            startY: root.height * 0.52
            PathAttribute { name: "cardScale"; value: 0.72 }
            PathAttribute { name: "cardOpacity"; value: 0.48 }
            PathAttribute { name: "cardTilt"; value: -1.8 }
            PathLine {
                x: root.width * 0.5
                y: root.height * 0.5
            }
            PathAttribute { name: "cardScale"; value: 1.0 }
            PathAttribute { name: "cardOpacity"; value: 1.0 }
            PathAttribute { name: "cardTilt"; value: 0 }
            PathLine {
                x: root.width * 0.84
                y: root.height * 0.52
            }
            PathAttribute { name: "cardScale"; value: 0.72 }
            PathAttribute { name: "cardOpacity"; value: 0.48 }
            PathAttribute { name: "cardTilt"; value: 1.8 }
        }

        onCurrentIndexChanged: {
            if (currentIndex >= 0 && currentIndex < root.maps.length)
                root.selectionChanged(root.maps[currentIndex])
        }
    }
}

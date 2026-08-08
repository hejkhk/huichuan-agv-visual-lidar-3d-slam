pragma ComponentBehavior: Bound
import QtQuick

QtObject {
    readonly property var steps: [
        {
            id: "map", target: "map", demo: "map", duration: 4200,
            objectLabel: "地图",
            title: "查看地图",
            description: "地图用于查看车辆位置和选择临时目的地。",
            actions: [
                "点击地图中的可通行区域设置目的地。",
                "确认箭头位于目的地，并指向车辆到达后的车头方向。"
            ],
            note: "目标箭头同时表示到达位置和最终车头方向。"
        },
        {
            id: "map_tools", target: "map_tools", demo: "map_tools", duration: 3800,
            objectLabel: "地图工具",
            title: "使用地图工具",
            description: "地图工具用于管理地图和调整显示范围。",
            actions: [
                "使用放大、缩小和回到全图调整视野。",
                "选择目的地后，使用编辑目标朝向调整车头方向。",
                "使用地图管理切换地图，或进入全屏操作。"
            ],
            note: "创建新地图前，请先确认车辆处于安全状态。"
        },
        {
            id: "travel_status", target: "travel_status", demo: "travel_status", duration: 3600,
            objectLabel: "行驶状态",
            title: "查看行驶状态",
            description: "行驶状态用于显示当前任务和完成进度。",
            actions: [
                "任务执行时可以查看进度和当前提示。",
                "需要时可以暂停、继续或结束导航。"
            ],
            note: "结束导航后，车辆将停止执行当前导航任务。"
        },
        {
            id: "vehicle", target: "vehicle", demo: "vehicle", duration: 4200,
            objectLabel: "车辆状态",
            title: "查看车辆状态",
            description: "车辆状态集中显示设备、电量、运动和连接情况。",
            actions: [
                "绿色表示正常，橙色表示需要注意。",
                "发现异常时，打开详细状态查看原因。"
            ],
            note: "重要警告应在继续操作前处理。"
        },
        {
            id: "navigation", target: "navigation", demo: "navigation", duration: 4400,
            objectLabel: "导航控制",
            title: "使用导航控制",
            description: "导航控制用于让车辆自动前往指定位置。",
            actions: [
                "先选择目的地，再启动导航。",
                "也可以返回充电座、保存位置或管理目的地。"
            ],
            note: "启动前请确认车辆周围没有障碍物。"
        },
        {
            id: "voice", target: "voice", demo: "voice", duration: 4200,
            objectLabel: "语音控制",
            title: "使用语音控制",
            description: "语音控制用于通过语音向车辆发出指令。",
            actions: [
                "确认语音功能已经开启。",
                "在正在听取指令时清楚说出操作内容。"
            ],
            note: "允许陌生人控制后，未登记人员也可以发出指令。"
        },
        {
            id: "gamepad", target: "gamepad", demo: "gamepad", duration: 4000,
            objectLabel: "手柄控制",
            title: "使用手柄控制",
            description: "手柄控制用于将车辆控制权交给无线手柄。",
            actions: [
                "点击交还控制权，等待状态变为手柄接管。",
                "交接完成后，请使用手柄控制车辆。"
            ],
            note: "不熟悉按键时，请先查看手柄教程。"
        },
        {
            id: "follow", target: "follow", demo: "follow", duration: 4400,
            objectLabel: "视觉控制",
            title: "使用视觉控制",
            description: "视觉控制用于识别人员并保持设定距离跟随。",
            actions: [
                "开启功能并确认需要跟随的人员。",
                "设置跟随距离后，再开始跟随。"
            ],
            note: "目标丢失时，请重新选择人员或停止跟随。"
        },
        {
            id: "status_bar", target: "status_bar", demo: "status_bar", duration: 3800,
            objectLabel: "底部状态栏",
            title: "使用底部状态栏",
            description: "底部状态栏用于查看连接情况和切换页面。",
            actions: [
                "返回用于回到上一页，主页用于返回首页。",
                "右侧可以查看时间和网络连接状态。"
            ],
            note: "首页按钮可以随时返回主操作界面。"
        }
    ]
}

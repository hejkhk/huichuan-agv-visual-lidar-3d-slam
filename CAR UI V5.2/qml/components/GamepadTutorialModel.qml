import QtQuick

QtObject {
    readonly property var steps: [
        {
            id: "overview", title: "认识手柄",
            description: "这是车辆的无线控制手柄。使用时请将手柄横向拿正。",
            warning: "主要区域包括十字键、两个摇杆、档位切换键和急停键。",
            focusX: 0.50, focusY: 0.50,
            x: 0.06, y: 0.10, w: 0.82, h: 0.78,
            scale: 1.0, demo: "overview", animationTicks: 5
        },
        {
            id: "dpad", title: "十字键控制",
            description: "所有6个档位都可以使用十字键。上键前进，下键后退，左键原地左转，右键原地右转。",
            warning: "十字键不受当前档位限制。",
            focusX: 0.22, focusY: 0.40,
            x: 0.14, y: 0.28, w: 0.18, h: 0.24,
            scale: 2.05, demo: "dpad", animationTicks: 10
        },
        {
            id: "estop", title: "急停按钮",
            description: "按下红色圆形急停按钮后，车辆会立即强制刹车。",
            warning: "急停触发后无法通过按键或软件恢复，只有给小车断电并重新上电才能解除。",
            focusX: 0.73, focusY: 0.39,
            x: 0.69, y: 0.33, w: 0.09, h: 0.12,
            scale: 2.15, demo: "estop", animationTicks: 10
        },
        {
            id: "gear", title: "档位切换",
            description: "绿色三角形按钮用于依次切换车辆的6个控制档位。",
            warning: "绿、蓝、黄、红为四个普通档位；白色常亮和白色闪烁为两个建图档位。",
            focusX: 0.68, focusY: 0.30,
            x: 0.64, y: 0.24, w: 0.09, h: 0.13,
            scale: 2.15, demo: "gear", animationTicks: 12
        },
        {
            id: "normal", title: "普通行驶档位",
            description: "绿色、蓝色、黄色、红色对应四个普通档位。四档都能使用十字键，也可以使用右侧摇杆连续控制车辆。",
            warning: "普通档位使用十字键或右侧摇杆，左侧摇杆不控制车辆。",
            focusX: 0.39, focusY: 0.50,
            x: 0.13, y: 0.27, w: 0.52, h: 0.42,
            scale: 2.05, demo: "normal", animationTicks: 14
        },
        {
            id: "mapping", title: "建图档位",
            description: "建图一档为白色常亮，建图二档为白色闪烁；两档都使用左侧摇杆，二档行驶速度更快。",
            warning: "建图档位请使用左侧摇杆，右侧摇杆无效。",
            focusX: 0.51, focusY: 0.47,
            x: 0.28, y: 0.24, w: 0.46, h: 0.46,
            scale: 2.05, demo: "mapping", animationTicks: 14
        },
        {
            id: "complete", title: "手柄操作教程完成",
            description: "所有档位都能使用十字键；普通四档还可使用右侧摇杆；两个建图档位使用左侧摇杆。",
            warning: "红色圆形按钮是必须断电重启才能恢复的急停按钮，绿色三角形按钮用于切换6个档位。",
            focusX: 0.50, focusY: 0.50,
            x: 0.06, y: 0.10, w: 0.82, h: 0.78,
            scale: 1.0, demo: "complete", animationTicks: 0
        }
    ]
}

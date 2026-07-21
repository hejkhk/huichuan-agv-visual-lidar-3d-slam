# `/robot/web_control` 控制协议

消息类型为 `std_msgs/String`，`data` 是 JSON 字符串：

```json
{
  "source": "web_console",
  "command": "forward",
  "gear": 1,
  "led_color": "green",
  "speed_cnt_per_sec": 15000000,
  "multiplier": 1,
  "timestamp_ms": 1710000000000
}
```

字段含义：`gear` 为 1～4；`speed_cnt_per_sec` 是给下位机的有效计数速度；`timestamp_ms` 是浏览器 Unix 毫秒时间。`stop` 与 `emergency_stop` 的速度固定为 `0`。

## 命令示例

以下示例省略格式差异，字段均完整。

### forward

```json
{"source":"web_console","command":"forward","gear":1,"led_color":"green","speed_cnt_per_sec":15000000,"multiplier":1,"timestamp_ms":1710000000000}
```

### backward

```json
{"source":"web_console","command":"backward","gear":2,"led_color":"blue","speed_cnt_per_sec":30000000,"multiplier":2,"timestamp_ms":1710000000000}
```

### turn_left

```json
{"source":"web_console","command":"turn_left","gear":3,"led_color":"yellow","speed_cnt_per_sec":75000000,"multiplier":5,"timestamp_ms":1710000000000}
```

### turn_right

```json
{"source":"web_console","command":"turn_right","gear":4,"led_color":"red","speed_cnt_per_sec":450000000,"multiplier":30,"timestamp_ms":1710000000000}
```

### stop

```json
{"source":"web_console","command":"stop","gear":1,"led_color":"green","speed_cnt_per_sec":0,"multiplier":1,"timestamp_ms":1710000000000}
```

### emergency_stop

```json
{"source":"web_console","command":"emergency_stop","gear":1,"led_color":"green","speed_cnt_per_sec":0,"multiplier":1,"timestamp_ms":1710000000000}
```

### reset_estop

```json
{"source":"web_console","command":"reset_estop","gear":1,"led_color":"green","speed_cnt_per_sec":15000000,"multiplier":1,"timestamp_ms":1710000000000}
```

### gear_change

```json
{"source":"web_console","command":"gear_change","gear":2,"led_color":"blue","speed_cnt_per_sec":30000000,"multiplier":2,"timestamp_ms":1710000000000}
```

允许的 `command` 仅为：`forward`、`backward`、`turn_left`、`turn_right`、`stop`、`emergency_stop`、`reset_estop`、`gear_change`。

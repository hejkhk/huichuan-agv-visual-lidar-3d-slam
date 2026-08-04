# STEP2C4.1.1 参数类型修复

## 本次错误

ROS 2 Jazzy 报错：

```text
parameter 'Odom/GuessMotion' has invalid type:
parameter is of type string, setting it to bool is not allowed
```

RTAB-Map 的核心参数（带 `/` 的参数，例如 `Odom/GuessMotion`）在
`rtabmap_ros` 中注册为字符串。它的内容虽然是 `true/false`，ROS 参数类型仍必须是
`string`，不能传入 ROS `bool`。

## 修复

- `Odom/GuessMotion` 改为显式字符串参数；
- 保留启动参数 `odom_guess_motion:=false`，但通过 `ParameterValue(..., value_type=str)` 传入；
- 测试脚本新增 RTAB-Map 节点存活检查，避免节点崩溃后仅凭残留话题名称误报“就绪”。

STEP1～STEP10V2.1 均未修改。

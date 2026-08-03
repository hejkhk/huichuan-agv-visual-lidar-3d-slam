# STEP2C4.1：根据首次干净链实测进行的最小修正

本次 `runtime.log` 的关键事实：

- Gemini2 实际发布的是 Color 640×480@15Hz、D2C 后 Depth 640×480@15Hz；
- RGB/Depth 时间差警告为 0，说明 15/15Hz + frame sync 已生效；
- RTAB-Map 共输出 184 帧，日志时长约 16.45 秒，平均输出约 11.13Hz；
- update time 中位数约 46.10ms，P95 约 143.77ms；
- delay 中位数约 221.71ms，P95 约 323.20ms；
- `delay-update` 中位数约 175.71ms，说明主要固定延迟发生在 RTAB-Map 开始计算之前；
- 出现 19 次“带运动猜测注册失败”，19 次均在去掉 guess 后成功。

因此 STEP2C4.1 只做三个有证据支持的修改：

1. `Odom/GuessMotion=false`，避免错误常速度猜测导致同一帧计算两遍；
2. `topic_queue_size=1`、`sync_queue_size=3`，降低旧帧排队机会；
3. 删除已经弃用的 `queue_size` 参数，并增强日志报告，增加输出频率、帧间隔和 `delay-update` 统计。

STEP1～STEP10V2.1均未修改。

# 起步预倾角修改

要求4/5/6启动流程现在为：

1. ARMING：识别 A 点并等待平衡板基础 READY ACK。
2. PRETILT：车轮保持停止，通过 UART1 bit4 发送预计起步加速度。
3. 至少预倾 220 ms；之后收到 ACK bit5 PRETILT_READY 即进入 RUNNING。
4. 若视觉闭环使目标持续微动，达到 `CAR_BALANCE_PRETILT_MAX_MS` 后，只要基础 ACK 仍有效，也会启动。

可在 `Config/CarConfig.h` 调整：

- `CAR_BALANCE_PRETILT_MIN_MS`
- `CAR_BALANCE_PRETILT_MAX_MS`
- `MOTION_BALANCE_ACCEL_MM_S2`

本工程必须与本次同步修改的平衡板工程配套烧录。

默认参数为最短 220 ms、最长 450 ms。预倾时间会计入按下启动键后的比赛总时间，因此不要盲目增大。

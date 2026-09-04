# 本次修改

## PB4 默认水平回位

PB4 按下后会停止要求3、要求4/5/6、手动电机测试和远程任务，驱动丝杆回到 `BalanceLevelConfig.h` 中的 `BALANCE_LEVEL_DEFAULT_RAW_DEG`，到位稳定后自动调用水平置零，不需要再按 PB5。

串口命令：

- `motorstatus` 或 `mstatus`：查询闭环、电机、编码器、默认水平位置，并输出可直接复制的 `DEFAULT_COPY` 宏。
- `levelgo` 或 `home`：执行与 PB4 相同的默认水平回位。
- `leveldefault=<raw_deg>`：仅修改本次上电会话中的默认值。
- `levelsave`：把当前位置保存为本次会话默认并立即认定为水平。

默认值标定后，把 `DEFAULT_COPY` 输出复制到 `BalanceLevelConfig.h` 并重新编译。由于当前多圈位置以 MCU 复位时的位置为原点，每次上电必须从同一已知机械位置开始。

## 起步预倾角

UART1 协议 bit4 已改为 PRETILT。小车进入 PRETILT 时车轮保持停止，但提前发送 `MOTION_BALANCE_ACCEL_MM_S2`。平衡板提前施加前馈倾角，并在丝杆到达目标后通过 ACK bit5 返回 PRETILT_READY。

小车端默认至少等待 220 ms，最长等待 450 ms；两块控制板必须使用本次配套版本。

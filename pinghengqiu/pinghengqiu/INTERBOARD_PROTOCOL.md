# 小车板与平衡球板通信协议（含起步预倾）

## 物理连接

- 小车板 UART1_TX PB6 -> 平衡球板 UART1_RX PB7
- 平衡球板 UART1_TX PB6 -> 小车板 UART1_RX PB7
- 两板 GND 共地，115200 8N1。

## 小车 -> 平衡球板：9 字节

`A5 5A seq flags planned_cps_L planned_cps_H accel_L accel_H checksum`

flags：bit0 RUNNING，bit1 EMERGENCY，bit2 READY，bit3 BALANCE_MODE，
bit4 PRETILT，bit5 REQUIREMENT6。

PRETILT 阶段小车保持停车，发送 READY+PRETILT，并在 acceleration 字段发送
`MOTION_BALANCE_ACCEL_MM_S2`。平衡球板提前建立加速度前馈倾角。

## 平衡球板 -> 小车：7 字节 ACK

`5A A5 seq status mission fault checksum`

status：bit0 LEVEL_READY，bit1 CAMERA_VALID，bit2 CONTROL_ACTIVE，
bit3 TARGET_LATCHED，bit4 FAULT，bit5 PRETILT_READY。

PRETILT_READY 只有在外部前馈有效且丝杆实际位置进入目标约 70 电机度窗口后置位。
小车至少等待 220 ms，并收到 PRETILT_READY 后才进入 RUNNING。

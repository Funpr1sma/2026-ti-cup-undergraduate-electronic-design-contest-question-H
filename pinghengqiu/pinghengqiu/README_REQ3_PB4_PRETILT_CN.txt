本版修改说明

一、要求3
- +5 cm 段参数保持不变。
- -5 cm 巡航倾角 0.22 -> 0.28 deg。
- 距离 -5 cm 仍超过 1.2 cm、速度低于 0.38 cm/s 连续 140 ms 时，
  临时使用 0.42 deg 防卡滞；速度恢复到 0.90 cm/s 后回到 0.28 deg。

二、默认水平高度
- motorstate / mstate / levelstatus：查看电机、编码器、目标和默认水平状态。
- leveldefault=<raw_deg>：设置本次上电的默认水平 raw 角度。
- levelsave：把当前真实水平位置保存为本次上电默认值并清零。
- levelgo：回默认位置，到位后自动认为水平。
- PB4：等同 levelgo。
- BalanceLevelConfig.h 可写入编译时默认值。增量编码器 raw 角在复位后重新参考，
  因此跨断电使用默认值要求每次上电机械初始位置一致。

推荐设置：
1. 手动调到真实水平。
2. 发送 motorstate，记录 raw。
3. 发送 levelsave；同次上电后 PB4 可随时恢复。
4. 需要固化时，把记录值写入 BALANCE_LEVEL_DEFAULT_RAW_DEG。

三、小车起步预倾
- UART 协议 bit4 恢复为 PRETILT。
- 默认 carffgain=0.85、carfflimit=1.20 deg，220 mm/s^2 对应约 -1 deg 预倾。
- 只有实际丝杆接近目标后才回 PRETILT_READY，避免单纯固定延时。

# TI 杯 H 题：车载平衡滚球运动控制系统

本项目是 2026 年全国大学生电子设计竞赛（TI 杯）H 题作品，目标是设计一辆沿黑色环形轨迹行驶的循线小车，并在行驶过程中通过摆杆控制机构使钢球保持在中心或指定位置附近。

## 项目概览

系统由两块 TI MSPM0G3507 控制板和一块 MaixCAM2 视觉模块组成：

```text
MaixCAM2
  ├─ YOLOv5 钢球检测
  ├─ Alpha-Beta 位置/速度跟踪
  └─ UART 发送视觉状态
           ↓
平衡球控制板（MSPM0G3507）
  ├─ 球位置闭环控制
  ├─ 丝杆执行器位置控制
  ├─ 起步预倾与加速度前馈
  └─ 向循迹板返回 ACK/就绪状态
           ↕ UART
循迹小车控制板（MSPM0G3507）
  ├─ 8 路红外循迹
  ├─ 双轮速度 PI 控制
  ├─ 加减速运动规划
  └─ 任务状态机与停车控制
```

平衡机构一端通过合页固定，另一端由电机驱动丝杆调节摆杆角度。钢球位置由摄像头测量，控制板根据球的位置和速度计算摆杆目标角度，再转换为丝杆位移和电机目标角度。

## 目录结构

### `lineCAR/`

循线小车控制板工程，主要功能包括：

- 8 路红外光电传感器采集和黑线位置加权计算；
- 直线/弯道识别与循迹 PD 控制；
- 左右车轮编码器反馈和独立速度 PI 控制；
- 加速度受限的速度规划；
- A 点启停、终点标志识别和停车状态机；
- 通过 UART 向平衡球板发送运行状态、规划速度和加速度。

主要入口和模块：

- `lineCAR/lineCAR/lineCAR.c`：系统初始化和主循环；
- `lineCAR/lineCAR/Hardware/LineFollow.c`：循迹控制；
- `lineCAR/lineCAR/Hardware/CarControl.c`：任务状态机；
- `lineCAR/lineCAR/Hardware/SpeedPI.c`：车轮速度闭环；
- `lineCAR/lineCAR/MotionPlanner.c`：速度/加速度规划；
- `lineCAR/lineCAR/CarMotionLink.c`：双板通信发送端。

### `pinghengqiu/`

平衡球控制板工程，主要功能包括：

- 接收 MaixCAM2 发送的钢球位置和速度；
- 20 ms 周期的球位置闭环控制；
- 电机编码器位置反馈和丝杆位置控制；
- 静摩擦破除、方向不对称补偿和卡滞恢复；
- 速度预测、提前制动、制动卸载和精调稳定；
- 起步预倾控制，补偿小车启动时的惯性扰动；
- 向循迹板返回摄像头有效、控制激活、目标锁存和预倾完成状态。

建议优先阅读：

- `pinghengqiu/pinghengqiu/empty.c`：系统入口、定时任务和主循环；
- `pinghengqiu/pinghengqiu/ball_balance_control.c`：平衡控制核心；
- `pinghengqiu/pinghengqiu/motor.c`：丝杆电机驱动；
- `pinghengqiu/pinghengqiu/encoder.c`：编码器采集；
- `pinghengqiu/pinghengqiu/CarMotionRx.c`：循迹板通信接收、前馈和 ACK；
- `pinghengqiu/pinghengqiu/ball_link.c`：摄像头数据帧解析；
- `pinghengqiu/pinghengqiu/INTERBOARD_PROTOCOL.md`：双板通信协议。

### `H2026_MaixCAM2_small/`

MaixCAM2 视觉工程，位于 `H2026_MaixCAM2_small/H2026_MaixCAM2_YOLO244_ROI_UART60/`，主要功能包括：

- 使用 YOLOv5 检测摆杆中的钢球；
- 对图像 ROI 进行裁剪，减少背景干扰和推理开销；
- 根据目标类别、尺寸、长宽比、纵向位置和连续运动约束筛选目标；
- 使用 Alpha-Beta 跟踪器估计钢球位置和速度；
- 通过 UART4 发送兼容旧协议的 12 字节帧和扩展 20 字节帧；
- 通过 RTSP 输出实时画面，用于现场观察、录像和回放。

MaixCAM2 程序入口：

```text
H2026_MaixCAM2_small/
└─ H2026_MaixCAM2_YOLO244_ROI_UART60/
   └─ maixcam2_app/
      ├─ main.py
      ├─ app.yaml
      ├─ model_308865.mud
      ├─ model_308865_npu.axmodel
      └─ model_308865_vnpu.axmodel
```

## 通信接口

### MaixCAM2 → 平衡球板

串口参数：115200 baud，8N1。

MaixCAM2 使用 UART4：

```text
A21 / UART4_TX → MSPM0G3507 RX
A22 / UART4_RX ← MSPM0G3507 TX
GND            ↔ GND
```

扩展状态帧为 20 字节：

```text
AA 5A seq flags ballN targetN velocity ballX targetX score fps mode checksum
```

其中位置使用 0～10000 的归一化坐标，中心位置为 5000；速度单位为 mm/s。完整字段说明见 MaixCAM2 工程中的 `README_CN.md` 和平衡球板的 `ball_link.h`。

### 循迹板 ↔ 平衡球板

串口参数同为 115200 baud，8N1。双板通信协议为：

- 循迹板 → 平衡球板：9 字节运行状态帧；
- 平衡球板 → 循迹板：7 字节 ACK 帧；
- 支持 READY、RUNNING、EMERGENCY、PRETILT、BALANCE_MODE 和 REQUIREMENT6 等状态位；
- 小车启动前等待平衡板完成视觉、控制、目标锁存和预倾准备。

详细协议见 [`INTERBOARD_PROTOCOL.md`](pinghengqiu/pinghengqiu/INTERBOARD_PROTOCOL.md)。

## 控制流程

1. 小车按键选择任务并进入 ARMING 状态；
2. 循迹板发送 READY/PRETILT 和预计启动加速度；
3. 平衡球板根据加速度前馈建立反向预倾角；
4. 平衡球板确认丝杆到达预倾位置后返回 PRETILT_READY；
5. 小车开始循迹运行，同时平衡球板执行视觉位置闭环；
6. 小车持续发送速度、加速度和任务状态，平衡球板实时更新补偿；
7. 到达终点或检测到异常时，小车执行减速/停车，平衡板进入安全状态。

## 运行与开发说明

### MSPM0G3507

使用 Code Composer Studio 打开 `lineCAR` 或 `pinghengqiu` 对应工程，配置目标芯片和下载器后编译、烧录。实际接线、引脚映射和任务参数以工程中的 SysConfig 文件及头文件为准。

### MaixCAM2

将 `maixcam2_app` 目录复制到 MaixCAM2，在 MaixVision 中运行 `main.py`。模型文件需要与 `main.py` 位于同一目录。首次使用时应根据镜头视野重新标定：

- ROI 的上下边界；
- 摆杆左右可达端点对应的像素坐标；
- 摆杆实际长度与像素到毫米的换算关系。

## 公开仓库注意事项

本仓库的 `.gitignore` 默认忽略 CCS 编译产物、Python 缓存、虚拟环境、测试视频、压缩包和 SolidWorks 工程文件，同时排除当前目录中与 H 题核心实现无关的培训资料。MaixCAM2 的模型文件是否提交取决于文件大小和许可证；如果模型过大，可使用 Git LFS 或在 README 中提供单独获取方式。

## 个人负责内容

主要负责项目的电控与视觉开发，包括：

- 双 MSPM0G3507 控制板的软件架构与协同控制；
- 循迹、车轮速度闭环和运动状态机；
- 平衡球位置闭环、丝杆执行器控制和加速度前馈；
- MaixCAM2 的 YOLO 钢球检测、位置/速度跟踪和串口数据发送；
- 双板通信协议、起步预倾、异常保护和 RTSP 实时图传。

## 相关资料

- `f493ffecb6efda69a3bd3fafbc8a88c2.pdf`：赛题原文；
- `pinghengqiu/pinghengqiu/README_REQ3_PB4_PRETILT_CN.txt`：要求3和起步预倾说明；
- `pinghengqiu/pinghengqiu/README_REQ3_LATE_BRAKE_RECOVERY_V2_CN.txt`：要求3后期制动与卡滞恢复说明；
- `H2026_MaixCAM2_small/H2026_MaixCAM2_YOLO244_ROI_UART60/maixcam2_app/README_CN.md`：视觉、UART 和 RTSP 说明。

## 免责声明

本仓库用于项目展示、学习和技术交流。硬件引脚、机械尺寸、模型文件和控制参数应结合实际作品重新校准，不保证未经调试即可在其他硬件上直接运行。

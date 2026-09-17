# MaixCAM2 钢球识别、UART控制与Wi-Fi录像完整系统（Python 3.13版）

## 本版本解决的问题

旧版电脑端安装脚本只会寻找Python 3.11或3.12，因此即使电脑已经安装Python 3.13，也无法创建运行环境。本版本已改为：

- 明确使用64位CPython 3.13；
- 使用独立的`.venv313`虚拟环境，不影响电脑上的全局Python；
- 使用支持Python 3.13的PySide6 Essentials、PyAV和NumPy；
- 强制安装官方二进制wheel，避免在Windows上本地编译FFmpeg/PyAV；
- 增加`check_environment.py`，启动前检查Python位数和依赖版本；
- 增加PyAV 16～18录像封装API兼容处理；
- EXE改用`onedir`模式，Qt与FFmpeg环境下比单文件模式更稳定、启动更快。

MaixCAM2端模型、RTSP图传和UART协议没有改变，MSPM0端协议解析代码也不需要因为电脑Python版本而修改。

## 第一次运行电脑端

电脑必须安装**64位Python 3.13**。然后进入：

```text
pc_receiver
```

双击：

```text
install_and_run.bat
```

脚本会依次完成：

1. 查找64位Python 3.13；
2. 创建`.venv313`；
3. 安装支持Python 3.13的依赖；
4. 检查PySide6、PyAV和NumPy；
5. 启动录像软件。

以后直接双击：

```text
run.bat
```

旧版留下的`.venv`不会被使用。如果`.venv313`安装中断或损坏，双击：

```text
reset_environment.bat
```

然后重新运行`install_and_run.bat`。

## 当前锁定的Python 3.13依赖

```text
PySide6-Essentials 6.11.1
PyAV 18.0.0
NumPy 2.5.1
```

这些包均提供Python 3.13的Windows 64位二进制wheel。正常安装不需要Visual Studio、C/C++编译器或单独安装FFmpeg。

## 系统工作方式

第3、4、5、6项测试期间，MaixCAM2同时运行两条链路：

```text
摄像头
  ├─ 224×224 RGB → YOLOv5 → UART4 → MSPM0G3507 → 摆杆闭环控制
  └─ 1280×720 NV21 → H.264 RTSP → 电脑 → 显示/录像/回放
```

两条链路并行工作，不需要切换程序。MaixVision用于Wi-Fi连接、部署和调试；比赛录像由电脑端RTSP接收软件完成。

## MaixCAM2端

在MaixVision中打开整个`maixcam2_app`目录并运行。程序会同时启动：

- YOLOv5钢球识别；
- UART4双向通信；
- 1280×720、25fps、2.5Mbps H.264 RTSP；
- `display.show()`调试预览。

终端会打印类似：

```text
rtsp://192.168.137.2:8554/live
```

## UART接线

```text
MaixCAM2 A21 / UART4_TX  → MSPM0 UART_RX
MaixCAM2 A22 / UART4_RX  ← MSPM0 UART_TX
MaixCAM2 GND              → MSPM0 GND
```

参数：115200、8N1。

## MaixCAM2发送给MSPM0

固定12字节二进制帧：

```text
AA 55 SEQ FLAGS BALL_X_L BALL_X_H TARGET_X_L TARGET_X_H SCORE_L SCORE_H MODE CHECKSUM
```

- `BALL_X`：当前钢球中心横坐标；无效时为`0xFFFF`；
- `TARGET_X`：目标横坐标；
- `FLAGS bit0`：检测有效；
- `FLAGS bit1`：目标已锁定；
- `FLAGS bit2`：RTSP已启动；
- `MODE`：0/3/4/5/6/7。

## MSPM0发送给MaixCAM2

```text
$MODE,4*       第4项，目标为中心
$MODE,5*       第5项，目标为中心
$MODE,6*       第6项，等待捕获目标
$CAPTURE*      锁存最近9帧钢球位置中值
$TARGET,123*   手动设置目标像素
$PING*         通信测试
```

第6项正确顺序：MSPM0按键后发送`$MODE,6*`和`$CAPTURE*`，等待状态帧的目标锁定位为1，再启动小车。

## 每次测试录像流程

1. 在电脑软件中输入MaixCAM2实际RTSP地址；
2. 点击“连接图传”；
3. 测试开始前约2秒点击“开始录像”；
4. 再按小车启动按键；
5. 测试结束后约2秒点击“停止录像”；
6. 在右侧录像列表双击文件回放。

录像把MaixCAM2输出的H.264数据直接封装为MKV，不进行二次编码。

## 打包Windows程序

先完成依赖安装并确保软件能运行，再双击：

```text
build_exe.bat
```

生成位置：

```text
dist\MaixCAM2_Ball_Receiver\MaixCAM2_Ball_Receiver.exe
```

需要把整个`MaixCAM2_Ball_Receiver`目录复制到比赛电脑，不能只复制其中的EXE。

## 必须标定

在`maixcam2_app/main.py`中修改：

```python
RAIL_LEFT_X = 轨道左端球心实测像素
RAIL_RIGHT_X = 轨道右端球心实测像素
```

不要长期使用默认的0和223，否则镜头安装误差会直接进入控制误差。

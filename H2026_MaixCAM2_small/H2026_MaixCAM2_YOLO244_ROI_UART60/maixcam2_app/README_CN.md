# MaixCAM2 YOLOv5 244×244 轨道 ROI 高速检测版

## 1. 默认运行参数

```text
摄像头采集：480×272 @ 60 fps
YOLO模型输入：244×244（模型文件不变）
YOLO检测区域：x=0..479，y=88..183，尺寸480×96
MaixVision刷新：30 fps
RTSP图传：640×360 @ 30 fps，1.5 Mbps
UART4：115200 8N1，约每16 ms发送一次
```

摄像头完整保留左右方向，因此轨道两端不会被裁掉；仅删除轨道上方和下方的大量无关背景。YOLO收到的是蓝色矩形中的白色管道区域。

MaixPy在送入图像尺寸与模型输入不同时，会采用保持宽高比的 `FIT_CONTAIN` 缩放并填充黑边，因此480×96轨道ROI不会被强行拉伸成正方形。

## 2. MaixVision显示内容

画面顶部显示：

```text
DET FPS 59.6  AI 8ms  OBJ 1
OK  X=241  N=5032  V=-84mm/s
TARGET X=240 N=5000 M4 Q912
```

- `DET FPS`：实际检测/控制循环帧率，不是摄像头设置值；
- `AI`：一次YOLO调用耗时；
- `OBJ`：模型当前输出目标数量；
- `X`：钢球在480×272完整画面中的横坐标；
- `N`：轨道归一化位置，左端0、中心5000、右端10000；
- `V`：钢球速度，单位mm/s，向右为正、向左为负；
- `TARGET X/N`：目标像素位置和目标归一化位置；
- `M`：工作模式；
- `Q`：置信度0～1000。

MaixVision只以30 fps刷新显示，但YOLO和UART仍按约60 Hz运行，以减小画面绘制对控制循环的影响。

## 3. 直接运行

将整个 `maixcam2_app` 文件夹复制到MaixCAM2，在MaixVision打开其中的 `main.py` 运行。以下文件必须在同一目录：

```text
main.py
model_308865.mud
model_308865_npu.axmodel
model_308865_vnpu.axmodel
app.yaml
```

UART接线：

```text
MaixCAM2 A21 / UART4_TX -> MSPM0G3507 RX
MaixCAM2 A22 / UART4_RX <- MSPM0G3507 TX
GND                     <-> GND
115200，8N1
```

## 4. 轨道ROI调整

默认蓝框：

```python
ROI_X = 0
ROI_Y = 88
ROI_WIDTH = 480
ROI_HEIGHT = 96
```

蓝框必须覆盖：

- 整根白色管道的左右两端；
- 钢球完整上下边缘；
- 钢球所有可能的运动位置。

蓝框上方仍包含较多黑色底板时，增大 `ROI_Y`；蓝框下方包含车轮时，减小 `ROI_HEIGHT`。左右方向不要缩小，除非镜头中轨道端点明显不在画面边缘。

## 5. 左右端标定

当前默认：

```python
RAIL_LEFT_X = 8
RAIL_RIGHT_X = 471
```

将钢球中心分别放到实际可达左端和右端，观察画面中的 `X=`，然后替换这两个数。不要直接使用0和479，因为钢球中心不能到达管道最外边缘。

轨道长度固定为25 cm：

```python
RAIL_LENGTH_MM = 250.0
```

速度换算基于该长度。

## 6. UART状态帧

程序默认同时发送两种帧：

### 兼容12字节帧

```text
AA 55 seq flags ballN targetN score mode checksum
```

原先使用12字节解析器的第二块MSPM0仍可继续工作。

### 新20字节扩展帧

```text
AA 5A seq flags ballN targetN velocity ballX targetX score fps mode checksum
```

全部为小端：

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
|0|0xAA|u8|帧头1|
|1|0x5A|u8|扩展帧头2|
|2|seq|u8|序号|
|3|flags|u8|状态位|
|4..5|ballN|u16|钢球归一化位置，失效为0xFFFF|
|6..7|targetN|u16|目标归一化位置|
|8..9|velocity|i16|钢球速度mm/s|
|10..11|ballX|u16|钢球横坐标，失效为0xFFFF|
|12..13|targetX|u16|目标横坐标|
|14..15|score|u16|置信度0～1000|
|16..17|fps|u16|帧率×10，例如596表示59.6fps|
|18|mode|u8|模式|
|19|checksum|u8|前19字节累加和低8位|

`flags`：

```text
bit0 钢球有效
bit1 目标已锁定
bit2 RTSP已启动
bit3 速度有效
bit4 扩展帧已启用
```

`mspm0_reference/ball_link.c/.h` 中包含新旧两种帧的解析器。

## 7. MSPM0发给MaixCAM2的命令

```text
$CENTER*         设置中心目标
$MODE,3*         要求3模式
$MODE,4*         要求4中心模式
$MODE,5*         要求5中心模式
$MODE,6*         要求6模式
$CAPTURE*        锁存最近9帧位置中值为目标
$TARGETN,7000*   设置归一化目标
$TARGET,240*     设置480×272画面像素目标
$PING*           返回$PONG*
```

## 8. 帧率不足时的处理顺序

默认配置以接近60 fps为目标，但最终帧率取决于固件、摄像头模组、模型实际推理时间和RTSP负载。观察 `DET FPS` 与 `AI`：

1. `AI > 16ms`：模型推理本身无法达到60 fps，代码侧不能完全补偿；
2. `AI < 14ms` 但 `DET FPS < 55`：先设置 `SHOW_PREVIEW = False`；
3. 仍不足时设置 `ENABLE_RTSP = False`，判断是否为图传资源占用；
4. 不要降低ROI左右宽度，否则会丢失轨道端点；可以把 `ROI_HEIGHT` 从96降到80，但必须确保钢球完整处于蓝框中。

## 9. 检测稳定性

程序除YOLO置信度外，还使用：

- 只接受 `ball` 类；
- 框宽高和长宽比检查；
- 轨道中心纵坐标评分；
- 预期钢球尺寸评分；
- Alpha-Beta位置/速度跟踪；
- 按帧间时间动态计算允许跳变量；
- 高置信度时允许钢球快速运动。

这样可以在使用较低置信度阈值识别小钢球的同时，减少螺钉、反光和背景误识别。

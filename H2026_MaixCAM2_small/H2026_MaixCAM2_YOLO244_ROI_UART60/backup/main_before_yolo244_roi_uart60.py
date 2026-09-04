"""
MaixCAM2 steel-ball detection + RTSP stream + UART4 normalized position link.

Wiring:
    MaixCAM2 A21 (UART4_TX) -> MSPM0 UART2_RX (PB16)
    MaixCAM2 A22 (UART4_RX) <- MSPM0 UART2_TX (PB15)
    GND <-> GND

UART: 115200, 8N1
MaixCAM2 -> MSPM0: fixed 12-byte binary frame.
MSPM0 -> MaixCAM2: ASCII commands enclosed by '$' and '*'.
"""

from maix import app, camera, display, err, image, nn, pinmap, rtsp, time, uart
import os
import struct

# ============================ Field tuning ============================
MODEL_PATH = "model_308865.mud"
CONF_THRESHOLD = 0.50
IOU_THRESHOLD = 0.45
AI_FPS = 30
AI_BUFFER_NUM = 2

# External receiver must record the RTSP stream for requirement 1.
RTSP_WIDTH = 1280
RTSP_HEIGHT = 720
RTSP_FPS = 25
RTSP_BITRATE = 2_500_000
RTSP_PORT = 8554
RTSP_BUFFER_NUM = 2
ENABLE_RTSP = True

UART_DEVICE = "/dev/ttyS4"
UART_BAUD = 115200
UART_TX_PIN = "A21"
UART_RX_PIN = "A22"
UART_SEND_PERIOD_MS = 20
DETECTION_TIMEOUT_MS = 120

# Put the ball at the physical left/right end of the 25 cm rail, read its
# center pixel from the preview, then replace these two values.
RAIL_LEFT_X = 0
RAIL_RIGHT_X = None  # None -> detector.input_width() - 1

# Optional vertical ROI. None means the full model input height.
RAIL_TOP_Y = 0
RAIL_BOTTOM_Y = None

# None accepts every class. Set to the trained ball class id when the model
# contains more than one class.
BALL_CLASS_ID = None

# Reject a low-confidence one-frame jump larger than this fraction of rail
# width. A high-confidence detection is still accepted for fast movement.
MAX_JUMP_FRACTION = 0.22
JUMP_ACCEPT_SCORE = 0.85
FILTER_ALPHA = 0.35
HISTORY_LENGTH = 9

POSITION_MIN = 0
POSITION_CENTER = 5000
POSITION_MAX = 10000
POSITION_INVALID = 0xFFFF

MODE_IDLE = 0
MODE_TASK3 = 3
MODE_TASK4 = 4
MODE_TASK5 = 5
MODE_TASK6 = 6
MODE_MANUAL = 7


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def median_int(values):
    if not values:
        return None
    ordered = sorted(values)
    return int(ordered[len(ordered) // 2])


def checksum8(data):
    return sum(data) & 0xFF


class BallLink:
    """Camera calibration, target handling, filtering and UART framing."""

    def __init__(self, serial_port, left_x, right_x):
        self.serial = serial_port
        self.left_x = int(left_x)
        self.right_x = int(right_x)
        self.rail_width = self.right_x - self.left_x
        self.max_jump_px = max(8, int(round(self.rail_width * MAX_JUMP_FRACTION)))

        self.mode = MODE_IDLE
        self.target_position = POSITION_CENTER
        self.target_locked = False

        self.ball_position = POSITION_INVALID
        self.ball_x_px = None
        self.ball_score_permille = 0
        self.last_detect_ms = 0
        self.filtered_x_px = None
        self.position_history = []

        self.seq = 0
        self.last_send_ms = 0
        self.rx_buffer = b""

    def pixel_to_position(self, x_px):
        x_px = clamp(float(x_px), self.left_x, self.right_x)
        ratio = (x_px - self.left_x) / float(self.rail_width)
        return int(round(ratio * POSITION_MAX))

    def position_to_pixel(self, position):
        position = clamp(int(position), POSITION_MIN, POSITION_MAX)
        return int(round(self.left_x + self.rail_width * position / POSITION_MAX))

    @property
    def center_x_px(self):
        return self.position_to_pixel(POSITION_CENTER)

    @property
    def target_x_px(self):
        return self.position_to_pixel(self.target_position)

    def update_detection(self, raw_x_px, score, now_ms):
        raw_x_px = int(clamp(raw_x_px, self.left_x, self.right_x))

        if self.filtered_x_px is not None:
            jump = abs(raw_x_px - self.filtered_x_px)
            if jump > self.max_jump_px and score < JUMP_ACCEPT_SCORE:
                return False

        if self.filtered_x_px is None:
            self.filtered_x_px = float(raw_x_px)
        else:
            self.filtered_x_px += FILTER_ALPHA * (raw_x_px - self.filtered_x_px)

        self.ball_x_px = int(round(self.filtered_x_px))
        self.ball_position = self.pixel_to_position(self.ball_x_px)
        self.ball_score_permille = int(clamp(round(score * 1000.0), 0, 1000))
        self.last_detect_ms = now_ms

        self.position_history.append(self.ball_position)
        if len(self.position_history) > HISTORY_LENGTH:
            self.position_history.pop(0)
        return True

    def detection_valid(self, now_ms):
        return (self.ball_position != POSITION_INVALID and
                (now_ms - self.last_detect_ms) <= DETECTION_TIMEOUT_MS)

    def set_center_target(self, mode):
        self.mode = mode
        self.target_position = POSITION_CENTER
        self.target_locked = True
        print("target=center mode:", self.mode)

    def set_normalized_target(self, position, mode=MODE_MANUAL):
        self.mode = mode
        self.target_position = int(clamp(position, POSITION_MIN, POSITION_MAX))
        self.target_locked = True
        print("target=normalized:", self.target_position, "mode:", self.mode)

    def set_pixel_target(self, target_x_px):
        self.set_normalized_target(self.pixel_to_position(target_x_px), MODE_MANUAL)

    def capture_current_target(self, now_ms):
        if not self.detection_valid(now_ms):
            print("capture target failed: ball invalid")
            return False
        target = median_int(self.position_history)
        if target is None:
            return False
        self.set_normalized_target(target, MODE_TASK6)
        print("target=captured position:", self.target_position)
        return True

    def handle_command(self, text, now_ms):
        """
        $CENTER*        center target, mode 4
        $MODE,3..6*     select problem mode
        $CAPTURE*       median of recent valid positions -> task 6 target
        $TARGETN,7000*  normalized target, preferred protocol
        $TARGET,123*    legacy pixel target
        $PING*          reply $PONG*
        """
        command = text.strip().upper()
        try:
            if command == "CENTER":
                self.set_center_target(MODE_TASK4)
            elif command == "CAPTURE":
                ok = self.capture_current_target(now_ms)
                self.serial.write_str("$CAPTURED*" if ok else "$NO_BALL*")
            elif command == "PING":
                self.serial.write_str("$PONG*")
            elif command.startswith("MODE,"):
                mode = int(command.split(",", 1)[1])
                if mode in (MODE_TASK4, MODE_TASK5):
                    self.set_center_target(mode)
                elif mode in (MODE_TASK3, MODE_TASK6):
                    self.mode = mode
                    self.target_locked = False
                    print("mode=", mode, "waiting target/capture")
                else:
                    self.mode = MODE_IDLE
                    self.target_locked = False
            elif command.startswith("TARGETN,"):
                target = int(command.split(",", 1)[1])
                # Keep task 3 mode when its MCU sends +5/-5 setpoints.
                mode = MODE_TASK3 if self.mode == MODE_TASK3 else MODE_MANUAL
                self.set_normalized_target(target, mode)
            elif command.startswith("TARGET,"):
                self.set_pixel_target(int(command.split(",", 1)[1]))
            else:
                print("unknown uart command:", command)
        except Exception as exc:
            print("bad uart command:", command, exc)

    def poll_commands(self, now_ms):
        data = self.serial.read()
        if not data:
            return
        self.rx_buffer += data
        if len(self.rx_buffer) > 256:
            self.rx_buffer = self.rx_buffer[-128:]

        while True:
            start = self.rx_buffer.find(b"$")
            if start < 0:
                self.rx_buffer = b""
                return
            end = self.rx_buffer.find(b"*", start + 1)
            if end < 0:
                if start > 0:
                    self.rx_buffer = self.rx_buffer[start:]
                return

            payload = self.rx_buffer[start + 1:end]
            self.rx_buffer = self.rx_buffer[end + 1:]
            try:
                self.handle_command(payload.decode("ascii"), now_ms)
            except Exception as exc:
                print("uart decode error:", exc)

    def send_status(self, now_ms, stream_running):
        if now_ms - self.last_send_ms < UART_SEND_PERIOD_MS:
            return
        self.last_send_ms = now_ms

        valid = self.detection_valid(now_ms)
        ball_position = self.ball_position if valid else POSITION_INVALID

        flags = 0
        if valid:
            flags |= 0x01
        if self.target_locked:
            flags |= 0x02
        if stream_running:
            flags |= 0x04

        # 0 AA, 1 55, 2 seq, 3 flags,
        # 4-5 normalized ball position, 6-7 normalized target position,
        # 8-9 score permille, 10 mode, 11 checksum(sum[0:11] & 0xFF).
        body = struct.pack(
            "<BBBBHHHB",
            0xAA,
            0x55,
            self.seq,
            flags,
            int(ball_position),
            int(self.target_position),
            int(self.ball_score_permille if valid else 0),
            int(self.mode),
        )
        self.serial.write(body + bytes([checksum8(body)]))
        self.seq = (self.seq + 1) & 0xFF


# ============================ Initialization ============================
if not os.path.exists(MODEL_PATH):
    MODEL_PATH = "/root/models/model_308865.mud"

print("loading model:", MODEL_PATH)
detector = nn.YOLOv5(model=MODEL_PATH)
model_width = detector.input_width()
model_height = detector.input_height()
right_x = model_width - 1 if RAIL_RIGHT_X is None else int(RAIL_RIGHT_X)
bottom_y = model_height - 1 if RAIL_BOTTOM_Y is None else int(RAIL_BOTTOM_Y)

if not (0 <= RAIL_LEFT_X < right_x < model_width):
    raise ValueError("RAIL_LEFT_X / RAIL_RIGHT_X calibration is invalid")
if not (0 <= RAIL_TOP_Y < bottom_y < model_height):
    raise ValueError("RAIL_TOP_Y / RAIL_BOTTOM_Y is invalid")

print("model input:", model_width, "x", model_height)
print("rail ROI: x", RAIL_LEFT_X, "..", right_x,
      "y", RAIL_TOP_Y, "..", bottom_y)

cam_ai = camera.Camera(
    model_width,
    model_height,
    detector.input_format(),
    fps=AI_FPS,
    buff_num=AI_BUFFER_NUM,
)

stream_server = None
stream_running = False
if ENABLE_RTSP:
    cam_stream = cam_ai.add_channel(
        RTSP_WIDTH,
        RTSP_HEIGHT,
        image.Format.FMT_YVU420SP,
        fps=RTSP_FPS,
        buff_num=RTSP_BUFFER_NUM,
    )
    stream_server = rtsp.Rtsp(
        port=RTSP_PORT,
        fps=RTSP_FPS,
        bitrate=RTSP_BITRATE,
    )
    err.check_raise(stream_server.bind_camera(cam_stream), "bind RTSP camera failed")
    err.check_raise(stream_server.start(), "start RTSP failed")
    stream_running = True
    print("RTSP URLs:")
    for url in stream_server.get_urls():
        print("  ", url)

err.check_raise(pinmap.set_pin_function(UART_TX_PIN, "UART4_TX"), "set UART4 TX failed")
err.check_raise(pinmap.set_pin_function(UART_RX_PIN, "UART4_RX"), "set UART4 RX failed")
serial = uart.UART(UART_DEVICE, UART_BAUD)
print("UART ready:", UART_DEVICE, UART_BAUD)

link = BallLink(serial, RAIL_LEFT_X, right_x)
dis = display.Display()
link.set_center_target(MODE_TASK4)

# ============================ Main loop ============================
while not app.need_exit():
    now_ms = time.ticks_ms()
    link.poll_commands(now_ms)

    img = cam_ai.read()
    objs = detector.detect(img, conf_th=CONF_THRESHOLD, iou_th=IOU_THRESHOLD)

    candidates = []
    for obj in objs:
        center_x = int(round(obj.x + obj.w / 2.0))
        center_y = int(round(obj.y + obj.h / 2.0))
        if BALL_CLASS_ID is not None and obj.class_id != BALL_CLASS_ID:
            continue
        if not (RAIL_LEFT_X <= center_x <= right_x and
                RAIL_TOP_Y <= center_y <= bottom_y):
            continue
        candidates.append((obj, center_x))

    # Prefer high confidence; when confidence is similar, prefer continuity.
    best = None
    best_x = None
    best_metric = None
    for obj, center_x in candidates:
        continuity_penalty = 0.0
        if link.filtered_x_px is not None:
            continuity_penalty = abs(center_x - link.filtered_x_px) / max(1, link.rail_width)
        metric = float(obj.score) - 0.20 * continuity_penalty
        if best_metric is None or metric > best_metric:
            best, best_x, best_metric = obj, center_x, metric

    if best is not None and link.update_detection(best_x, best.score, now_ms):
        img.draw_rect(best.x, best.y, best.w, best.h,
                      color=image.COLOR_RED, thickness=2)
        img.draw_string(
            best.x,
            max(0, best.y - 18),
            "ball N={} px={} {:.2f}".format(
                link.ball_position, link.ball_x_px, best.score),
            color=image.COLOR_RED,
        )

    # Preview overlays. RTSP itself is a clean camera channel; recording is done
    # by the external receiver as required by the problem statement.
    img.draw_rect(link.left_x, RAIL_TOP_Y, 2, bottom_y - RAIL_TOP_Y,
                  color=image.COLOR_BLUE, thickness=-1)
    img.draw_rect(link.right_x - 1, RAIL_TOP_Y, 2, bottom_y - RAIL_TOP_Y,
                  color=image.COLOR_BLUE, thickness=-1)
    img.draw_rect(link.center_x_px, RAIL_TOP_Y, 2, bottom_y - RAIL_TOP_Y,
                  color=image.COLOR_GREEN, thickness=-1)
    img.draw_rect(link.target_x_px, RAIL_TOP_Y, 2, bottom_y - RAIL_TOP_Y,
                  color=image.COLOR_YELLOW, thickness=-1)

    valid_text = "OK" if link.detection_valid(now_ms) else "LOST"
    img.draw_string(
        2,
        2,
        "{} M{} B{} T{}".format(
            valid_text, link.mode, link.ball_position, link.target_position),
        color=image.COLOR_WHITE,
    )

    dis.show(img)
    link.send_status(now_ms, stream_running)
    time.sleep_ms(1)

if stream_server is not None:
    stream_server.stop()

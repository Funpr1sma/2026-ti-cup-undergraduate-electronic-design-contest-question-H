"""
MaixCAM2 YOLOv5 244x244 steel-ball tracker.

Features:
  * Camera: 480x272 @ 60 fps.
  * YOLO only receives the central white-pipe ROI.
  * YOLOv5 dual-buffer pipeline.
  * MaixVision displays measured detection FPS, inference time, position,
    target and ball velocity.
  * UART4 sends the original 12-byte frame and a new 20-byte extended frame.
  * RTSP remains enabled on an independent hardware camera channel.

Wiring:
  MaixCAM2 A21 / UART4_TX -> MSPM0G3507 RX
  MaixCAM2 A22 / UART4_RX <- MSPM0G3507 TX
  GND                     <-> GND
  UART: 115200, 8N1
"""

from maix import app, camera, display, err, image, nn, pinmap, rtsp, time, uart
import os
import struct

# ============================ Performance profile ============================
APP_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(APP_DIR, "model_308865.mud")

# The uploaded model is fixed at 244x244. The camera is deliberately larger and
# wide enough to retain the complete rail. detect() uses FIT_CONTAIN internally,
# preserving the ROI aspect ratio instead of stretching it.
EXPECTED_MODEL_WIDTH = 244
EXPECTED_MODEL_HEIGHT = 244
CAMERA_WIDTH = 480
CAMERA_HEIGHT = 272
CAMERA_FPS = 60
CAMERA_BUFFER_NUM = 2
CAMERA_WARMUP_FRAMES = 12

# Only this full-width central strip is sent to YOLO.
# Based on the supplied 640x360 live view, the rail is around y=135..215.
# The equivalent region at 480x272 is about y=102..162; extra margins are kept.
ROI_X = 0
ROI_Y = 84
ROI_WIDTH = 480
ROI_HEIGHT = 64

# Detection settings. The ROI, class, shape and temporal checks allow a lower
# confidence threshold without readily accepting background false detections.
CONF_THRESHOLD = 0.35
IOU_THRESHOLD = 0.45
BALL_CLASS_ID = 0
BALL_BOX_MIN_WIDTH = 4
BALL_BOX_MAX_WIDTH = 80
BALL_BOX_MIN_HEIGHT = 4
BALL_BOX_MAX_HEIGHT = 80
BALL_ASPECT_MIN = 0.45
BALL_ASPECT_MAX = 2.20
BALL_EXPECTED_DIAMETER_PX = 16.0

# Alpha-beta tracker. Position and speed are estimated together at about 60 Hz.
TRACK_ALPHA = 0.72
TRACK_BETA = 0.12
BASE_ALLOWED_JUMP_PX = 10.0
MAX_BALL_SPEED_PX_S = 2600.0
HIGH_SCORE_JUMP_ACCEPT = 0.86
TRACK_RESET_MS = 220
DETECTION_TIMEOUT_MS = 100

# The ball centre cannot physically reach the outermost image pixels. These
# defaults assume an approximately 16 px ball at 480x272. Recalibrate in situ.
RAIL_LEFT_X = 8
RAIL_RIGHT_X = 471
RAIL_LENGTH_MM = 230.0

# Display is intentionally decoupled from inference. Inference runs every camera
# frame; MaixVision is refreshed at 30 fps to reduce display overhead.
DISPLAY_FPS = 30
DISPLAY_PERIOD_MS = max(1, int(round(1000.0 / DISPLAY_FPS)))
PRINT_STATUS_PERIOD_MS = 1000
SHOW_PREVIEW = True

# RTSP is a separate hardware camera channel. 30 fps is sufficient for recording
# and leaves more bandwidth for the 60 fps control loop. Set False to benchmark
# the absolute maximum detection FPS.
ENABLE_RTSP = True
RTSP_WIDTH = 640
RTSP_HEIGHT = 360
RTSP_FPS = 30
RTSP_BITRATE = 1_500_000
RTSP_PORT = 8554
RTSP_BUFFER_NUM = 2

# UART. A new status is produced at roughly camera rate.
UART_DEVICE = "/dev/ttyS4"
UART_BAUD = 115200
UART_TX_PIN = "A21"
UART_RX_PIN = "A22"
UART_SEND_PERIOD_MS = 16
SEND_LEGACY_FRAME = True
SEND_EXTENDED_FRAME = True

POSITION_MIN = 0
POSITION_CENTER = 5000
POSITION_MAX = 10000
POSITION_INVALID = 0xFFFF
PIXEL_INVALID = 0xFFFF
VELOCITY_MIN_MM_S = -32768
VELOCITY_MAX_MM_S = 32767

MODE_IDLE = 0
MODE_TASK3 = 3
MODE_TASK4 = 4
MODE_TASK5 = 5
MODE_TASK6 = 6
MODE_MANUAL = 7
HISTORY_LENGTH = 9


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def median_int(values):
    if not values:
        return None
    ordered = sorted(values)
    return int(ordered[len(ordered) // 2])


def checksum8(data):
    return sum(data) & 0xFF


def validate_geometry():
    if ROI_X < 0 or ROI_Y < 0 or ROI_WIDTH < 2 or ROI_HEIGHT < 2:
        raise ValueError("invalid ROI geometry")
    if ROI_X + ROI_WIDTH > CAMERA_WIDTH or ROI_Y + ROI_HEIGHT > CAMERA_HEIGHT:
        raise ValueError("ROI exceeds camera image")
    if not (ROI_X <= RAIL_LEFT_X < RAIL_RIGHT_X < ROI_X + ROI_WIDTH):
        raise ValueError("RAIL_LEFT_X / RAIL_RIGHT_X must be inside ROI")


class FpsMeter:
    """Windowed FPS measurement; steadier than 1 ms frame-period inversion."""

    def __init__(self, window_ms=500):
        self.window_ms = int(window_ms)
        self.window_start_ms = 0
        self.frame_count = 0
        self.fps = 0.0

    def tick(self, now_ms):
        if self.window_start_ms == 0:
            self.window_start_ms = now_ms
        self.frame_count += 1
        elapsed = now_ms - self.window_start_ms
        if elapsed >= self.window_ms:
            self.fps = self.frame_count * 1000.0 / float(max(1, elapsed))
            self.frame_count = 0
            self.window_start_ms = now_ms
        return self.fps


class BallLink:
    """Tracking, calibration, target handling and UART framing."""

    def __init__(self, serial_port, left_x, right_x):
        self.serial = serial_port
        self.left_x = int(left_x)
        self.right_x = int(right_x)
        self.rail_width_px = self.right_x - self.left_x

        self.mode = MODE_IDLE
        self.target_position = POSITION_CENTER
        self.target_locked = False

        self.ball_position = POSITION_INVALID
        self.ball_x_px = None
        self.filtered_x_px = None
        self.ball_velocity_px_s = 0.0
        self.ball_velocity_mm_s = 0
        self.ball_score_permille = 0
        self.last_detect_ms = 0
        self.filter_time_ms = 0
        self.position_history = []

        self.seq = 0
        self.last_send_ms = 0
        self.rx_buffer = b""

    def pixel_to_position(self, x_px):
        x_px = clamp(float(x_px), self.left_x, self.right_x)
        ratio = (x_px - self.left_x) / float(self.rail_width_px)
        return int(round(ratio * POSITION_MAX))

    def position_to_pixel(self, position):
        position = clamp(int(position), POSITION_MIN, POSITION_MAX)
        return int(round(self.left_x + self.rail_width_px * position / POSITION_MAX))

    @property
    def center_x_px(self):
        return self.position_to_pixel(POSITION_CENTER)

    @property
    def target_x_px(self):
        return self.position_to_pixel(self.target_position)

    def predicted_x(self, now_ms):
        if self.filtered_x_px is None or self.filter_time_ms == 0:
            return None
        dt_s = clamp((now_ms - self.filter_time_ms) / 1000.0, 0.0, 0.10)
        return self.filtered_x_px + self.ball_velocity_px_s * dt_s

    def update_detection(self, raw_x_px, score, now_ms):
        raw_x_px = float(clamp(raw_x_px, self.left_x, self.right_x))

        if self.filtered_x_px is None or self.filter_time_ms == 0 or \
                now_ms - self.filter_time_ms > TRACK_RESET_MS:
            filtered_x = raw_x_px
            velocity_px_s = 0.0
        else:
            dt_ms = max(1, now_ms - self.filter_time_ms)
            dt_s = clamp(dt_ms / 1000.0, 0.005, 0.10)
            prediction = self.filtered_x_px + self.ball_velocity_px_s * dt_s
            residual = raw_x_px - prediction

            allowed_jump = BASE_ALLOWED_JUMP_PX + MAX_BALL_SPEED_PX_S * dt_s
            if abs(residual) > allowed_jump and score < HIGH_SCORE_JUMP_ACCEPT:
                return False

            filtered_x = prediction + TRACK_ALPHA * residual
            velocity_px_s = self.ball_velocity_px_s + TRACK_BETA * residual / dt_s
            velocity_px_s = clamp(
                velocity_px_s,
                -MAX_BALL_SPEED_PX_S,
                MAX_BALL_SPEED_PX_S,
            )

        self.filtered_x_px = float(clamp(filtered_x, self.left_x, self.right_x))
        self.ball_x_px = int(round(self.filtered_x_px))
        self.ball_velocity_px_s = float(velocity_px_s)
        velocity_mm_s = velocity_px_s * RAIL_LENGTH_MM / float(self.rail_width_px)
        self.ball_velocity_mm_s = int(round(clamp(
            velocity_mm_s,
            VELOCITY_MIN_MM_S,
            VELOCITY_MAX_MM_S,
        )))
        self.ball_position = self.pixel_to_position(self.ball_x_px)
        self.ball_score_permille = int(clamp(round(score * 1000.0), 0, 1000))
        self.last_detect_ms = now_ms
        self.filter_time_ms = now_ms

        self.position_history.append(self.ball_position)
        if len(self.position_history) > HISTORY_LENGTH:
            self.position_history.pop(0)
        return True

    def detection_valid(self, now_ms):
        return (
            self.ball_position != POSITION_INVALID and
            (now_ms - self.last_detect_ms) <= DETECTION_TIMEOUT_MS
        )

    def valid_velocity_mm_s(self, now_ms):
        return self.ball_velocity_mm_s if self.detection_valid(now_ms) else 0

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
        $MODE,3..6*     select task mode
        $CAPTURE*       recent median position becomes task-6 target
        $TARGETN,7000*  normalized target (0..10000)
        $TARGET,123*    full-camera pixel target
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

    def _flags(self, now_ms, stream_running):
        flags = 0
        if self.detection_valid(now_ms):
            flags |= 0x01
        if self.target_locked:
            flags |= 0x02
        if stream_running:
            flags |= 0x04
        if self.detection_valid(now_ms):
            flags |= 0x08  # velocity valid
        if SEND_EXTENDED_FRAME:
            flags |= 0x10
        return flags

    def _legacy_frame(self, now_ms, stream_running):
        valid = self.detection_valid(now_ms)
        body = struct.pack(
            "<BBBBHHHB",
            0xAA,
            0x55,
            self.seq,
            self._flags(now_ms, stream_running),
            int(self.ball_position if valid else POSITION_INVALID),
            int(self.target_position),
            int(self.ball_score_permille if valid else 0),
            int(self.mode),
        )
        return body + bytes([checksum8(body)])

    def _extended_frame(self, now_ms, stream_running, measured_fps):
        valid = self.detection_valid(now_ms)
        ball_position = self.ball_position if valid else POSITION_INVALID
        ball_x_px = self.ball_x_px if valid and self.ball_x_px is not None else PIXEL_INVALID
        velocity_mm_s = self.valid_velocity_mm_s(now_ms)
        fps_x10 = int(clamp(round(measured_fps * 10.0), 0, 65535))
        body = struct.pack(
            "<BBBBHHhHHHHB",
            0xAA,
            0x5A,
            self.seq,
            self._flags(now_ms, stream_running),
            int(ball_position),
            int(self.target_position),
            int(velocity_mm_s),
            int(ball_x_px),
            int(self.target_x_px),
            int(self.ball_score_permille if valid else 0),
            int(fps_x10),
            int(self.mode),
        )
        return body + bytes([checksum8(body)])

    def send_status(self, now_ms, stream_running, measured_fps):
        if now_ms - self.last_send_ms < UART_SEND_PERIOD_MS:
            return False
        self.last_send_ms = now_ms

        if SEND_LEGACY_FRAME:
            self.serial.write(self._legacy_frame(now_ms, stream_running))
        if SEND_EXTENDED_FRAME:
            self.serial.write(self._extended_frame(now_ms, stream_running, measured_fps))
        self.seq = (self.seq + 1) & 0xFF
        return True


def candidate_metric(obj, center_x, center_y, link, now_ms):
    width = float(obj.w)
    height = float(obj.h)
    if width < BALL_BOX_MIN_WIDTH or width > BALL_BOX_MAX_WIDTH:
        return None
    if height < BALL_BOX_MIN_HEIGHT or height > BALL_BOX_MAX_HEIGHT:
        return None
    aspect = width / max(1.0, height)
    if aspect < BALL_ASPECT_MIN or aspect > BALL_ASPECT_MAX:
        return None

    predicted = link.predicted_x(now_ms)
    if predicted is None:
        continuity = 1.0
    else:
        distance = abs(center_x - predicted)
        continuity = clamp(1.0 - distance / max(20.0, link.rail_width_px * 0.30), 0.0, 1.0)

    roi_center_y = ROI_Y + ROI_HEIGHT * 0.5
    y_score = clamp(1.0 - abs(center_y - roi_center_y) / max(1.0, ROI_HEIGHT * 0.5), 0.0, 1.0)
    diameter = 0.5 * (width + height)
    size_score = clamp(
        1.0 - abs(diameter - BALL_EXPECTED_DIAMETER_PX) /
        max(8.0, BALL_EXPECTED_DIAMETER_PX),
        0.0,
        1.0,
    )
    shape_score = clamp(1.0 - abs(1.0 - aspect), 0.0, 1.0)

    return (
        float(obj.score) +
        0.16 * continuity +
        0.06 * y_score +
        0.05 * size_score +
        0.04 * shape_score
    )


# ============================ Initialization ============================
validate_geometry()
if not os.path.exists(MODEL_PATH):
    fallback_model = "/root/models/model_308865.mud"
    if os.path.exists(fallback_model):
        MODEL_PATH = fallback_model
    else:
        raise FileNotFoundError("model_308865.mud not found beside main.py")

print("loading model:", MODEL_PATH)
detector = nn.YOLOv5(model=MODEL_PATH, dual_buff=True)
model_width = detector.input_width()
model_height = detector.input_height()
print("model input:", model_width, "x", model_height)
if model_width != EXPECTED_MODEL_WIDTH or model_height != EXPECTED_MODEL_HEIGHT:
    print("WARNING: expected 244x244 model, actual:", model_width, "x", model_height)

print("camera:", CAMERA_WIDTH, "x", CAMERA_HEIGHT, "@", CAMERA_FPS, "fps")
print("YOLO ROI: x", ROI_X, "..", ROI_X + ROI_WIDTH - 1,
      "y", ROI_Y, "..", ROI_Y + ROI_HEIGHT - 1,
      "size", ROI_WIDTH, "x", ROI_HEIGHT)
print("rail calibration:", RAIL_LEFT_X, "..", RAIL_RIGHT_X)

cam_ai = camera.Camera(
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    detector.input_format(),
    fps=CAMERA_FPS,
    buff_num=CAMERA_BUFFER_NUM,
)
try:
    cam_ai.skip_frames(CAMERA_WARMUP_FRAMES)
except Exception as exc:
    print("camera skip_frames unavailable:", exc)

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

link = BallLink(serial, RAIL_LEFT_X, RAIL_RIGHT_X)
link.set_center_target(MODE_TASK4)
dis = display.Display()
fps_meter = FpsMeter(window_ms=500)

last_display_ms = 0
last_print_ms = 0
last_infer_ms = 0
last_obj_count = 0
last_uart_send_ms = 0

# ============================ Main loop ============================
try:
    while not app.need_exit():
        frame_start_ms = time.ticks_ms()
        measured_fps = fps_meter.tick(frame_start_ms)
        link.poll_commands(frame_start_ms)

        full_img = cam_ai.read()
        # The complete rail width is retained, while the top and bottom vehicle
        # background are removed before NPU inference.
        roi_img = full_img.crop(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT)

        infer_start_ms = time.ticks_ms()
        objects = detector.detect(
            roi_img,
            conf_th=CONF_THRESHOLD,
            iou_th=IOU_THRESHOLD,
        )
        last_infer_ms = max(0, time.ticks_ms() - infer_start_ms)
        last_obj_count = len(objects)

        best = None
        best_metric = None
        for obj in objects:
            if BALL_CLASS_ID is not None and obj.class_id != BALL_CLASS_ID:
                continue

            box_x = ROI_X + int(obj.x)
            box_y = ROI_Y + int(obj.y)
            box_w = int(obj.w)
            box_h = int(obj.h)
            center_x = box_x + int(round(box_w * 0.5))
            center_y = box_y + int(round(box_h * 0.5))

            if not (ROI_X <= center_x < ROI_X + ROI_WIDTH and
                    ROI_Y <= center_y < ROI_Y + ROI_HEIGHT):
                continue
            if not (link.left_x <= center_x <= link.right_x):
                continue

            metric = candidate_metric(obj, center_x, center_y, link, frame_start_ms)
            if metric is None:
                continue
            if best_metric is None or metric > best_metric:
                best = (obj, box_x, box_y, box_w, box_h, center_x, center_y)
                best_metric = metric

        if best is not None:
            obj, box_x, box_y, box_w, box_h, center_x, center_y = best
            accepted = link.update_detection(center_x, float(obj.score), frame_start_ms)
        else:
            accepted = False

        if link.send_status(frame_start_ms, stream_running, measured_fps):
            last_uart_send_ms = frame_start_ms

        # Drawing and MaixVision output are rate-limited. This keeps the control
        # loop near camera rate while still providing a live diagnostic view.
        if SHOW_PREVIEW and frame_start_ms - last_display_ms >= DISPLAY_PERIOD_MS:
            last_display_ms = frame_start_ms

            full_img.draw_rect(
                ROI_X,
                ROI_Y,
                ROI_WIDTH,
                ROI_HEIGHT,
                color=image.COLOR_BLUE,
                thickness=2,
            )
            full_img.draw_rect(
                link.center_x_px,
                ROI_Y,
                2,
                ROI_HEIGHT,
                color=image.COLOR_GREEN,
                thickness=-1,
            )
            full_img.draw_rect(
                link.target_x_px,
                ROI_Y,
                2,
                ROI_HEIGHT,
                color=image.COLOR_YELLOW,
                thickness=-1,
            )

            if accepted and best is not None:
                full_img.draw_rect(
                    box_x,
                    box_y,
                    box_w,
                    box_h,
                    color=image.COLOR_RED,
                    thickness=2,
                )

            valid = link.detection_valid(frame_start_ms)
            valid_text = "OK" if valid else "LOST"
            ball_pos_text = link.ball_position if valid else -1
            ball_px_text = link.ball_x_px if valid and link.ball_x_px is not None else -1
            velocity_text = link.valid_velocity_mm_s(frame_start_ms)

            full_img.draw_rect(0, 0, CAMERA_WIDTH, 64,
                               color=image.COLOR_BLACK, thickness=-1)
            full_img.draw_string(
                4,
                2,
                "DET FPS {:.1f}  AI {}ms  OBJ {}".format(
                    measured_fps,
                    last_infer_ms,
                    last_obj_count,
                ),
                color=image.COLOR_GREEN,
            )
            full_img.draw_string(
                4,
                22,
                "{}  X={}  N={}  V={:+d}mm/s".format(
                    valid_text,
                    ball_px_text,
                    ball_pos_text,
                    velocity_text,
                ),
                color=image.COLOR_WHITE if valid else image.COLOR_RED,
            )
            full_img.draw_string(
                4,
                42,
                "TARGET X={} N={} M{} Q{}".format(
                    link.target_x_px,
                    link.target_position,
                    link.mode,
                    link.ball_score_permille if valid else 0,
                ),
                color=image.COLOR_YELLOW,
            )
            dis.show(full_img)

        if frame_start_ms - last_print_ms >= PRINT_STATUS_PERIOD_MS:
            last_print_ms = frame_start_ms
            print(
                "fps={:.1f} infer={}ms objs={} valid={} x={} pos={} target={} vel_mm_s={} score={} roi={}x{}".format(
                    measured_fps,
                    last_infer_ms,
                    last_obj_count,
                    1 if link.detection_valid(frame_start_ms) else 0,
                    link.ball_x_px if link.ball_x_px is not None else -1,
                    link.ball_position,
                    link.target_position,
                    link.valid_velocity_mm_s(frame_start_ms),
                    link.ball_score_permille,
                    ROI_WIDTH,
                    ROI_HEIGHT,
                )
            )
finally:
    if stream_server is not None:
        stream_server.stop()

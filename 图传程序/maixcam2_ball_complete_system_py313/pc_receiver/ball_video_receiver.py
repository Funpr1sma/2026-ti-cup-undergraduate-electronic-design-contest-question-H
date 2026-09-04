"""
MaixCAM2 steel-ball video receiver for Windows/Linux.

Features:
- Low-latency RTSP/TCP live preview
- Manual start/stop recording
- H.264 packet remux to MKV (no re-encoding)
- Automatic reconnection
- Timestamped, test-labelled recording files
- Snapshot
- Recording browser and built-in playback

Python: CPython 3.13 (64-bit) supported.
"""

from __future__ import annotations

import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    import av
    import numpy as np  # noqa: F401 - required by PyAV frame.to_ndarray()
    from PySide6 import __version__ as PYSIDE_VERSION
except ImportError as exc:
    print(
        "Missing Python 3.13 dependencies. "
        "Run install_and_run.bat before starting this program.\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc

from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtGui import QCloseEvent, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QSizePolicy,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

APP_TITLE = "MaixCAM2 摆球图传录像系统"
DEFAULT_RTSP_URL = "rtsp://192.168.137.2:8554/live"
DEFAULT_RECORD_DIR = Path.home() / "Videos" / "MaixCAM2_Ball_Recordings"


def resource_base_dir() -> Path:
    """Folder beside the executable/script, useful for future resources."""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


def safe_file_part(text: str) -> str:
    text = text.strip()
    if not text:
        return "test"
    text = re.sub(r"[\\/:*?\"<>|\s]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_.")
    return text[:48] or "test"


def format_seconds(value: float) -> str:
    total = max(0, int(value))
    hours, rem = divmod(total, 3600)
    minutes, seconds = divmod(rem, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


@dataclass
class StreamStats:
    width: int = 0
    height: int = 0
    fps: float = 0.0


class LiveStreamThread(QThread):
    frame_ready = Signal(QImage)
    status_changed = Signal(str)
    stats_changed = Signal(object)
    recording_changed = Signal(str, str)  # state, path/message
    stream_error = Signal(str)

    def __init__(self, url: str, record_dir: Path, test_label: str) -> None:
        super().__init__()
        self.url = url
        self.record_dir = record_dir
        self.test_label = test_label

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._record_requested = False
        self._record_label = test_label

        self._output: Optional[av.container.OutputContainer] = None
        self._out_stream = None
        self._record_path: Optional[Path] = None
        self._base_pts: Optional[int] = None
        self._base_dts: Optional[int] = None
        self._waiting_emitted = False

    def request_stop(self) -> None:
        self._stop_event.set()

    def set_recording(self, enabled: bool, label: str) -> None:
        with self._lock:
            self._record_requested = enabled
            self._record_label = label

    def _record_request(self) -> tuple[bool, str]:
        with self._lock:
            return self._record_requested, self._record_label

    def _start_recording(self, in_stream, first_packet) -> None:
        self.record_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        label = safe_file_part(self._record_request()[1])
        path = self.record_dir / f"{timestamp}_{label}.mkv"

        output = av.open(str(path), mode="w", format="matroska")

        # PyAV 16-18 compatibility: newer releases expose
        # add_stream_from_template(), while add_stream(template=...) remains
        # available in other supported builds.
        add_from_template = getattr(output, "add_stream_from_template", None)
        if callable(add_from_template):
            out_stream = add_from_template(in_stream)
        else:
            out_stream = output.add_stream(template=in_stream)

        try:
            out_stream.time_base = in_stream.time_base
        except Exception:
            pass

        self._output = output
        self._out_stream = out_stream
        self._record_path = path
        self._base_pts = first_packet.pts
        self._base_dts = first_packet.dts
        self._waiting_emitted = False
        self.recording_changed.emit("recording", str(path))

    def _stop_recording(self, completed: bool = True) -> None:
        output = self._output
        path = self._record_path
        self._output = None
        self._out_stream = None
        self._record_path = None
        self._base_pts = None
        self._base_dts = None
        self._waiting_emitted = False

        if output is not None:
            try:
                output.close()
                state = "saved" if completed else "interrupted"
                self.recording_changed.emit(state, str(path or ""))
            except Exception as exc:
                self.recording_changed.emit("error", f"录像封装失败：{exc}")

    def _mux_packet(self, packet) -> None:
        if self._output is None or self._out_stream is None:
            return

        # Rebase timestamps so each test video begins near 00:00:00.
        if packet.pts is not None and self._base_pts is not None:
            packet.pts -= self._base_pts
        if packet.dts is not None and self._base_dts is not None:
            packet.dts -= self._base_dts
        packet.stream = self._out_stream
        self._output.mux(packet)

    def _close_input(self, container) -> None:
        try:
            container.close()
        except Exception:
            pass

    def run(self) -> None:
        retry_count = 0
        while not self._stop_event.is_set():
            container = None
            try:
                self.status_changed.emit("正在连接 RTSP…")
                options = {
                    "rtsp_transport": "tcp",
                    "fflags": "nobuffer",
                    "flags": "low_delay",
                    "max_delay": "100000",
                    "reorder_queue_size": "0",
                    "probesize": "65536",
                    "analyzeduration": "500000",
                }
                container = av.open(
                    self.url,
                    mode="r",
                    options=options,
                    timeout=(5.0, 3.0),
                )
                in_stream = container.streams.video[0]
                in_stream.thread_type = "SLICE"  # lower latency than AUTO/FRAME

                retry_count = 0
                self.status_changed.emit("已连接，正在接收画面")

                fps_window_start = time.monotonic()
                fps_frames = 0
                last_width = 0
                last_height = 0

                for packet in container.demux(in_stream):
                    if self._stop_event.is_set():
                        break
                    if packet.size == 0:
                        continue

                    record_requested, _ = self._record_request()
                    if not record_requested and self._output is not None:
                        self._stop_recording(completed=True)

                    # Decode before changing packet timestamps/stream for remuxing.
                    try:
                        frames = packet.decode()
                    except av.FFmpegError:
                        frames = []

                    for frame in frames:
                        array = frame.to_ndarray(format="rgb24")
                        height, width, _ = array.shape
                        image = QImage(
                            array.data,
                            width,
                            height,
                            array.strides[0],
                            QImage.Format.Format_RGB888,
                        ).copy()
                        self.frame_ready.emit(image)

                        fps_frames += 1
                        last_width, last_height = width, height
                        now = time.monotonic()
                        elapsed = now - fps_window_start
                        if elapsed >= 1.0:
                            fps = fps_frames / elapsed
                            self.stats_changed.emit(StreamStats(width, height, fps))
                            fps_window_start = now
                            fps_frames = 0

                    # Start only on a keyframe, ensuring independent playback.
                    record_requested, _ = self._record_request()
                    if record_requested and self._output is None:
                        if packet.is_keyframe:
                            try:
                                self._start_recording(in_stream, packet)
                            except Exception as exc:
                                self.recording_changed.emit("error", f"无法开始录像：{exc}")
                                with self._lock:
                                    self._record_requested = False
                        elif not self._waiting_emitted:
                            self._waiting_emitted = True
                            self.recording_changed.emit("waiting", "等待视频关键帧…")

                    if record_requested and self._output is not None:
                        try:
                            self._mux_packet(packet)
                        except Exception as exc:
                            self.recording_changed.emit("error", f"录像写入失败：{exc}")
                            self._stop_recording(completed=False)
                            with self._lock:
                                self._record_requested = False

                if last_width and last_height:
                    self.stats_changed.emit(StreamStats(last_width, last_height, 0.0))

                if self._stop_event.is_set():
                    break
                raise RuntimeError("RTSP 数据流已中断")

            except Exception as exc:
                if self._stop_event.is_set():
                    break
                retry_count += 1
                self._stop_recording(completed=False)
                message = f"连接中断：{exc}；1 秒后重连（第 {retry_count} 次）"
                self.status_changed.emit(message)
                self.stream_error.emit(str(exc))
                self._close_input(container)
                for _ in range(10):
                    if self._stop_event.is_set():
                        break
                    time.sleep(0.1)
            finally:
                self._close_input(container)

        self._stop_recording(completed=True)
        self.status_changed.emit("已断开")


class PlaybackThread(QThread):
    frame_ready = Signal(QImage)
    position_changed = Signal(float, float)
    playback_error = Signal(str)
    finished_normally = Signal()

    def __init__(self, path: Path) -> None:
        super().__init__()
        self.path = path
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def set_paused(self, paused: bool) -> None:
        if paused:
            self._pause_event.set()
        else:
            self._pause_event.clear()

    def run(self) -> None:
        container = None
        try:
            container = av.open(str(self.path), mode="r")
            stream = container.streams.video[0]
            stream.thread_type = "AUTO"
            duration = 0.0
            if container.duration is not None:
                duration = float(container.duration / av.time_base)

            first_time: Optional[float] = None
            wall_start: Optional[float] = None
            paused_started: Optional[float] = None

            for frame in container.decode(stream):
                if self._stop_event.is_set():
                    break

                while self._pause_event.is_set() and not self._stop_event.is_set():
                    if paused_started is None:
                        paused_started = time.monotonic()
                    time.sleep(0.03)
                if self._stop_event.is_set():
                    break
                if paused_started is not None and wall_start is not None:
                    wall_start += time.monotonic() - paused_started
                    paused_started = None

                frame_time = float(frame.time or 0.0)
                if first_time is None:
                    first_time = frame_time
                    wall_start = time.monotonic()
                position = max(0.0, frame_time - first_time)

                if wall_start is not None:
                    target = wall_start + position
                    while not self._stop_event.is_set():
                        remain = target - time.monotonic()
                        if remain <= 0:
                            break
                        time.sleep(min(0.01, remain))

                array = frame.to_ndarray(format="rgb24")
                height, width, _ = array.shape
                image = QImage(
                    array.data,
                    width,
                    height,
                    array.strides[0],
                    QImage.Format.Format_RGB888,
                ).copy()
                self.frame_ready.emit(image)
                self.position_changed.emit(position, duration)

            if not self._stop_event.is_set():
                self.finished_normally.emit()
        except Exception as exc:
            self.playback_error.emit(str(exc))
        finally:
            if container is not None:
                try:
                    container.close()
                except Exception:
                    pass


class PlaybackDialog(QDialog):
    def __init__(self, path: Path, parent=None) -> None:
        super().__init__(parent)
        self.path = path
        self.thread: Optional[PlaybackThread] = None
        self.paused = False

        self.setWindowTitle(f"录像回放 - {path.name}")
        self.resize(960, 650)

        self.video_label = QLabel("正在打开录像…")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(640, 360)
        self.video_label.setStyleSheet("background:#111; color:#bbb;")
        self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        self.progress.setTextVisible(True)
        self.progress.setFormat("00:00:00 / 00:00:00")

        self.pause_btn = QPushButton("暂停")
        self.restart_btn = QPushButton("重新播放")
        self.close_btn = QPushButton("关闭")

        controls = QHBoxLayout()
        controls.addWidget(self.pause_btn)
        controls.addWidget(self.restart_btn)
        controls.addStretch(1)
        controls.addWidget(self.close_btn)

        layout = QVBoxLayout(self)
        layout.addWidget(self.video_label, 1)
        layout.addWidget(self.progress)
        layout.addLayout(controls)

        self.pause_btn.clicked.connect(self.toggle_pause)
        self.restart_btn.clicked.connect(self.start_playback)
        self.close_btn.clicked.connect(self.close)
        self.start_playback()

    def start_playback(self) -> None:
        self.stop_playback()
        self.paused = False
        self.pause_btn.setText("暂停")
        self.thread = PlaybackThread(self.path)
        self.thread.frame_ready.connect(self.show_frame)
        self.thread.position_changed.connect(self.update_position)
        self.thread.playback_error.connect(self.show_error)
        self.thread.finished_normally.connect(lambda: self.pause_btn.setText("播放结束"))
        self.thread.start()

    def stop_playback(self) -> None:
        if self.thread is not None:
            self.thread.stop()
            self.thread.wait(2500)
            self.thread = None

    def toggle_pause(self) -> None:
        if self.thread is None or not self.thread.isRunning():
            return
        self.paused = not self.paused
        self.thread.set_paused(self.paused)
        self.pause_btn.setText("继续" if self.paused else "暂停")

    def show_frame(self, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(
            self.video_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video_label.setPixmap(pixmap)

    def update_position(self, position: float, duration: float) -> None:
        if duration > 0:
            self.progress.setValue(min(1000, int(position / duration * 1000)))
        self.progress.setFormat(f"{format_seconds(position)} / {format_seconds(duration)}")

    def show_error(self, message: str) -> None:
        QMessageBox.critical(self, "回放失败", message)

    def closeEvent(self, event: QCloseEvent) -> None:
        self.stop_playback()
        event.accept()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(f"{APP_TITLE}  ·  Python {sys.version_info.major}.{sys.version_info.minor}")
        self.resize(1280, 760)

        self.stream_thread: Optional[LiveStreamThread] = None
        self.last_image: Optional[QImage] = None
        self.last_record_path: Optional[Path] = None
        self.recording_started_at: Optional[float] = None
        self.record_requested = False
        self.record_dir = DEFAULT_RECORD_DIR
        self.record_dir.mkdir(parents=True, exist_ok=True)

        self._build_ui()
        self._apply_style()
        self.refresh_recordings()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_recording_timer)
        self.timer.start(200)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        title = QLabel(APP_TITLE)
        title.setObjectName("title")
        root.addWidget(title)

        connection_frame = QFrame()
        connection_layout = QHBoxLayout(connection_frame)
        self.url_edit = QLineEdit(DEFAULT_RTSP_URL)
        self.url_edit.setPlaceholderText("rtsp://MaixCAM2_IP:8554/live")
        self.connect_btn = QPushButton("连接图传")
        self.connect_btn.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(QLabel("RTSP 地址"))
        connection_layout.addWidget(self.url_edit, 1)
        connection_layout.addWidget(self.connect_btn)
        root.addWidget(connection_frame)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(splitter, 1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.video_label = QLabel("尚未连接 MaixCAM2")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumSize(720, 405)
        self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video_label.setObjectName("video")
        left_layout.addWidget(self.video_label, 1)

        status_row = QHBoxLayout()
        self.connection_status = QLabel("状态：未连接")
        self.stream_stats = QLabel("分辨率：--  FPS：--")
        self.record_status = QLabel("录像：未开始")
        status_row.addWidget(self.connection_status, 2)
        status_row.addWidget(self.stream_stats, 1)
        status_row.addWidget(self.record_status, 1)
        left_layout.addLayout(status_row)

        action_row = QHBoxLayout()
        self.test_label_edit = QLineEdit("test_01")
        self.test_label_edit.setMaximumWidth(180)
        self.record_btn = QPushButton("● 开始录像")
        self.record_btn.setObjectName("recordButton")
        self.record_btn.setEnabled(False)
        self.snapshot_btn = QPushButton("保存截图")
        self.snapshot_btn.setEnabled(False)
        self.fullscreen_btn = QPushButton("画面全屏")
        action_row.addWidget(QLabel("测试编号"))
        action_row.addWidget(self.test_label_edit)
        action_row.addWidget(self.record_btn)
        action_row.addWidget(self.snapshot_btn)
        action_row.addWidget(self.fullscreen_btn)
        action_row.addStretch(1)
        left_layout.addLayout(action_row)

        self.record_btn.clicked.connect(self.toggle_recording)
        self.snapshot_btn.clicked.connect(self.save_snapshot)
        self.fullscreen_btn.clicked.connect(self.toggle_fullscreen)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(QLabel("已保存录像（双击回放）"))
        self.record_list = QListWidget()
        self.record_list.itemDoubleClicked.connect(self.play_selected_recording)
        right_layout.addWidget(self.record_list, 1)

        folder_form = QFormLayout()
        self.folder_label = QLineEdit(str(self.record_dir))
        self.folder_label.setReadOnly(True)
        folder_form.addRow("存储目录", self.folder_label)
        right_layout.addLayout(folder_form)

        folder_buttons = QHBoxLayout()
        choose_folder_btn = QPushButton("更改目录")
        open_folder_btn = QPushButton("打开目录")
        refresh_btn = QPushButton("刷新列表")
        play_btn = QPushButton("回放选中")
        folder_buttons.addWidget(choose_folder_btn)
        folder_buttons.addWidget(open_folder_btn)
        folder_buttons.addWidget(refresh_btn)
        folder_buttons.addWidget(play_btn)
        right_layout.addLayout(folder_buttons)

        choose_folder_btn.clicked.connect(self.choose_record_dir)
        open_folder_btn.clicked.connect(self.open_record_dir)
        refresh_btn.clicked.connect(self.refresh_recordings)
        play_btn.clicked.connect(self.play_selected_recording)

        tips = QLabel(
            "比赛建议：使用独立热点；双方支持时优先 5 GHz，否则使用 2.4 GHz；"
            "开始测试前先确认画面连续，再按开始录像。"
        )
        tips.setWordWrap(True)
        tips.setObjectName("tips")
        right_layout.addWidget(tips)

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 2)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow { background: #f3f5f7; }
            QLabel#title { font-size: 24px; font-weight: 700; padding: 4px; }
            QLabel#video { background: #101214; color: #aab0b6; border: 2px solid #30363d; }
            QLabel#tips { color: #555; background: #fff8dc; padding: 8px; border-radius: 5px; }
            QPushButton { min-height: 32px; padding: 2px 12px; }
            QPushButton#recordButton { font-weight: 700; }
            QLineEdit { min-height: 30px; }
            QListWidget { background: white; }
            """
        )

    def toggle_connection(self) -> None:
        if self.stream_thread is not None and self.stream_thread.isRunning():
            self.disconnect_stream()
        else:
            self.connect_stream()

    def connect_stream(self) -> None:
        url = self.url_edit.text().strip()
        if not url.startswith("rtsp://"):
            QMessageBox.warning(self, "地址错误", "RTSP 地址应以 rtsp:// 开头。")
            return

        self.stream_thread = LiveStreamThread(
            url=url,
            record_dir=self.record_dir,
            test_label=self.test_label_edit.text(),
        )
        self.stream_thread.frame_ready.connect(self.show_live_frame)
        self.stream_thread.status_changed.connect(self.update_connection_status)
        self.stream_thread.stats_changed.connect(self.update_stats)
        self.stream_thread.recording_changed.connect(self.on_recording_changed)
        self.stream_thread.finished.connect(self.on_stream_finished)
        self.stream_thread.start()

        self.connect_btn.setText("断开图传")
        self.url_edit.setEnabled(False)
        self.record_btn.setEnabled(True)
        self.snapshot_btn.setEnabled(True)

    def disconnect_stream(self) -> None:
        if self.stream_thread is None:
            return
        self.record_btn.setEnabled(False)
        self.stream_thread.set_recording(False, self.test_label_edit.text())
        self.stream_thread.request_stop()
        if not self.stream_thread.wait(4500):
            QMessageBox.warning(self, "提示", "流线程仍在退出，请稍后再关闭程序。")
        self.stream_thread = None
        self.on_stream_finished()

    def on_stream_finished(self) -> None:
        self.connect_btn.setText("连接图传")
        self.url_edit.setEnabled(True)
        self.record_btn.setEnabled(False)
        self.record_btn.setText("● 开始录像")
        self.snapshot_btn.setEnabled(False)
        self.recording_started_at = None
        self.record_requested = False
        self.connection_status.setText("状态：已断开")
        self.record_status.setText("录像：未开始")

    def update_connection_status(self, message: str) -> None:
        self.connection_status.setText(f"状态：{message}")

    def update_stats(self, stats: StreamStats) -> None:
        fps_text = f"{stats.fps:.1f}" if stats.fps > 0 else "--"
        self.stream_stats.setText(f"分辨率：{stats.width}×{stats.height}  FPS：{fps_text}")

    def show_live_frame(self, image: QImage) -> None:
        self.last_image = image
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(
            self.video_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.video_label.setPixmap(pixmap)

    def toggle_recording(self) -> None:
        if self.stream_thread is None or not self.stream_thread.isRunning():
            return
        if self.record_requested:
            self.record_requested = False
            self.stream_thread.set_recording(False, self.test_label_edit.text())
            if self.recording_started_at is None:
                self.record_btn.setText("● 开始录像")
                self.record_status.setText("录像：已取消")
            else:
                self.record_btn.setEnabled(False)
                self.record_status.setText("录像：正在封装文件…")
        else:
            self.record_requested = True
            self.stream_thread.set_recording(True, self.test_label_edit.text())
            self.record_btn.setText("■ 停止录像")
            self.record_status.setText("录像：等待关键帧…")

    def on_recording_changed(self, state: str, message: str) -> None:
        if state == "waiting":
            self.record_status.setText("录像：等待关键帧…")
        elif state == "recording":
            self.record_requested = True
            self.last_record_path = Path(message)
            self.recording_started_at = time.monotonic()
            self.record_btn.setEnabled(True)
            self.record_btn.setText("■ 停止录像")
            self.record_status.setText("录像：REC 00:00:00")
        elif state in {"saved", "interrupted"}:
            self.record_requested = False
            self.recording_started_at = None
            self.record_btn.setEnabled(self.stream_thread is not None)
            self.record_btn.setText("● 开始录像")
            prefix = "已保存" if state == "saved" else "中断后已封装"
            self.record_status.setText(f"录像：{prefix}")
            self.refresh_recordings()
        elif state == "error":
            self.record_requested = False
            self.recording_started_at = None
            self.record_btn.setEnabled(self.stream_thread is not None)
            self.record_btn.setText("● 开始录像")
            self.record_status.setText("录像：失败")
            QMessageBox.critical(self, "录像错误", message)

    def update_recording_timer(self) -> None:
        if self.recording_started_at is not None:
            elapsed = time.monotonic() - self.recording_started_at
            self.record_status.setText(f"录像：REC {format_seconds(elapsed)}")

    def save_snapshot(self) -> None:
        if self.last_image is None:
            return
        self.record_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        label = safe_file_part(self.test_label_edit.text())
        path = self.record_dir / f"{timestamp}_{label}_snapshot.jpg"
        if self.last_image.save(str(path), "JPG", 95):
            self.statusBar().showMessage(f"截图已保存：{path}", 5000)
        else:
            QMessageBox.critical(self, "截图失败", f"无法写入：{path}")

    def choose_record_dir(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "选择录像存储目录", str(self.record_dir))
        if not selected:
            return
        self.record_dir = Path(selected)
        self.record_dir.mkdir(parents=True, exist_ok=True)
        self.folder_label.setText(str(self.record_dir))
        if self.stream_thread is not None:
            self.stream_thread.record_dir = self.record_dir
        self.refresh_recordings()

    def refresh_recordings(self) -> None:
        self.record_dir.mkdir(parents=True, exist_ok=True)
        self.record_list.clear()
        files = sorted(self.record_dir.glob("*.mkv"), key=lambda p: p.stat().st_mtime, reverse=True)
        for path in files:
            size_mb = path.stat().st_size / (1024 * 1024)
            self.record_list.addItem(f"{path.name}    {size_mb:.1f} MB")
            self.record_list.item(self.record_list.count() - 1).setData(Qt.ItemDataRole.UserRole, str(path))

    def selected_recording_path(self) -> Optional[Path]:
        item = self.record_list.currentItem()
        if item is None:
            return None
        value = item.data(Qt.ItemDataRole.UserRole)
        return Path(value) if value else None

    def play_selected_recording(self, *_args) -> None:
        path = self.selected_recording_path()
        if path is None:
            QMessageBox.information(self, "选择录像", "请先在右侧列表中选择一个录像。")
            return
        if not path.exists():
            QMessageBox.warning(self, "文件不存在", str(path))
            self.refresh_recordings()
            return
        dialog = PlaybackDialog(path, self)
        dialog.exec()

    def open_record_dir(self) -> None:
        self.record_dir.mkdir(parents=True, exist_ok=True)
        try:
            if sys.platform.startswith("win"):
                os.startfile(str(self.record_dir))  # type: ignore[attr-defined]
            elif sys.platform == "darwin":
                os.system(f'open "{self.record_dir}"')
            else:
                os.system(f'xdg-open "{self.record_dir}"')
        except Exception as exc:
            QMessageBox.warning(self, "无法打开目录", str(exc))

    def toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
            self.fullscreen_btn.setText("画面全屏")
        else:
            self.showFullScreen()
            self.fullscreen_btn.setText("退出全屏")

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.stream_thread is not None and self.stream_thread.isRunning():
            self.stream_thread.set_recording(False, self.test_label_edit.text())
            self.stream_thread.request_stop()
            self.stream_thread.wait(4500)
        event.accept()


def main() -> int:
    av.logging.set_level(av.logging.ERROR)
    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

"""Dual-Stream Visual Observer.

Captures two visual streams:
  1. Stream 1 (Robot Perspective): Reads on-board RGB camera (/camera/image_raw),
     computes optical degradation metrics (motion blur, Gaussian noise, fog contrast attenuation).
  2. Stream 2 (Desktop GUI Windows): Uses mss + xdotool/Win32 to capture native desktop
     windows for Gazebo (Sim Time, RTF, 3D viewport) and RViz2 (Costmaps, LaserScan, Planned Path).

Encodes frames as base64 JPEG strings ready for multimodal reasoning models.
Pure NumPy decoding for raw ROS camera images (no cv_bridge required).
"""

from __future__ import annotations

import argparse
import base64
import io
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence, Tuple

import numpy as np
from PIL import Image

__all__ = [
    "VisionResult",
    "compute_optical_metrics",
    "encode_jpeg_base64",
    "capture_desktop_window",
    "capture_gazebo_window",
    "capture_rviz_window",
    "read_robot_camera",
    "generate_synthetic_frame",
    "CameraSampler",
    "get_camera_sampler",
]


@dataclass
class VisionResult:
    """Standard envelope for visual perception frames and metadata."""

    ok: bool
    stream: str  # "desktop_window" or "robot_camera"
    target: str  # "gazebo", "rviz", "screen", or topic name
    width: int = 0
    height: int = 0
    mime: str = "image/jpeg"
    jpeg_base64: str = ""
    metrics: dict[str, float] = field(default_factory=dict)
    latency_ms: float = 0.0
    message: str = ""
    save_path: Optional[str] = None
    fallback: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "stream": self.stream,
            "target": self.target,
            "width": self.width,
            "height": self.height,
            "mime": self.mime,
            "jpeg_base64": self.jpeg_base64,
            "metrics": self.metrics,
            "latency_ms": round(self.latency_ms, 2),
            "message": self.message,
            "save_path": self.save_path,
            "fallback": self.fallback,
        }


# --------------------------------------------------------------------------- Optical Degradation Metrics
def _to_grayscale_f32(img_arr: np.ndarray) -> np.ndarray:
    """Convert RGB/BGR/L array to 2D grayscale float32 in [0, 255]."""
    if img_arr.ndim == 2:
        return img_arr.astype(np.float32)
    # ITU-R 601-2 luma transform
    return (0.299 * img_arr[:, :, 0] + 0.587 * img_arr[:, :, 1] + 0.114 * img_arr[:, :, 2]).astype(
        np.float32
    )


def compute_optical_metrics(img_arr: np.ndarray) -> dict[str, float]:
    """Calculate optical degradation metrics: blur (variance of Laplacian), noise, contrast, and brightness."""
    gray = _to_grayscale_f32(img_arr)
    h, w = gray.shape

    # 1. Brightness: mean luma in [0, 255]
    brightness = float(np.mean(gray))

    # 2. RMS Contrast: stddev / mean (or stddev) — attenuation by volumetric fog drops this
    rms_contrast = float(np.std(gray))

    # 3. Motion Blur: variance of 4-connected discrete Laplacian
    # L(x, y) = gray[y+1, x] + gray[y-1, x] + gray[y, x+1] + gray[y, x-1] - 4*gray[y, x]
    if h > 2 and w > 2:
        laplacian = (
            gray[2:, 1:-1]
            + gray[:-2, 1:-1]
            + gray[1:-1, 2:]
            + gray[1:-1, :-2]
            - 4.0 * gray[1:-1, 1:-1]
        )
        blur_score = float(np.var(laplacian))
    else:
        blur_score = 0.0

    # 4. Noise estimate: high-pass residual standard deviation
    if h > 2 and w > 2:
        # High pass kernel [[1, -2, 1], [-2, 4, -2], [1, -2, 1]] / 16
        high_pass = (
            (gray[2:, 2:] + gray[:-2, :-2] + gray[2:, :-2] + gray[:-2, 2:])
            - 2.0 * (gray[2:, 1:-1] + gray[:-2, 1:-1] + gray[1:-1, 2:] + gray[1:-1, :-2])
            + 4.0 * gray[1:-1, 1:-1]
        ) / 16.0
        noise_score = float(np.std(high_pass))
    else:
        noise_score = 0.0

    return {
        "brightness": round(brightness, 2),
        "contrast_rms": round(rms_contrast, 2),
        "blur_laplacian_var": round(blur_score, 2),
        "noise_stddev": round(noise_score, 2),
    }


def encode_jpeg_base64(img: Image.Image, quality: int = 80, max_dimension: int = 1280) -> str:
    """Downscale if larger than max_dimension, encode to JPEG, and return base64 string."""
    w, h = img.size
    if max(w, h) > max_dimension:
        scale = max_dimension / max(w, h)
        img = img.resize((int(w * scale), int(h * scale)), Image.Resampling.BILINEAR)

    # Ensure RGB
    if img.mode != "RGB":
        img = img.convert("RGB")

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode("ascii")


# --------------------------------------------------------------------------- Desktop Window Capture
def _find_window_rect_x11(title_pattern: str) -> Optional[Tuple[int, int, int, int]]:
    """Query xdotool on Linux/X11 to locate window coordinates (x, y, w, h)."""
    if not shutil.which("xdotool"):
        return None
    try:
        cmd = ["xdotool", "search", "--onlyvisible", "--name", title_pattern]
        out = subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL).strip()
        win_ids = [w.strip() for w in out.splitlines() if w.strip()]
        if not win_ids:
            return None
        win_id = win_ids[0]
        geom_out = subprocess.check_output(
            ["xdotool", "getwindowgeometry", "--shell", win_id],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        geom = {}
        for line in geom_out.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                geom[k.strip()] = int(v.strip())
        return geom["X"], geom["Y"], geom["WIDTH"], geom["HEIGHT"]
    except Exception:
        return None


def _find_window_rect_windows(title_pattern: str) -> Optional[Tuple[int, int, int, int]]:
    """Locate window coordinates on Windows using Win32 API if available."""
    try:
        import win32gui

        matched_hwnd = None

        def enum_windows_callback(hwnd, extra):
            nonlocal matched_hwnd
            if win32gui.IsWindowVisible(hwnd):
                title = win32gui.GetWindowText(hwnd)
                if title_pattern.lower() in title.lower():
                    matched_hwnd = hwnd

        win32gui.EnumWindows(enum_windows_callback, None)
        if matched_hwnd:
            rect = win32gui.GetWindowRect(matched_hwnd)
            x, y, x2, y2 = rect
            return x, y, max(1, x2 - x), max(1, y2 - y)
    except Exception:
        pass
    return None


def _generate_synthetic_desktop(title: str, width: int = 1280, height: int = 720) -> Image.Image:
    """Generate a mock Gazebo/RViz interface screen when desktop GDI is unavailable."""
    arr = np.full((height, width, 3), 30, dtype=np.uint8)
    # Title bar
    arr[:35, :] = [45, 45, 48]
    # Sidebar
    arr[35:, :240] = [38, 38, 42]
    # Viewport center grid
    arr[35:, 240:] = [24, 24, 26]
    img = Image.fromarray(arr)
    return img


def capture_desktop_window(
    title_pattern: str = "Gazebo",
    save_path: Optional[str | Path] = None,
) -> VisionResult:
    """Capture a specific desktop application window, falling back to full desktop or synthetic frame."""
    t0 = time.perf_counter()
    import mss

    rect: Optional[Tuple[int, int, int, int]] = None
    fallback: Optional[str] = None
    img: Optional[Image.Image] = None

    if sys.platform.startswith("linux"):
        rect = _find_window_rect_x11(title_pattern)
    elif sys.platform.startswith("win"):
        rect = _find_window_rect_windows(title_pattern)

    try:
        # Use mss.MSS or mss.mss
        mss_cls = getattr(mss, "MSS", None) or mss.mss
        with mss_cls() as sct:
            if rect is not None:
                rx, ry, rw, rh = rect
                monitor = {
                    "top": max(0, ry),
                    "left": max(0, rx),
                    "width": max(10, rw),
                    "height": max(10, rh),
                }
                sct_img = sct.grab(monitor)
                msg = f"Captured '{title_pattern}' window ({rw}x{rh})"
            else:
                monitor = sct.monitors[1] if len(sct.monitors) > 1 else sct.monitors[0]
                sct_img = sct.grab(monitor)
                fallback = "full_screen"
                msg = f"Window '{title_pattern}' not found; captured full desktop ({monitor['width']}x{monitor['height']})"
            img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX")
    except Exception as exc:
        fallback = "synthetic_desktop"
        msg = f"Direct screen capture unavailable ({exc}); generated mock '{title_pattern}' desktop frame"
        img = _generate_synthetic_desktop(title_pattern)

    arr = np.asarray(img)
    metrics = compute_optical_metrics(arr)
    b64_str = encode_jpeg_base64(img)

    if save_path:
        out_path = Path(save_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        img.save(out_path, format="JPEG", quality=85)
        saved = str(out_path)
    else:
        saved = None

    latency = (time.perf_counter() - t0) * 1000.0
    return VisionResult(
        ok=True,
        stream="desktop_window",
        target=title_pattern.lower(),
        width=img.width,
        height=img.height,
        jpeg_base64=b64_str,
        metrics=metrics,
        latency_ms=latency,
        message=msg,
        save_path=saved,
        fallback=fallback,
    )


async def capture_gazebo_window(save_path: Optional[str | Path] = None) -> dict[str, Any]:
    """Capture active Gazebo desktop simulation window."""
    return capture_desktop_window("Gazebo", save_path).to_dict()


async def capture_rviz_window(save_path: Optional[str | Path] = None) -> dict[str, Any]:
    """Capture active RViz2 desktop GUI window."""
    return capture_desktop_window("RViz", save_path).to_dict()


# --------------------------------------------------------------------------- Robot Camera Stream
def generate_synthetic_frame(
    width: int = 640,
    height: int = 480,
    motion_blur: int = 0,
    fog_density: float = 0.0,
    noise_std: float = 0.0,
) -> Image.Image:
    """Generate a realistic test camera frame for offline testing."""
    # Create checkerboard floor with a horizon and colored block obstacle
    img_arr = np.zeros((height, width, 3), dtype=np.float32)
    horizon = int(height * 0.45)
    # Sky
    sky_luma = max(20.0, 180.0 * (1.0 - fog_density))
    img_arr[:horizon, :, :] = [sky_luma * 0.8, sky_luma * 0.9, sky_luma]
    # Ground tiles
    for y in range(horizon, height):
        for x in range(width):
            check = ((x // 30) + (y // 30)) % 2
            col = 140.0 if check else 80.0
            img_arr[y, x, :] = [col * 0.8, col, col * 0.8]

    # Red/yellow obstacle in front
    ox1, ox2 = int(width * 0.4), int(width * 0.6)
    oy1, oy2 = int(height * 0.5), int(height * 0.8)
    img_arr[oy1:oy2, ox1:ox2, :] = [210.0, 140.0, 30.0]

    # Apply fog contrast attenuation
    if fog_density > 0:
        fog_color = 200.0
        alpha = min(0.9, fog_density * 3.0)
        img_arr = (1.0 - alpha) * img_arr + alpha * fog_color

    # Add Gaussian noise
    if noise_std > 0:
        noise = np.random.normal(0, noise_std * 255.0, img_arr.shape)
        img_arr = np.clip(img_arr + noise, 0.0, 255.0)

    # Convert to PIL
    pil_img = Image.fromarray(np.uint8(np.clip(img_arr, 0.0, 255.0)))

    # Apply motion blur if requested
    if motion_blur > 1:
        from PIL import ImageFilter

        pil_img = pil_img.filter(ImageFilter.BoxBlur(motion_blur))

    return pil_img


class CameraSampler:
    """Subscribes to ROS 2 camera topic or serves synthetic test frames when ROS is absent."""

    def __init__(self, topic: str = "/camera/image_raw") -> None:
        self.topic = topic
        self.last_frame: Optional[Image.Image] = None
        self.last_timestamp: float = 0.0
        self.fps: float = 0.0
        self._subscription = None
        self._init_ros()

    def _init_ros(self) -> None:
        try:
            import rclpy  # type: ignore[import-not-found]
            from sensor_msgs.msg import Image as RosImage  # type: ignore[import-not-found]

            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("lang2sim_vision_sampler")

            def _callback(msg: RosImage):
                t_now = time.time()
                dt = t_now - self.last_timestamp
                if dt > 0:
                    inst_fps = 1.0 / dt
                    self.fps = 0.8 * self.fps + 0.2 * inst_fps if self.fps > 0 else inst_fps
                self.last_timestamp = t_now

                # Pure numpy decoding
                w, h = msg.width, msg.height
                enc = msg.encoding.lower()
                data = np.frombuffer(msg.data, dtype=np.uint8)
                if enc in ("rgb8", "bgr8"):
                    arr = data.reshape((h, w, 3))
                    if enc == "bgr8":
                        arr = arr[:, :, ::-1]
                    self.last_frame = Image.fromarray(arr)
                elif enc == "mono8":
                    arr = data.reshape((h, w))
                    self.last_frame = Image.fromarray(arr).convert("RGB")

            self._subscription = node.create_subscription(RosImage, self.topic, _callback, 10)
        except Exception:
            # ROS 2 not installed or daemon not running — lazy fallback
            pass

    def sample(self, save_path: Optional[str | Path] = None) -> VisionResult:
        t0 = time.perf_counter()
        if self.last_frame is not None:
            img = self.last_frame
            msg = f"Sampled camera topic '{self.topic}' ({self.fps:.1f} FPS)"
            fallback = None
        else:
            # Fallback to realistic synthetic frame
            img = generate_synthetic_frame()
            msg = (
                f"ROS topic '{self.topic}' not active; generated synthetic camera perception frame"
            )
            fallback = "synthetic_camera"

        arr = np.asarray(img)
        metrics = compute_optical_metrics(arr)
        b64_str = encode_jpeg_base64(img)

        if save_path:
            out_path = Path(save_path)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            img.save(out_path, format="JPEG", quality=85)
            saved = str(out_path)
        else:
            saved = None

        latency = (time.perf_counter() - t0) * 1000.0
        return VisionResult(
            ok=True,
            stream="robot_camera",
            target=self.topic,
            width=img.width,
            height=img.height,
            jpeg_base64=b64_str,
            metrics=metrics,
            latency_ms=latency,
            message=msg,
            save_path=saved,
            fallback=fallback,
        )


_GLOBAL_SAMPLER: Optional[CameraSampler] = None


def get_camera_sampler(topic: str = "/camera/image_raw") -> CameraSampler:
    global _GLOBAL_SAMPLER
    if _GLOBAL_SAMPLER is None or _GLOBAL_SAMPLER.topic != topic:
        _GLOBAL_SAMPLER = CameraSampler(topic)
    return _GLOBAL_SAMPLER


async def read_robot_camera(
    topic: str = "/camera/image_raw",
    save_path: Optional[str | Path] = None,
) -> dict[str, Any]:
    """Read latest frame from on-board robot camera topic."""
    return get_camera_sampler(topic).sample(save_path).to_dict()


# --------------------------------------------------------------------------- CLI
def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Lang2Sim Dual-Stream Visual Observer")
    parser.add_argument(
        "--target",
        choices=["screen", "gazebo", "rviz", "camera"],
        default="screen",
        help="Capture target (default: screen)",
    )
    parser.add_argument("--out", help="Save captured frame to JPEG path")
    parser.add_argument(
        "--print-b64-len", action="store_true", help="Print length of base64 JPEG payload"
    )
    parser.add_argument("--topic", default="/camera/image_raw", help="Robot camera ROS topic")
    args = parser.parse_args(argv)

    if args.target == "camera":
        res = get_camera_sampler(args.topic).sample(args.out)
    elif args.target in ("gazebo", "rviz"):
        res = capture_desktop_window(args.target.capitalize(), args.out)
    else:
        res = capture_desktop_window("Screen", args.out)

    print(f"[{res.stream.upper()}] Status: {'OK' if res.ok else 'FAIL'}")
    print(f"Target: {res.target} ({res.width}x{res.height})")
    print(f"Message: {res.message}")
    print(f"Metrics: {res.metrics}")
    print(f"Latency: {res.latency_ms:.2f}ms")
    if res.save_path:
        print(f"Saved: {res.save_path}")
    if args.print_b64_len:
        print(f"Base64 length: {len(res.jpeg_base64)} characters")
    return 0


if __name__ == "__main__":
    import warnings

    warnings.filterwarnings("ignore", category=RuntimeWarning, message=".*found in sys.modules.*")
    sys.exit(main())

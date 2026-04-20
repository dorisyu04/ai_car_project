# -*- coding: utf-8 -*-

import time
import threading

import cv2
from flask import Flask, Response, jsonify, render_template, request
from picamera2 import Picamera2
from libcamera import Transform

from lane_detection import detect_white_lane
from motor_control import RobotControl


app = Flask(__name__)

robot = RobotControl()
control_lock = threading.Lock()
frame_lock = threading.Lock()

latest_ai_msg = "MANUAL MODE"
latest_car_state = "stop"
latest_cam_state = "center"

auto_mode = False

BASE_SPEED = 20
TURN_SPEED = 16
SEARCH_TURN_SPEED = 10
SEARCH_FORWARD_SPEED = 9

CAMERA_CENTER_PAN = 70
CAMERA_CENTER_TILT = 10

PAN_STEP = 5
TILT_STEP = 5

PAN_MIN = 35
PAN_MAX = 145
TILT_MIN = 0
TILT_MAX = 145

current_pan = CAMERA_CENTER_PAN
current_tilt = CAMERA_CENTER_TILT

latest_raw_frame = None
latest_debug_frame = None
latest_lane_result = {}

lost_count = 0
smoothed_error = 0
last_steer = "forward"

search_direction = "left"
search_phase = 0
search_phase_count = 0

FRAME_WIDTH = 320
FRAME_HEIGHT = 240


picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
    transform=Transform(hflip=True, vflip=True)
)
picam2.configure(config)

try:
    picam2.set_controls({
        "AeEnable": True,
        "AwbEnable": True,
        "Brightness": -0.08,
        "Contrast": 1.30,
        "Saturation": 1.0,
        "Sharpness": 1.0
    })
except Exception:
    pass

picam2.start()
time.sleep(0.5)


def safe_call(obj, method_names, *args, **kwargs):
    for name in method_names:
        fn = getattr(obj, name, None)
        if callable(fn):
            return fn(*args, **kwargs)
    raise AttributeError(f"Method not found: {method_names}")


def car_forward(speed=BASE_SPEED):
    try:
        safe_call(robot, ["forward"], speed)
    except Exception:
        try:
            safe_call(robot, ["move"], "forward", speed)
        except Exception:
            safe_call(robot, ["forward"])


def car_backward(speed=BASE_SPEED):
    try:
        safe_call(robot, ["backward"], speed)
    except Exception:
        try:
            safe_call(robot, ["move"], "backward", speed)
        except Exception:
            safe_call(robot, ["backward"])


def car_left(speed=TURN_SPEED):
    try:
        safe_call(robot, ["left"], speed)
    except Exception:
        try:
            safe_call(robot, ["turn_left"], speed)
        except Exception:
            try:
                safe_call(robot, ["move"], "left", speed)
            except Exception:
                safe_call(robot, ["left"])


def car_right(speed=TURN_SPEED):
    try:
        safe_call(robot, ["right"], speed)
    except Exception:
        try:
            safe_call(robot, ["turn_right"], speed)
        except Exception:
            try:
                safe_call(robot, ["move"], "right", speed)
            except Exception:
                safe_call(robot, ["right"])


def car_stop():
    safe_call(robot, ["stop"])


def set_camera_angles(pan, tilt):
    global current_pan, current_tilt

    current_pan = max(PAN_MIN, min(PAN_MAX, pan))
    current_tilt = max(TILT_MIN, min(TILT_MAX, tilt))

    try:
        safe_call(robot, ["set_camera", "set_cam", "set_camera_angle"], current_pan, current_tilt)
        return
    except Exception:
        pass

    try:
        safe_call(robot, ["set_servo_angle"], 9, current_pan)
        safe_call(robot, ["set_servo_angle"], 10, current_tilt)
    except Exception:
        pass


def camera_center():
    global latest_cam_state
    set_camera_angles(CAMERA_CENTER_PAN, CAMERA_CENTER_TILT)
    latest_cam_state = "center"


def camera_left():
    global latest_cam_state
    set_camera_angles(current_pan - PAN_STEP, current_tilt)
    latest_cam_state = "cam_left"


def camera_right():
    global latest_cam_state
    set_camera_angles(current_pan + PAN_STEP, current_tilt)
    latest_cam_state = "cam_right"


def camera_up():
    global latest_cam_state
    set_camera_angles(current_pan, current_tilt - TILT_STEP)
    latest_cam_state = "cam_up"


def camera_down():
    global latest_cam_state
    set_camera_angles(current_pan, current_tilt + TILT_STEP)
    latest_cam_state = "cam_down"


try:
    camera_center()
except Exception:
    pass


def normalize_lane_result(result, fallback_frame=None):
    if not isinstance(result, dict):
        return {
            "found": False,
            "mode": "none",
            "error": 0,
            "target_x": None,
            "debug_frame": fallback_frame.copy() if fallback_frame is not None else None
        }

    normalized = dict(result)
    normalized.setdefault("found", False)
    normalized.setdefault("mode", "none")
    normalized.setdefault("error", 0)
    normalized.setdefault("target_x", None)

    frame = normalized.get("debug_frame")
    if frame is None and fallback_frame is not None:
        normalized["debug_frame"] = fallback_frame.copy()

    return normalized


def reset_search_state():
    global search_direction, search_phase, search_phase_count
    search_direction = "left"
    search_phase = 0
    search_phase_count = 0


def auto_drive_with_line(line_result):
    global latest_ai_msg, latest_car_state
    global lost_count, smoothed_error, last_steer
    global search_direction, search_phase, search_phase_count

    found = line_result.get("found", False)
    raw_error = int(line_result.get("error", 0))
    mode = line_result.get("mode", "none")
    target_x = line_result.get("target_x", None)

    if found:
        lost_count = 0
        reset_search_state()

        smoothed_error = int(0.78 * smoothed_error + 0.22 * raw_error)
        error = smoothed_error

        latest_ai_msg = f"AUTO {mode} E={error} RAW={raw_error} TX={target_x}"

        # --- two-line mode (old logic) ---
        if mode == "both":
            if error <= -60:
                car_left(18)
                latest_car_state = "auto_left_strong"
                last_steer = "left"
            elif error >= 60:
                car_right(18)
                latest_car_state = "auto_right_strong"
                last_steer = "right"
            elif error <= -28:
                car_left(14)
                latest_car_state = "auto_left"
                last_steer = "left"
            elif error >= 28:
                car_right(14)
                latest_car_state = "auto_right"
                last_steer = "right"
            else:
                car_forward(17)
                latest_car_state = "auto_forward"
                last_steer = "forward"
            return

        # --- single-line old modes (kept for compatibility) ---
        if mode == "left_only":
            if error <= -55:
                car_left(14)
                latest_car_state = "left_only_adjust_left"
                last_steer = "left"
            elif error >= 38:
                car_right(13)
                latest_car_state = "left_only_adjust_right"
                last_steer = "right"
            else:
                car_forward(15)
                latest_car_state = "left_only_forward"
                last_steer = "forward"
            return

        if mode == "right_only":
            if error >= 55:
                car_right(14)
                latest_car_state = "right_only_adjust_right"
                last_steer = "right"
            elif error <= -38:
                car_left(13)
                latest_car_state = "right_only_adjust_left"
                last_steer = "left"
            else:
                car_forward(15)
                latest_car_state = "right_only_forward"
                last_steer = "forward"
            return

        # --- single-line new modes ---
        # error = line_cx - LANE_TARGET_CX
        # error < 0 -> line drifted LEFT  -> steer LEFT
        # error > 0 -> line drifted RIGHT -> steer RIGHT
        # error = 0 -> on track -> go straight
        if mode in ("line_left", "line_right", "straight"):
            if error <= -15:
                car_left(16)
                latest_car_state = "line_left_strong"
                last_steer = "left"
            elif error >= 15:
                car_right(16)
                latest_car_state = "line_right_strong"
                last_steer = "right"
            elif error <= -15:
                car_left(13)
                latest_car_state = "line_left_soft"
                last_steer = "left"
            elif error >= 15:
                car_right(13)
                latest_car_state = "line_right_soft"
                last_steer = "right"
            else:
                car_forward(15)
                latest_car_state = "line_straight"
                last_steer = "forward"
            return

        # fallback
        car_forward(15)
        latest_car_state = "auto_forward"
        last_steer = "forward"
        return

    lost_count += 1
    smoothed_error = int(smoothed_error * 0.55)

    # Step 1: short stop
    if lost_count <= 2:
        car_stop()
        latest_ai_msg = "AUTO LOST LINE STOP"
        latest_car_state = "lost_stop"
        return

    # Step 2: keep last direction gently
    if lost_count <= 5:
        if last_steer == "left":
            car_left(SEARCH_TURN_SPEED)
            latest_ai_msg = "AUTO MEMORY SEARCH LEFT"
            latest_car_state = "memory_left"
        elif last_steer == "right":
            car_right(SEARCH_TURN_SPEED)
            latest_ai_msg = "AUTO MEMORY SEARCH RIGHT"
            latest_car_state = "memory_right"
        else:
            car_forward(SEARCH_FORWARD_SPEED)
            latest_ai_msg = "AUTO MEMORY SEARCH FORWARD"
            latest_car_state = "memory_forward"
        return

    # Step 3: slow left-right sweep
    search_phase_count += 1

    if search_phase == 0:
        if search_direction == "left":
            car_left(SEARCH_TURN_SPEED)
            latest_ai_msg = "AUTO SEARCH LEFT"
            latest_car_state = "search_left"
        else:
            car_right(SEARCH_TURN_SPEED)
            latest_ai_msg = "AUTO SEARCH RIGHT"
            latest_car_state = "search_right"

        if search_phase_count >= 4:
            search_phase = 1
            search_phase_count = 0
        return

    if search_phase == 1:
        car_stop()
        latest_ai_msg = "AUTO SEARCH PAUSE"
        latest_car_state = "search_pause"

        if search_phase_count >= 2:
            search_phase = 0
            search_phase_count = 0
            search_direction = "right" if search_direction == "left" else "left"
        return


def capture_thread():
    global latest_raw_frame

    while True:
        try:
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            with frame_lock:
                latest_raw_frame = frame_bgr
        except Exception:
            pass

        time.sleep(0.04)


def lane_thread():
    global latest_debug_frame, latest_lane_result, latest_ai_msg

    while True:
        raw = None

        with frame_lock:
            if latest_raw_frame is not None:
                raw = latest_raw_frame.copy()

        if raw is not None:
            try:
                result = detect_white_lane(raw)
                result = normalize_lane_result(result, raw)

                with frame_lock:
                    latest_debug_frame = result["debug_frame"]
                    latest_lane_result = result

                with control_lock:
                    if auto_mode:
                        auto_drive_with_line(result)
                    else:
                        if latest_ai_msg.startswith("AUTO"):
                            latest_ai_msg = "MANUAL MODE"

            except Exception:
                with frame_lock:
                    latest_debug_frame = raw
                    latest_lane_result = {
                        "found": False,
                        "mode": "none",
                        "error": 0,
                        "target_x": None,
                        "debug_frame": raw
                    }

        time.sleep(0.10)


threading.Thread(target=capture_thread, daemon=True).start()
threading.Thread(target=lane_thread, daemon=True).start()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/control", methods=["POST"])
def control():
    global auto_mode, latest_ai_msg, latest_car_state, lost_count

    action = request.form.get("action", "").strip()

    with control_lock:
        if action == "auto_on":
            auto_mode = True
            lost_count = 0
            reset_search_state()
            latest_ai_msg = "AUTO MODE ON"
            latest_car_state = "auto_mode"
            return "OK"

        if action == "auto_off":
            auto_mode = False
            car_stop()
            latest_ai_msg = "AUTO MODE OFF"
            latest_car_state = "auto_off"
            return "OK"

        if action == "camera_left":
            camera_left()
            return "OK"

        if action == "camera_right":
            camera_right()
            return "OK"

        if action == "camera_up":
            camera_up()
            return "OK"

        if action == "camera_down":
            camera_down()
            return "OK"

        if action == "camera_center":
            camera_center()
            return "OK"

        if auto_mode:
            return "OK"

        if action == "forward":
            car_forward(BASE_SPEED)
            latest_ai_msg = "MANUAL MODE"
            latest_car_state = "forward"
        elif action == "backward":
            car_backward(BASE_SPEED)
            latest_ai_msg = "MANUAL MODE"
            latest_car_state = "backward"
        elif action == "left":
            car_left(TURN_SPEED)
            latest_ai_msg = "MANUAL MODE"
            latest_car_state = "left"
        elif action == "right":
            car_right(TURN_SPEED)
            latest_ai_msg = "MANUAL MODE"
            latest_car_state = "right"
        elif action == "stop":
            car_stop()
            latest_ai_msg = "MANUAL MODE"
            latest_car_state = "stop"

    return "OK"


@app.route("/status")
def status():
    return jsonify({
        "ai_msg": latest_ai_msg,
        "car_state": latest_car_state,
        "cam_state": latest_cam_state,
        "auto_mode": auto_mode
    })


def gen_frames():
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]

    while True:
        with frame_lock:
            frame = latest_debug_frame if latest_debug_frame is not None else latest_raw_frame

        if frame is None:
            time.sleep(0.05)
            continue

        ret, buffer = cv2.imencode(".jpg", frame, encode_param)
        if not ret:
            time.sleep(0.05)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buffer.tobytes() +
            b"\r\n"
        )

        time.sleep(0.08)


@app.route("/video_feed")
def video_feed():
    return Response(
        gen_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        try:
            car_stop()
        except Exception:
            pass
        try:
            picam2.stop()
        except Exception:
            pass
"""Host-side checks for the OpenMV protocol and recording state machine."""

import ast
import pathlib
import types
import unittest


MAIN = pathlib.Path(__file__).resolve().parents[1] / "main.py"


class FakeTime:
    def __init__(self):
        self.now = 0

    def ticks_ms(self):
        return self.now

    def ticks_diff(self, a, b):
        return a - b

    def ticks_add(self, a, b):
        return a + b


class FakeUART:
    def __init__(self):
        self.rx = bytearray()
        self.sent = []
        self.busy = False

    def any(self):
        return len(self.rx)

    def read(self, n):
        result = self.rx[:n]
        del self.rx[:n]
        return result

    def txdone(self):
        return not self.busy

    def write(self, data):
        self.sent.append(bytes(data))


class FakeImageIO:
    instances = []

    def __init__(self, path, mode):
        if mode != "w":
            raise ValueError("Recording must use a file stream in write mode")
        self.path = path
        self.frames = 0
        self.closed = False
        self.syncs = 0
        self.instances.append(self)

    def write(self, img):
        self.frames += 1

    def sync(self):
        self.syncs += 1

    def close(self):
        self.closed = True


class FakeTextFile:
    instances = []

    def __init__(self, path, mode):
        self.path = path
        self.mode = mode
        self.data = []
        self.flushes = 0
        self.closed = False
        self.instances.append(self)

    def write(self, value):
        self.data.append(value)

    def flush(self):
        self.flushes += 1

    def close(self):
        self.closed = True


def load_logic():
    tree = ast.parse(MAIN.read_text(encoding="utf-8"), filename=str(MAIN))
    runtime_start = next(node.lineno for node in tree.body
                         if isinstance(node, ast.Expr)
                         and isinstance(node.value, ast.Call)
                         and isinstance(node.value.func, ast.Name)
                         and node.value.func.id == "init_sensor")
    nodes = [node for node in tree.body
             if node.lineno < runtime_start
             and isinstance(node, (ast.Assign, ast.FunctionDef, ast.ClassDef))
             and not (isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == "uart"
                              for target in node.targets))]
    module = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    fake_time = FakeTime()
    fake_uart = FakeUART()
    FakeImageIO.instances = []
    FakeTextFile.instances = []
    namespace = {
        "time": fake_time,
        "uart": fake_uart,
        "open": FakeTextFile,
        "start": 0,
        "image": types.SimpleNamespace(ImageIO=FakeImageIO),
        "os": types.SimpleNamespace(stat=lambda path: (_ for _ in ()).throw(OSError())),
        "sensor": types.SimpleNamespace(QVGA=1, RGB565=2),
    }
    exec(compile(module, str(MAIN), "exec"), namespace)
    return namespace, fake_time, fake_uart


def frame(ns, pitch, yaw, roll, state):
    b = bytearray(ns["TELEM_FRAME_LEN"])
    b[0] = ns["TELEM_SOF"]
    for index, angle in ((7, pitch), (9, yaw), (11, roll)):
        raw = int(angle * 100) & 0xFFFF
        b[index] = raw >> 8
        b[index + 1] = raw & 0xFF
    b[19] = ns["MODE_BOTH"]
    b[20] = state
    b[21] = ns["F_IMU_READY"] | ns["F_HANDSHAKE_DONE"]
    b[ns["T_EOF_INDEX"]] = ns["TELEM_EOF"]
    b[ns["T_CRC_INDEX"]] = ns["crc8_lsb"](b, 0, ns["T_CRC_INDEX"])
    return b


class MainLogicTest(unittest.TestCase):
    def test_target_frames_and_busy_uart(self):
        ns, _, uart = load_logic()
        ns["send_target"](-2, 257)
        ns["send_no_target"]()
        self.assertEqual(uart.sent[0][:7],
                         bytes([0xAA, 0xFF, 0xFE, 0x01, 0x01, 1, 0xFF]))
        self.assertEqual(uart.sent[1][:7],
                         bytes([0xAA, 0, 0, 0, 0, 0, 0xFF]))
        for sent in uart.sent:
            self.assertEqual(sent[7], ns["crc8_lsb"](sent, 0, 7))
        uart.busy = True
        ns["send_no_target"]()
        self.assertEqual(len(uart.sent), 2)
        self.assertEqual(ns["TX_SKIP_COUNT"], 1)

    def test_tracking_roi_stays_inside_frame(self):
        ns, _, _ = load_logic()
        blob = ns["BlobData"]
        self.assertEqual(ns["build_found_roi"](blob(0, 0, 8, 10)),
                         (0, 0, 48, 50))
        self.assertEqual(ns["build_found_roi"](blob(310, 230, 10, 10)),
                         (270, 190, 50, 50))

    def test_largest_blob_is_selected(self):
        ns, _, _ = load_logic()
        class Frame:
            def find_blobs(self, thresholds, roi):
                self.args = (thresholds, roi)
                return [types.SimpleNamespace(rect=(2, 3, 4, 5)),
                        types.SimpleNamespace(rect=(8, 9, 10, 11))]
        img = Frame()
        found, selected = ns["find_max_blob"](img, (0, 0, 320, 240))
        self.assertEqual((found, selected.x, selected.y, selected.w, selected.h),
                         (1, 8, 9, 10, 11))
        self.assertEqual(img.args[0], ns["THRESHOLDS1"])

    def test_ypr_changes_on_each_valid_frame(self):
        ns, fake_time, uart = load_logic()
        uart.rx.extend(frame(ns, 1.25, -2.5, 3.75, ns["STATE_SERVO_READY"]))
        ns["poll_telemetry"]()
        self.assertEqual((ns["telem_pitch"], ns["telem_yaw"], ns["telem_roll"]),
                         (1.25, -2.5, 3.75))
        fake_time.now = 20
        uart.rx.extend(frame(ns, -4.5, 5.25, -6.75, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        self.assertEqual((ns["telem_pitch"], ns["telem_yaw"], ns["telem_roll"]),
                         (-4.5, 5.25, -6.75))
        self.assertTrue(ns["flight_seen"])

    def test_telemetry_csv_logs_every_valid_frame_and_closes_after_descent(self):
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        fake_time.now = 25
        uart.rx.extend(frame(ns, 1.25, -2.5, 3.75, ns["STATE_ASCENT"]))
        uart.rx.extend(frame(ns, 2.5, -3.0, 4.0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()

        log = FakeTextFile.instances[0]
        rows = "".join(log.data).splitlines()
        self.assertEqual(log.path, "/sdcard/telem_00001.csv")
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[1].split(",")[:5], ["25", "1", "3", "4", "12"])
        self.assertEqual(rows[2].split(",")[1], "2")

        invalid = frame(ns, 9, 9, 9, ns["STATE_ASCENT"])
        invalid[-1] ^= 0xFF
        uart.rx.extend(invalid)
        ns["poll_telemetry"]()
        self.assertEqual(len("".join(log.data).splitlines()), 3)

        fake_time.now = 100
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_DESCENT"]))
        ns["poll_telemetry"]()
        fake_time.now = 3101
        ns["poll_telemetry"]()
        self.assertTrue(log.closed)
        self.assertTrue(ns["telem_log_done"])

    def test_recording_starts_after_missed_launch_and_closes_after_tail(self):
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_DESCENT"]))
        ns["poll_telemetry"]()
        ns["update_recording"](object())
        self.assertEqual(FakeImageIO.instances[0].frames, 1)
        self.assertEqual(FakeImageIO.instances[0].path, "/sdcard/rec_00001.bin")
        fake_time.now = 3001
        ns["update_recording"](object())
        self.assertTrue(FakeImageIO.instances[0].closed)
        self.assertTrue(ns["record_done"])

    def test_recording_closes_on_max_duration_without_descent(self):
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        ns["update_recording"](object())
        fake_time.now = ns["MAX_RECORD_MS"]
        ns["update_recording"](object())
        self.assertTrue(FakeImageIO.instances[0].closed)

    def test_native_stream_syncs_without_jpeg_arguments(self):
        ns, fake_time, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["flight_seen"] = True
        ns["update_recording"](object())
        fake_time.now = 1100
        ns["update_recording"](object())
        self.assertEqual(FakeImageIO.instances[0].frames, 2)
        self.assertEqual(FakeImageIO.instances[0].syncs, 1)

    def test_write_failure_closes_stream(self):
        ns, _, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["start_recording"]()
        stream = FakeImageIO.instances[0]

        def fail_write(img):
            raise OSError("SD write failed")

        stream.write = fail_write
        ns["update_recording"](object())
        self.assertTrue(stream.closed)
        self.assertFalse(ns["recording"])
        self.assertTrue(ns["record_done"])


if __name__ == "__main__":
    unittest.main()

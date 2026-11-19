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


class FakeMjpeg:
    instances = []

    def __init__(self, path):
        self.path = path
        self.frames = 0
        self.qualities = []
        self.closed = False
        self.instances.append(self)

    def write(self, img, quality=60):
        self.frames += 1
        self.qualities.append(quality)

    def close(self):
        self.closed = True


class FailingMjpeg(FakeMjpeg):
    """模拟 mjpeg 模块在这块板子上写不进去的情况。"""

    def write(self, img, quality=60):
        raise OSError("mjpeg write unsupported")


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
             and isinstance(node, (ast.Assign, ast.FunctionDef,
                                   ast.ClassDef, ast.Try))
             and not (isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == "uart"
                              for target in node.targets))]
    module = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    fake_time = FakeTime()
    fake_uart = FakeUART()
    FakeImageIO.instances = []
    FakeMjpeg.instances = []
    FakeTextFile.instances = []
    namespace = {
        "time": fake_time,
        "uart": fake_uart,
        "open": FakeTextFile,
        "start": 0,
        "image": types.SimpleNamespace(ImageIO=FakeImageIO),
        "os": types.SimpleNamespace(stat=lambda path: (_ for _ in ()).throw(OSError())),
        "sensor": types.SimpleNamespace(QVGA=1, RGB565=2),
        # main.py 顶层 import gc 会被 AST 过滤掉，这里补一个：
        # mem_free 给大值让预检查跳过，enable/disable/collect 走空实现。
        "gc": types.SimpleNamespace(
            enable=lambda: None,
            disable=lambda: None,
            collect=lambda: None,
            mem_free=lambda: 999999,
        ),
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

    def test_telemetry_csv_logs_every_valid_frame_and_closes_after_launch_cap(self):
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

    def test_new_guidance_columns_carry_values(self):
        """四个新增制导列必须真的从帧里解出来（曾因漏写 global 恒为 0）。"""
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        fake_time.now = 10
        raw = frame(ns, 0, 0, 0, ns["STATE_DESCENT"])
        for name, value in (("A_MEAS", 350), ("A_CMD", -1225),
                            ("PN_OMEGA", 2500), ("VIS_RATE", -180),
                            ("VIS_X", 96), ("VIS_Y", -42)):
            raw[ns["T_" + name + "_H"]] = (value >> 8) & 0xFF
            raw[ns["T_" + name + "_L"]] = value & 0xFF
        raw[ns["T_CRC_INDEX"]] = ns["crc8_lsb"](raw, 0, ns["T_CRC_INDEX"])
        uart.rx.extend(raw)
        ns["poll_telemetry"]()
        cols = "".join(FakeTextFile.instances[0].data).splitlines()[1].split(",")
        self.assertEqual(cols[20], "3.50")
        self.assertEqual(cols[21], "-12.25")
        self.assertEqual(cols[22], "25.0")
        self.assertEqual(cols[23], "-1.8")     # 视线角速率 /100
        self.assertEqual(cols[24], "96")       # 绿灯像素 x，不缩放
        self.assertEqual(cols[25], "-42")      # 绿灯像素 y

    def test_recording_starts_after_missed_launch_and_closes_after_launch_cap(self):
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

    def test_mjpeg_backend_writes_playable_stream(self):
        """固件带 mjpeg 时应走 .mjpeg 直编码，而不是 .bin 原始流。"""
        ns, fake_time, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["mjpeg"] = types.SimpleNamespace(Mjpeg=FakeMjpeg)
        ns["flight_seen"] = True
        ns["update_recording"](object())

        stream = FakeMjpeg.instances[0]
        self.assertTrue(stream.path.endswith(".mjpeg"))
        self.assertEqual(stream.frames, 1)
        self.assertEqual(stream.qualities, [ns["RECORD_JPEG_QUALITY"]])
        self.assertTrue(ns["record_is_mjpeg"])
        self.assertEqual(FakeImageIO.instances, [])

    def test_mjpeg_recording_pools_image_before_encode(self):
        """录像前必须把画面池化，给 JPEG 编码器腾出 fb_alloc。

        11.15 下午 22 发里只有 2 发 mjpeg 成功，其余第一帧就抛
        Compression Failed!——QVGA 下帧缓冲 150 KB + 编码 scratch 约 150 KB
        把 fb_alloc 卡在边界上，池化到 1/2 后 scratch 降到约 38 KB。
        """
        class FakeImg:
            def __init__(self):
                self.pooled = []

            def mean_pool(self, x_div, y_div):
                self.pooled.append((x_div, y_div))

        ns, fake_time, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["mjpeg"] = types.SimpleNamespace(Mjpeg=FakeMjpeg)
        ns["flight_seen"] = True
        img = FakeImg()
        ns["update_recording"](img)

        self.assertGreater(ns["RECORD_POOL_DIV"], 1, "默认应当池化")
        self.assertEqual(img.pooled,
                         [(ns["RECORD_POOL_DIV"], ns["RECORD_POOL_DIV"])])
        self.assertEqual(FakeMjpeg.instances[0].frames, 1)

    def test_bin_recording_does_not_pool_image(self):
        """.bin 走原始帧，池化只会丢分辨率，不能对它生效。"""
        class FakeImg:
            def __init__(self):
                self.pooled = []

            def mean_pool(self, x_div, y_div):
                self.pooled.append((x_div, y_div))

        ns, fake_time, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["flight_seen"] = True
        img = FakeImg()
        ns["update_recording"](img)

        self.assertFalse(ns["record_is_mjpeg"])
        self.assertEqual(img.pooled, [])
        self.assertEqual(FakeImageIO.instances[0].frames, 1)

    def test_recording_starts_in_debug_mode(self):
        """台架调试没有发射，也要能录像。"""
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        fake_time.now = 25
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_DEBUG"]))
        ns["poll_telemetry"]()
        self.assertFalse(ns["flight_seen"])
        ns["update_recording"](object())
        self.assertTrue(ns["recording"])
        self.assertEqual(FakeImageIO.instances[0].frames, 1)

        # debug 不套用发射后 3 s 兜底
        fake_time.now = ns["RECORD_MAX_FROM_LAUNCH_MS"] + 100
        ns["update_recording"](object())
        self.assertTrue(ns["recording"])
        self.assertFalse(FakeImageIO.instances[0].closed)

    def test_debug_mode_ignores_launch_cap_but_honours_absolute_cap(self):
        """debug 只等落地标志，不套用发射后 3 s 兜底，但仍有绝对上限。"""
        ns, fake_time, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["telem_state"] = ns["STATE_DEBUG"]
        ns["flight_seen"] = True
        ns["update_recording"](object())
        stream = FakeImageIO.instances[0]

        fake_time.now = ns["RECORD_MAX_FROM_LAUNCH_MS"] + 1
        ns["update_recording"](object())
        self.assertFalse(stream.closed)
        self.assertTrue(ns["recording"])

        fake_time.now = ns["MAX_RECORD_MS"] + 1
        ns["update_recording"](object())
        self.assertTrue(stream.closed)
        self.assertTrue(ns["record_done"])

    def test_mjpeg_write_failure_falls_back_to_imageio(self):
        """MJPEG 连第一帧都写不进去时，自动降级到 ImageIO，至少能拿到画面。"""
        ns, fake_time, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["mjpeg"] = types.SimpleNamespace(Mjpeg=FailingMjpeg)
        ns["flight_seen"] = True

        ns["update_recording"](object())

        self.assertTrue(ns["record_fallback_bin"])
        self.assertTrue(ns["recording"])
        self.assertEqual(len(FakeImageIO.instances), 1)
        self.assertTrue(FakeImageIO.instances[0].path.endswith(".bin"))

        # 降级后的下一帧应该能正常写入
        fake_time.now = 100
        ns["update_recording"](object())
        self.assertEqual(FakeImageIO.instances[0].frames, 1)

    def test_second_flight_clears_previous_landed_flag(self):
        """落地后再次起飞必须清掉上一轮的 landed_seen_ms。

        否则 record_should_stop 会拿上一轮的落地时间戳立即返回 True，
        录像只写出文件头（约 1 KB）就结束。
        """
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"

        # 第一轮：起飞 -> 落地 -> 停止
        fake_time.now = 100
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        ns["update_recording"](object())
        self.assertTrue(ns["recording"])

        fake_time.now = 2000
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_LANDED"]))
        ns["poll_telemetry"]()
        fake_time.now = 2000 + ns["LANDED_RECORD_TAIL_MS"] + 1
        ns["update_recording"](object())
        self.assertFalse(ns["recording"])
        self.assertTrue(ns["record_done"])
        self.assertIsNotNone(ns["landed_seen_ms"])

        # 第二轮：重新起飞，落地标志与 record_done 都要复位
        fake_time.now = 5000
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        self.assertIsNone(ns["landed_seen_ms"])
        self.assertIsNone(ns["descent_seen_ms"])
        self.assertFalse(ns["record_done"])

        ns["update_recording"](object())
        self.assertTrue(ns["recording"])
        self.assertEqual(len(FakeImageIO.instances), 2)

    def test_recording_stops_after_landed_flag(self):
        """落地标志优先于 3 s 兜底：落地早于 3 s 时必须提前停。"""
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        fake_time.now = 25
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        ns["update_recording"](object())
        stream = FakeImageIO.instances[0]
        self.assertTrue(ns["recording"])

        fake_time.now = 2000
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_LANDED"]))
        ns["poll_telemetry"]()
        ns["update_recording"](object())
        self.assertFalse(stream.closed)        # 落地尾巴还没到

        fake_time.now = 2000 + ns["LANDED_RECORD_TAIL_MS"] + 1
        ns["update_recording"](object())
        self.assertTrue(stream.closed)
        self.assertTrue(ns["record_done"])

    def test_record_event_log_records_start_and_stop_with_frame_count(self):
        """record_err.txt 必须能区分「开完就停」和「写到一半断」。

        224 字节的零帧录像在场上无法判断是哪一种，所以 start 和 stop 都要记，
        stop 还要带已写帧数和停止原因。
        """
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        fake_time.now = 25
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        ns["update_recording"](object())
        # 测试走 ImageIO(.bin)，写间隔是 RECORD_INTERVAL_BIN_MS，不是 33 ms
        fake_time.now = 25 + ns["RECORD_INTERVAL_BIN_MS"] + 1
        ns["update_recording"](object())
        self.assertEqual(FakeImageIO.instances[0].frames, 2)

        fake_time.now = 2000
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_LANDED"]))
        ns["poll_telemetry"]()
        fake_time.now = 2000 + ns["LANDED_RECORD_TAIL_MS"] + 1
        ns["update_recording"](object())

        logs = [f for f in FakeTextFile.instances
                if f.path.endswith("record_err.txt")]
        self.assertTrue(logs, "record_err.txt 没被写过")
        # 每次 append 都会新开一个 FakeTextFile，要把所有实例拼起来
        text = "".join("".join(f.data) for f in logs)
        self.assertIn(",start,", text)
        self.assertIn(",stop,", text)
        self.assertIn("frames=2", text)
        self.assertIn("reason=should_stop", text)

    def test_telemetry_log_stops_after_landed_flag(self):
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        fake_time.now = 25
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        log = FakeTextFile.instances[0]

        fake_time.now = 2000
        uart.rx.extend(frame(ns, 0, 0, 0, ns["STATE_LANDED"]))
        ns["poll_telemetry"]()
        self.assertFalse(log.closed)

        fake_time.now = 2000 + ns["LANDED_RECORD_TAIL_MS"] + 1
        ns["poll_telemetry"]()
        self.assertTrue(log.closed)
        self.assertTrue(ns["telem_log_done"])

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

    def test_sync_failure_does_not_stop_recording(self):
        """sync 只是掉电保险，失败不能让整段录像结束（曾导致录像提前一半）。"""
        ns, fake_time, _ = load_logic()
        ns["record_dir"] = "/sdcard"
        ns["start_recording"]()
        stream = FakeImageIO.instances[0]

        def fail_sync():
            raise OSError("SD sync failed")

        stream.sync = fail_sync
        fake_time.now = 1100          # 跨过 RECORD_SYNC_MS = 1000
        ns["update_recording"](object())

        self.assertFalse(stream.closed)
        self.assertTrue(ns["recording"])
        self.assertFalse(ns["record_done"])
        self.assertEqual(ns["record_sync_err"], 1)

        fake_time.now = 1200          # 下一帧必须照常写入
        ns["update_recording"](object())
        self.assertEqual(stream.frames, 2)
        self.assertTrue(ns["recording"])

    def test_flush_failure_does_not_stop_telemetry_log(self):
        """flush 失败同理，不能把 CSV 断在 2 秒处。"""
        ns, fake_time, uart = load_logic()
        ns["record_dir"] = "/sdcard"
        fake_time.now = 25
        uart.rx.extend(frame(ns, 1.0, -2.0, 3.0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        log = FakeTextFile.instances[0]

        def fail_flush():
            raise OSError("flush failed")

        log.flush = fail_flush
        fake_time.now = 1025         # 跨过 TELEM_LOG_SYNC_MS = 1000
        uart.rx.extend(frame(ns, 1.5, -2.5, 3.5, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()

        self.assertFalse(log.closed)
        self.assertFalse(ns["telem_log_done"])
        self.assertEqual(ns["telem_log_sync_err"], 1)

        fake_time.now = 1050
        uart.rx.extend(frame(ns, 2.0, -3.0, 4.0, ns["STATE_ASCENT"]))
        ns["poll_telemetry"]()
        self.assertEqual(len("".join(log.data).splitlines()), 4)


if __name__ == "__main__":
    unittest.main()

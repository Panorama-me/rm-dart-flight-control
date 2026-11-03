"""Path discovery and missing-recording behavior without OpenMV hardware."""

import ast
import contextlib
import io
import pathlib
import types
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "imageio_playback.py"


class FakeStorage:
    def __init__(self, directories):
        self.directories = directories
        self.listdir_calls = []

    def getcwd(self):
        return "/"

    def listdir(self, path):
        self.listdir_calls.append(path)
        if path not in self.directories:
            raise OSError(2)
        return self.directories[path]

    def stat(self, path):
        directory, name = path.rsplit("/", 1)
        if name not in self.directories.get(directory or "/", []):
            raise OSError(2)
        return (0x8000, 0, 0, 0, 0, 0, 160000, 0, 0, 0)


def load_playback(directories, fail_open=False):
    opened = []
    source = types.SimpleNamespace(closed=False)

    def read(**kwargs):
        return None

    def close():
        source.closed = True

    def open_stream(path, mode):
        opened.append((path, mode))
        if fail_open:
            raise OSError("Invalid ImageIO header")
        return source

    source.read = read
    source.close = close
    namespace = {
        "os": FakeStorage(directories),
        "image": types.SimpleNamespace(ImageIO=open_stream),
        "csi": types.SimpleNamespace(
            CSI=lambda: types.SimpleNamespace(reset=lambda: None,
                                              pixformat=lambda value: None,
                                              framesize=lambda value: None),
            RGB565=2, QVGA=1),
    }
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
    nodes = [node for node in tree.body
             if isinstance(node, (ast.Assign, ast.FunctionDef))]
    module = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    exec(compile(module, str(SCRIPT), "exec"), namespace)
    return namespace, opened, source


class PlaybackPathsTest(unittest.TestCase):
    def test_existing_requested_path_does_not_scan_storage(self):
        ns, opened, source = load_playback({
            "/sdcard": ["rec_00002.bin"],
        })
        ns["STREAM_PATH"] = "/sdcard/rec_00002.bin"
        with contextlib.redirect_stdout(io.StringIO()):
            ns["play_stream"]()
        self.assertEqual(opened, [("/sdcard/rec_00002.bin", "r")])
        self.assertEqual(ns["os"].listdir_calls, [])
        self.assertTrue(source.closed)

    def test_auto_selects_highest_record_number_on_either_sd_mount(self):
        for mount in ("/sdcard", "/sd"):
            with self.subTest(mount=mount):
                ns, opened, source = load_playback({
                    mount: ["rec_00001.bin", "rec_00012.bin", "firmware.bin"],
                    "/": [mount.lstrip("/")],
                })
                with contextlib.redirect_stdout(io.StringIO()):
                    ns["play_stream"]()
                self.assertEqual(opened, [(mount + "/rec_00012.bin", "r")])
                self.assertTrue(source.closed)

    def test_missing_requested_path_lists_choices_without_opening_another_file(self):
        ns, opened, _ = load_playback({"/sdcard": ["rec_00002.bin"]})
        ns["STREAM_PATH"] = "/sdcard/rec_00001.bin"
        messages = io.StringIO()
        with contextlib.redirect_stdout(messages):
            ns["play_stream"]()
        self.assertEqual(opened, [])
        self.assertIn("/sdcard/rec_00002.bin", messages.getvalue())

    def test_no_recording_prints_storage_and_recording_instructions(self):
        ns, opened, _ = load_playback({"/": ["flash"], "/flash": ["main.py"]})
        messages = io.StringIO()
        with contextlib.redirect_stdout(messages):
            ns["play_stream"]()
        self.assertEqual(opened, [])
        self.assertIn("Record with main.py first", messages.getvalue())

    def test_invalid_recording_reports_file_and_error(self):
        ns, _, _ = load_playback({"/sdcard": ["rec_00001.bin"]}, fail_open=True)
        messages = io.StringIO()
        with contextlib.redirect_stdout(messages):
            ns["play_stream"]()
        self.assertIn("Cannot open ImageIO recording", messages.getvalue())
        self.assertIn("Invalid ImageIO header", messages.getvalue())


if __name__ == "__main__":
    unittest.main()

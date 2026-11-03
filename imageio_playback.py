"""Run separately in OpenMV IDE to play a recording or export it to MJPEG.

Stop main.py before running this script. By default the highest-numbered
rec_XXXXX.bin is selected automatically. Set STREAM_PATH to choose a file.
Set EXPORT_MJPEG=True to also create a host-playable file on the same SD card.
"""

import image
import os
import csi


STREAM_PATH = None  # Auto-select; or set an existing full path from record start.
EXPORT_MJPEG = False
EXPORT_QUALITY = 60
SEARCH_DIRS = ("/sdcard", "/sd", "/flash", "/")


def find_recordings():
    candidates = []
    directories = list(SEARCH_DIRS)
    try:
        current = os.getcwd()
        if current not in directories:
            directories.append(current)
    except OSError:
        pass

    for priority, directory in enumerate(directories):
        try:
            names = os.listdir(directory)
        except OSError:
            continue
        for name in names:
            if not (name.startswith("rec_") and name.endswith(".bin")):
                continue
            number = name[4:-4]
            if not number.isdigit():
                continue
            path = directory.rstrip("/") + "/" + name
            candidates.append((int(number), -priority, path))
    candidates.sort(reverse=True)
    return [candidate[2] for candidate in candidates]


def select_stream_path():
    if STREAM_PATH:
        try:
            os.stat(STREAM_PATH)
            return STREAM_PATH
        except OSError:
            print("Recording path does not exist:", STREAM_PATH)
    recordings = find_recordings()
    if STREAM_PATH and recordings:
        print("Available recordings:")
        for path in recordings[:10]:
            print(" ", path)
        print("Set STREAM_PATH=None to select automatically.")
        return None
    if not STREAM_PATH and recordings:
        print("Auto selected:", recordings[0])
        return recordings[0]

    print("No rec_XXXXX.bin recording found. Storage contents:")
    for directory in SEARCH_DIRS:
        try:
            print(directory, os.listdir(directory)[:20])
        except OSError:
            print(directory, "not mounted")
    print("Record with main.py first; wait for record start / record stop.")
    print("This script plays existing .bin files; it does not record video.")
    return None


def play_stream():
    stream_path = select_stream_path()
    if stream_path is None:
        return
    output_path = stream_path.rsplit(".", 1)[0] + ".mjpeg"
    if EXPORT_MJPEG:
        try:
            os.stat(output_path)
        except OSError:
            pass
        else:
            print("Output already exists:", output_path)
            print("Rename the existing output or set EXPORT_MJPEG=False.")
            return

    try:
        source = image.ImageIO(stream_path, "r")
    except (OSError, ValueError) as exc:
        print("Cannot open ImageIO recording:", stream_path, exc)
        return
    output = None
    frames = 0
    try:
        camera = csi.CSI()
        camera.reset()
        camera.pixformat(csi.RGB565)
        camera.framesize(csi.QVGA)
        while True:
            try:
                # Show each frame in the IDE and follow the stored frame timing.
                img = source.read(copy_to_fb=True, loop=False, pause=True)
            except (OSError, EOFError) as exc:
                # Keep any successfully read prefix if the recording is damaged.
                print("Stream read stopped after", frames, "frames:", exc)
                break
            if img is None:
                break

            if EXPORT_MJPEG:
                if output is None:
                    import mjpeg

                    output = mjpeg.Mjpeg(output_path)
                output.write(img, quality=EXPORT_QUALITY)
            frames += 1
    finally:
        try:
            if output is not None:
                output.close()
        finally:
            source.close()

    print("Frames played:", frames)
    if EXPORT_MJPEG and output is not None:
        print("MJPEG saved:", output_path)


if __name__ == "__main__":
    play_stream()

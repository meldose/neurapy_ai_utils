from rosbags.highlevel import AnyReader
from pathlib import Path
import cv2
from rosbags.image import message_to_cvimage


def get_images(bagdir, outdir):
    outdir = Path(outdir)
    if not outdir.exists():
        outdir.mkdir()
    count = 0
    with AnyReader([Path(bagdir)]) as reader:
        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)
        for connection, timestamp, rawdata in reader.messages():
            if (
                connection.topic == "/camera/camera/color/image_raw"
            ):  # topic Name of images
                msg = reader.deserialize(rawdata, connection.msgtype)
                # change encoding type if needed
                img = message_to_cvimage(msg, "bgr8")
                cv2.imwrite(outdir / f"img_{count}.png", img)
                count += 1


bagdir = "/home/alina.kloss/Downloads/rosbag2_2025_06_22-17_38_27"
out_dir = "/home/alina.kloss/Desktop/annotation_data/maira2"

get_images(bagdir, out_dir)

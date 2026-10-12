#!/usr/bin/env python3
"""
extract_bag_video.py
Estrae lo stream video della camera registrato in una rosbag ROS 2 e genera un video MP4.
Supporta sia sensor_msgs/msg/CompressedImage che sensor_msgs/msg/Image.

Uso:
    python3 extract_bag_video.py [--bag <percorso_rosbag>] [--output <percorso_output.mp4>] [--topic <nome_topic>]
"""

import os
import sys
import glob
import argparse
import cv2
import numpy as np
import yaml

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image


def find_latest_bag(base_dir):
    bags = glob.glob(os.path.join(base_dir, "**/metadata.yaml"), recursive=True)
    if not bags:
        return None
    latest = max(bags, key=os.path.getmtime)
    return os.path.dirname(latest)


def detect_image_topic(bag_dir):
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.exists(meta_path):
        return None, None
    with open(meta_path, 'r') as f:
        meta = yaml.safe_load(f)
    
    info = meta.get('rosbag2_bagfile_information', {})
    topics = info.get('topics_with_message_count', [])
    for t in topics:
        t_meta = t.get('topic_metadata', {})
        name = t_meta.get('name', '')
        t_type = t_meta.get('type', '')
        if 'CompressedImage' in t_type or 'Image' in t_type:
            return name, t_type
    return None, None


def main():
    parser = argparse.ArgumentParser(description="Estrai video MP4 da una rosbag ROS 2.")
    parser.add_argument('--bag', type=str, default=None, help="Percorso della directory rosbag (contenente metadata.yaml)")
    parser.add_argument('--topic', type=str, default=None, help="Topic dell'immagine (default: auto-detect)")
    parser.add_argument('--output', type=str, default=None, help="Percorso del file video di output (.mp4)")
    parser.add_argument('--fps', type=float, default=None, help="FPS del video di output (default: calcolato dai timestamp)")
    args = parser.parse_args()

    # 1. Trova la rosbag
    bag_path = args.bag
    if bag_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        bag_files_dir = os.path.join(script_dir, "bag_files")
        bag_path = find_latest_bag(bag_files_dir)
        if bag_path is None:
            print("[ERRORE] Nessuna rosbag trovata automaticamente. Specifica il path con --bag <path>")
            sys.exit(1)
        print(f"[INFO] Trovata ultima rosbag: {bag_path}")

    # 2. Identifica il topic
    topic_name = args.topic
    topic_type = None
    if topic_name is None:
        topic_name, topic_type = detect_image_topic(bag_path)
        if topic_name is None:
            topic_name = '/camera/camera/color/image_raw/compressed'
            topic_type = 'sensor_msgs/msg/CompressedImage'
    print(f"[INFO] Topic video selezionato: {topic_name} (tipo: {topic_type or 'auto'})")

    # 3. Path di output
    output_path = args.output
    if output_path is None:
        output_dir = os.path.dirname(bag_path) if os.path.basename(bag_path) == "rosbag" else bag_path
        output_path = os.path.join(output_dir, "camera_video.mp4")

    # 4. Apri la bag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"[ERRORE] Impossibile aprire la bag: {e}")
        sys.exit(1)

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_name]))

    # 5. Primo passaggio: estrazione frame
    is_compressed = "compressed" in topic_name.lower() or (topic_type and "CompressedImage" in topic_type)
    msg_cls = CompressedImage if is_compressed else Image

    timestamps = []
    frames = []
    print("[INFO] Lettura dei frame dalla rosbag in corso...")

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, msg_cls)
        timestamps.append(t)

        if is_compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            # sensor_msgs/msg/Image
            if msg.encoding in ('rgb8', 'bgr8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if msg.encoding == 'rgb8':
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                print(f"[ATTENZIONE] Encoding {msg.encoding} non supportato direttamente, skip.")
                continue

        if frame is not None:
            frames.append(frame)

    if not frames:
        print(f"[ERRORE] Nessun frame estratto dal topic {topic_name}.")
        sys.exit(1)

    # 6. Calcolo FPS
    if args.fps is not None:
        fps = args.fps
    elif len(timestamps) > 1:
        duration_sec = (timestamps[-1] - timestamps[0]) / 1e9
        fps = round(len(timestamps) / duration_sec, 2) if duration_sec > 0 else 25.0
    else:
        fps = 25.0

    h, w = frames[0].shape[:2]
    print(f"[INFO] Frame estratti: {len(frames)} | Risoluzione: {w}x{h} | FPS calcolato: {fps}")

    # 7. Scrittura video MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

    for f in frames:
        writer.write(f)
    writer.release()

    print(f"[SUCCESS] Video salvato con successo in:\n  {output_path}")


if __name__ == '__main__':
    main()

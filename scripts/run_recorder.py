"""Record what happens during a simulator run, for the report.

Run it next to any launch. Every 5 seconds it logs the rover's position and how much of the cave is mapped, and it keeps
pictures from the detections image. On exit it saves the final map.

Usage: python3 scripts/run_recorder.py OUTPUT_DIR SECONDS
Writes: OUTPUT_DIR/timeline.csv, OUTPUT_DIR/final_map.npz, OUTPUT_DIR/detections/*.png
"""
import csv
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Recorder(Node):
    def __init__(self, out_dir):
        super().__init__('run_recorder')
        self.out_dir = out_dir
        os.makedirs(os.path.join(out_dir, 'detections'), exist_ok=True)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map = None
        self.latest_detection = None
        self.saved_detections = 0
        self.start = time.time()
        self.rows = []
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map', m), 1)
        self.create_subscription(Image, 'detections_image', lambda m: setattr(self, 'latest_detection', m), 1)
        self.create_timer(5.0, self.tick)
        self.create_timer(30.0, self.save)      # keep a copy on disk in case the run is interrupted

    def tick(self):
        elapsed = time.time() - self.start
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x, y = t.transform.translation.x, t.transform.translation.y
        except TransformException:
            x = y = float('nan')
        free = walls = 0.0
        if self.map is not None:
            data = np.array(self.map.data, dtype=np.int8)
            res = self.map.info.resolution
            free = float(((data >= 0) & (data < 50)).sum()) * res * res
            walls = float((data >= 50).sum()) * res * res
        self.rows.append([round(elapsed, 1), x, y, round(free, 1), round(walls, 1)])

        # Keep a picture of the detections image now and then, but only when it has a coloured box drawn on it
        if self.latest_detection is not None and self.saved_detections < 60:
            image = self.bridge.imgmsg_to_cv2(self.latest_detection, 'passthrough')
            boxes = ((image[..., 1] > 200) & (image[..., 0] < 80) & (image[..., 2] < 80)).sum()
            if boxes > 60:
                cv2.imwrite(os.path.join(self.out_dir, 'detections', f'{int(elapsed):05d}.png'),
                            cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                self.saved_detections += 1

    def save(self):
        with open(os.path.join(self.out_dir, 'timeline.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['seconds', 'x', 'y', 'free_area_m2', 'wall_area_m2'])
            writer.writerows(self.rows)
        if self.map is not None:
            info = self.map.info
            grid = np.array(self.map.data, dtype=np.int8).reshape(info.height, info.width)
            np.savez(os.path.join(self.out_dir, 'final_map.npz'), map=grid, res=info.resolution,
                     ox=info.origin.position.x, oy=info.origin.position.y)


def main():
    rclpy.init()
    node = Recorder(sys.argv[1])
    end = time.time() + float(sys.argv[2])
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.5)
    except BaseException as error:           # an interrupt must not lose the recording
        print('stopped early:', type(error).__name__)
    node.save()
    print(f'saved {len(node.rows)} rows and {node.saved_detections} detection pictures to {sys.argv[1]}')


if __name__ == '__main__':
    main()

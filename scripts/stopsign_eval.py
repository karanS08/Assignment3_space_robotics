"""Score the template's stop-sign detector (Haar cascade) on the labelled simulator captures.

A stop sign counts as visible in an image when the wall carrying it is in front of the camera, 1 to 9 m away, with a clear line of
sight. The wall's centre is used as the sign's position, so the check is generous by up to about 2.5 m.
A detection is correct when its box covers the image column of a visible sign (with a margin); otherwise it is a false alarm.

Usage: python3 scripts/stopsign_eval.py DATA_DIR
"""
import json
import math
import os
import re
import sys

import cv2
import numpy as np

from offline_cave_map import PACKAGE, load_cave_grid

RES = 0.3
FOCAL = 360.0 / math.tan(2.0944 / 2)
MARGIN = 60


def stop_sign_walls():
    sdf = open(os.path.join(PACKAGE, 'worlds', 'mars_cave.sdf')).read()
    out = []
    for block in re.split(r'(?=<model name=")', sdf):
        m = re.match(r'<model name="([^"]+)">\s*<pose>([^<]+)</pose>', block)
        if m and 'stop' in block.split('</model>')[0].lower():
            p = [float(v) for v in m.group(2).split()]
            out.append((p[0], p[1]))
    return out


def clear_line(wall, a, b, origin):
    """No wall between a and b. The line stops 0.7 m short of b, because b is the wall that carries the sign."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    for t in np.linspace(0, max(0.0, 1 - 0.7 / max(length, 1e-6)), max(2, int(length / (RES / 2)))):
        col = int((a[0] + t * (b[0] - a[0]) - origin[0]) / RES)
        row = int((a[1] + t * (b[1] - a[1]) - origin[1]) / RES)
        if not (0 <= row < wall.shape[0] and 0 <= col < wall.shape[1]) or wall[row, col]:
            return False
    return True


def main():
    data = sys.argv[1]
    records = json.load(open(os.path.join(data, 'labels.json')))
    grid, origin = load_cave_grid(RES)
    wall = grid >= 50
    signs = stop_sign_walls()
    cascade = cv2.CascadeClassifier(os.path.join(PACKAGE, 'config', 'stop_data.xml'))
    assert not cascade.empty()

    visible_images = found = detections_total = false_alarm_boxes = images_with_false = 0
    for r in records:
        yaw = r['yaw']
        cam = (r['x'] + 0.5 * math.cos(yaw), r['y'] + 0.5 * math.sin(yaw))
        columns = []
        for sx, sy in signs:
            dx, dy = sx - cam[0], sy - cam[1]
            forward, left = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
            if forward > 0.2 and 1.0 <= math.hypot(dx, dy) <= 9.0 and clear_line(wall, cam, (sx, sy), origin):
                column = 360 - FOCAL * left / forward
                if -MARGIN <= column <= 720 + MARGIN:
                    columns.append(column)
        image = cv2.imread(os.path.join(data, r['id'] + '.png'))
        boxes = cascade.detectMultiScale(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), scaleFactor=1.1, minNeighbors=4, minSize=(20, 20))
        detections_total += len(boxes)
        correct = [b for b in boxes if any(b[0] - MARGIN <= c <= b[0] + b[2] + MARGIN for c in columns)]
        false = len(boxes) - len(correct)
        false_alarm_boxes += false
        images_with_false += 1 if false else 0
        if columns:
            visible_images += 1
            found += 1 if correct else 0
    print(f'{len(records)} images, {len(signs)} stop signs in the world')
    print(f'images where a stop sign is visible: {visible_images}; detector found it in {found} ({100 * found / max(visible_images, 1):.0f}%)')
    print(f'boxes drawn in total: {detections_total}; false alarms: {false_alarm_boxes} boxes in {images_with_false} images '
          f'({100 * images_with_false / len(records):.0f}% of all images)')


if __name__ == '__main__':
    main()

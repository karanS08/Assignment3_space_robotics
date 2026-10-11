"""Offline check of the cave geometry analysis (Advanced 2), no ROS needed.

Rasterises the walls of worlds/mars_cave.sdf into an occupancy grid at the
SLAM resolution, runs the analysis and saves a picture.

Usage: python3 scripts/geometry_offline_check.py [full|partial] [output.png]
"""
import math
import os
import re
import sys
import time

import cv2
import numpy as np

PACKAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cave_explorer')
sys.path.insert(0, PACKAGE)
from cave_explorer.advanced import cave_geometry as cg

RES = 0.3
sdf = open(os.path.join(PACKAGE, 'worlds', 'mars_cave.sdf')).read()
walls = []
for m in re.finditer(r'<model name="wall_\d+">\s*<pose>([^<]+)</pose>.*?<size>([^<]+)</size>', sdf, re.S):
    px, py, _, _, _, yaw = map(float, m.group(1).split())
    sx, sy, sz = map(float, m.group(2).split())
    length = sx if abs(sx - 4) > 1e-6 else sy
    walls.append((px, py, yaw, length))
print('walls', len(walls))
xs = [w[0] for w in walls]; ys = [w[1] for w in walls]
ox, oy = min(xs) - 6, min(ys) - 6
W = int((max(xs) + 6 - ox) / RES); H = int((max(ys) + 6 - oy) / RES)
grid = np.zeros((H, W), np.int8)
img = np.zeros((H, W), np.uint8)
for px, py, yaw, L in walls:
    a = (px - L/2*math.cos(yaw), py - L/2*math.sin(yaw)); b = (px + L/2*math.cos(yaw), py + L/2*math.sin(yaw))
    pa = (int((a[0]-ox)/RES), int((a[1]-oy)/RES)); pb = (int((b[0]-ox)/RES), int((b[1]-oy)/RES))
    cv2.line(img, pa, pb, 255, 1)
grid[img > 0] = 100
mode = sys.argv[1] if len(sys.argv) > 1 else 'full'
robot = (0.0, 0.0)
if mode == 'partial':   # only a disc around the robot is known
    yy, xx = np.mgrid[0:H, 0:W]
    far = np.hypot(xx*RES + ox - 10, yy*RES + oy - 8) > 16
    grid[far] = -1
t = time.time(); res = cg.analyse(grid, RES, (ox, oy), robot_xy=robot); dt = time.time() - t
print('grid', grid.shape, 'time %.3fs' % dt, 'accessible cells', int(res.accessible.sum()), 'ridge', int(res.ridge.sum()))
print('widest', res.widest)
for p in res.passages: print('  passage (%.1f, %.1f) width %.2f %s saddle=%s' % (p.x, p.y, p.width, p.kind, p.is_saddle))
print('clearance_at robot', cg.clearance_at(grid, RES, (ox, oy), *robot))

# Render
S = 5
c = res.clearance.copy(); c[~res.accessible] = 0
vis = cv2.applyColorMap((255 * c / max(c.max(), 1e-6)).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
vis[~res.accessible] = (60, 60, 60); vis[grid < 0] = (120, 120, 120); vis[grid >= 50] = (255, 255, 255)
vis[res.ridge] = (0, 220, 255)
vis = cv2.resize(vis, (W*S, H*S), interpolation=cv2.INTER_NEAREST)
vis = cv2.flip(vis, 0)
px = lambda x, y: (int((x - ox) / RES * S), H * S - int((y - oy) / RES * S))
if res.widest:
    cv2.circle(vis, px(res.widest.x, res.widest.y), int(res.widest.radius / RES * S), (255, 120, 0), 3)
for i, p in enumerate(res.passages):
    cv2.line(vis, px(*p.wall_a), px(*p.wall_b), (0, 0, 255), 3)
    cv2.putText(vis, '%.1fm' % p.width, px(p.x + 0.5, p.y + 0.5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
cv2.circle(vis, px(*robot), 6, (255, 0, 255), -1)
cv2.imwrite(sys.argv[2] if len(sys.argv) > 2 else 'geom.png', vis)

#!/usr/bin/env python3
"""ROS node that analyses the cave geometry each time the map is updated.

Publishes the distance transform as a grid and the detected features as
RViz markers. Runs alongside cave_explorer_node; it only reads the map.

Owner: Karan Sharma
Task: Advanced 2
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from cave_explorer.advanced import cave_geometry


class CaveGeometryNode(Node):
    def __init__(self):
        super().__init__('cave_geometry_node')

        self.declare_parameter('robot_radius', 0.5)
        self.declare_parameter('max_passages', 5)

        # Latest analysis, and what was last reported to the terminal
        self.result_ = None
        self.reported_widest_ = None
        self.reported_narrowest_ = None
        self.previous_passages_ = []

        # Prepare transformation to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.distance_transform_pub_ = self.create_publisher(OccupancyGrid, 'cave_geometry/distance_transform', 1)
        self.marker_pub_ = self.create_publisher(MarkerArray, 'cave_geometry/markers', 1)
        self.map_sub_ = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)

    def get_robot_xy(self):
        """Robot position in the map frame, or None if not yet available"""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def map_callback(self, map_msg: OccupancyGrid):
        """New map received, so redo the analysis"""
        resolution = map_msg.info.resolution
        origin_xy = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        grid = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)

        self.result_ = cave_geometry.analyse(
            grid, resolution, origin_xy,
            robot_xy=self.get_robot_xy(),
            robot_radius=self.get_parameter('robot_radius').value,
            max_passages=self.get_parameter('max_passages').value)

        # Walls that are only partly mapped can look like a passage for one update.
        # Only keep passages that were also found, at about the same place, last time
        detected = self.result_.passages
        self.result_.passages = [
            p for p in detected
            if any(math.hypot(p.x - q.x, p.y - q.y) < 1.5 for q in self.previous_passages_)]
        self.previous_passages_ = detected

        self.publish_distance_transform(map_msg)
        self.publish_markers(map_msg)
        self.report()

    def publish_distance_transform(self, map_msg):
        """Publish clearance as a grid scaled to 0..100, with -1 outside the accessible space"""
        clearance = self.result_.clearance
        accessible = self.result_.accessible

        scaled = np.full(clearance.shape, -1, dtype=np.int8)
        if accessible.any():
            scaled[accessible] = np.round(100.0 * clearance[accessible] / clearance[accessible].max())

        msg = OccupancyGrid()
        msg.header = map_msg.header
        msg.info = map_msg.info
        msg.data = scaled.ravel().tolist()
        self.distance_transform_pub_.publish(msg)

    def make_marker(self, map_msg, ns, marker_id, marker_type, rgba):
        marker = Marker()
        marker.header.frame_id = map_msg.header.frame_id
        marker.header.stamp = map_msg.header.stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def publish_markers(self, map_msg):
        """Publish the medial axis, widest area and narrow passages"""
        result = self.result_
        resolution = map_msg.info.resolution
        origin_xy = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)

        # Clear the previous markers, since the number of passages changes
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]

        # Medial axis
        ridge = self.make_marker(map_msg, 'ridge', 0, Marker.POINTS, (1.0, 0.85, 0.0, 0.9))
        ridge.scale.x = resolution
        ridge.scale.y = resolution
        for row, col in zip(*np.nonzero(result.ridge)):
            x, y = cave_geometry.cell_to_world(row, col, resolution, origin_xy)
            ridge.points.append(Point(x=x, y=y, z=0.05))
        markers.append(ridge)

        # Widest accessible area
        if result.widest is not None:
            disc = self.make_marker(map_msg, 'widest_area', 0, Marker.CYLINDER, (0.0, 1.0, 1.0, 0.5))
            disc.pose.position.x = float(result.widest.x)
            disc.pose.position.y = float(result.widest.y)
            disc.pose.position.z = 0.3
            disc.scale.x = 2.0 * result.widest.radius
            disc.scale.y = 2.0 * result.widest.radius
            disc.scale.z = 0.05
            markers.append(disc)

            label = self.make_marker(map_msg, 'widest_area', 1, Marker.TEXT_VIEW_FACING, (0.4, 0.7, 1.0, 1.0))
            label.pose.position.x = float(result.widest.x)
            label.pose.position.y = float(result.widest.y)
            label.pose.position.z = 2.0
            label.scale.z = 2.2
            label.text = f'Widest area: radius {result.widest.radius:.1f} m'
            markers.append(label)

        # Narrow passages, the narrowest one highlighted
        for i, passage in enumerate(result.passages):
            rgba = (1.0, 0.0, 0.0, 1.0) if i == 0 else (1.0, 0.5, 0.0, 0.9)

            line = self.make_marker(map_msg, 'passages', 2 * i, Marker.LINE_LIST, rgba)
            line.scale.x = 0.6 if i == 0 else 0.4
            line.points = [Point(x=passage.wall_a[0], y=passage.wall_a[1], z=0.2),
                           Point(x=passage.wall_b[0], y=passage.wall_b[1], z=0.2)]
            markers.append(line)

            label = self.make_marker(map_msg, 'passages', 2 * i + 1, Marker.TEXT_VIEW_FACING, rgba)
            label.pose.position.x = float(passage.x)
            label.pose.position.y = float(passage.y)
            label.pose.position.z = 2.0
            label.scale.z = 2.2 if i == 0 else 1.6
            label.text = f'Narrowest: {passage.width:.1f} m' if i == 0 else f'{passage.width:.1f} m'
            markers.append(label)

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub_.publish(marker_array)

    def report(self):
        """Log the main features when they change"""
        widest = self.result_.widest
        if widest is not None:
            last = self.reported_widest_
            if last is None or abs(widest.radius - last.radius) > 0.3 \
                    or math.hypot(widest.x - last.x, widest.y - last.y) > 3.0:
                self.reported_widest_ = widest
                self.get_logger().info(
                    f'Widest accessible area: centre [{widest.x:.1f}, {widest.y:.1f}], '
                    f'radius {widest.radius:.2f} m, area {math.pi * widest.radius**2:.0f} m^2')

        if self.result_.passages:
            # The map shifts by a cell between updates, so only report a real change
            narrowest = self.result_.passages[0]
            last = self.reported_narrowest_
            if last is None or abs(narrowest.width - last.width) > 0.5 \
                    or math.hypot(narrowest.x - last.x, narrowest.y - last.y) > 3.0:
                self.reported_narrowest_ = narrowest
                self.get_logger().info(
                    f'Narrowest passage: [{narrowest.x:.1f}, {narrowest.y:.1f}], '
                    f'width {narrowest.width:.2f} m, {narrowest.kind}, '
                    f'saddle point: {narrowest.is_saddle} '
                    f'({len(self.result_.passages)} passages found)')


def main():
    # Initialise
    rclpy.init()

    cave_geometry_node = CaveGeometryNode()

    while rclpy.ok():
        rclpy.spin(cave_geometry_node)

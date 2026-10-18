#!/usr/bin/env python3
"""ROS node that drives the robot to sample a measurement field adaptively.

Loop: measure at the current position, update the Gaussian Process, choose the
next location, navigate there with Nav2, repeat. Run this instead of
cave_explorer_node, since both send navigation goals.

Owner: Karan Sharma
Task: Advanced 6
"""

import csv
import math
import os
import subprocess
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from cave_explorer.advanced import cave_geometry
from cave_explorer.advanced.adaptive_sampling import AdaptiveSampler
from cave_explorer.advanced.measurement_field import MeasurementField, default_table_path


def value_colour(fraction):
    """Blue (low) to red (high)"""
    fraction = min(max(fraction, 0.0), 1.0)
    return ColorRGBA(r=fraction, g=0.2, b=1.0 - fraction, a=1.0)


class AdaptiveSamplingNode(Node):
    def __init__(self):
        super().__init__('adaptive_sampling_node')

        self.declare_parameter('strategy', 'adaptive')
        self.declare_parameter('num_samples', 30)
        self.declare_parameter('seed', 0)
        self.declare_parameter('travel_weight', 0.3)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('goal_clearance', 1.5)
        self.declare_parameter('stuck_time', 15.0)
        self.declare_parameter('settle_time', 1.0)
        self.declare_parameter('measurement_table', '')
        self.declare_parameter('log_directory', '~/cave_sampling_logs')
        self.declare_parameter('show_results', True)   # open the results map in an image viewer when finished

        self.strategy_ = self.get_parameter('strategy').value
        self.num_samples_ = self.get_parameter('num_samples').value
        seed = self.get_parameter('seed').value

        # The field being measured. Only measure() is used for deciding where to go;
        # ground_truth() is used in evaluate() to score the result.
        table = self.get_parameter('measurement_table').value or default_table_path()
        self.field_ = MeasurementField(table, seed=seed)
        self.sampler_ = AdaptiveSampler(strategy=self.strategy_, seed=seed,
                                        travel_weight=self.get_parameter('travel_weight').value,
                                        goal_clearance=self.get_parameter('goal_clearance').value)

        # Latest map
        self.grid_ = None
        self.resolution_ = None
        self.origin_xy_ = None

        # Progress
        self.navigating_ = False
        self.finished_ = False
        self.goal_xy_ = None
        self.goal_handle_ = None
        self.goal_sent_time_ = None
        self.goal_attempts_ = 0
        self.retry_goal_ = False
        self.navigation_ended_time_ = None
        self.progress_xy_ = None
        self.progress_time_ = None
        self.recovery_start_time_ = None
        self.travelled_ = 0.0
        self.last_xy_ = None
        self.path_ = []                # robot positions, for the results map
        self.last_maps_ = None         # (walls, accessible, GP mean, GP std, true field) from the latest evaluation
        self.results_image_ = None     # results map, shown in the Detections panel once finished
        self.log_rows_ = []
        self.last_sample_ = None       # (count, x, y, value, rmse, mean uncertainty), shown on the camera image
        self.next_goal_info_ = None    # (x, y, predicted value, predicted uncertainty) of the location being driven to

        # Prepare transformation to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for nav2
        self.nav2_action_client_ = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().warn('Waiting for navigate_to_pose action...')
        self.nav2_action_client_.wait_for_server()
        self.get_logger().warn('navigate_to_pose connected')

        self.goal_pose_vis_ = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.cmd_vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.mean_pub_ = self.create_publisher(OccupancyGrid, 'adaptive_sampling/mean', 1)
        self.uncertainty_pub_ = self.create_publisher(OccupancyGrid, 'adaptive_sampling/uncertainty', 1)
        self.ground_truth_pub_ = self.create_publisher(OccupancyGrid, 'adaptive_sampling/ground_truth', 1)
        self.marker_pub_ = self.create_publisher(MarkerArray, 'adaptive_sampling/markers', 1)
        self.map_sub_ = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        # Camera image with the sampling progress written on it, shown in the RViz Detections panel
        self.overlay_pub_ = self.create_publisher(Image, 'detections_image', 1)
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)

        self.get_logger().info(f'Sampling strategy: {self.strategy_}, {self.num_samples_} samples, seed {seed}')

        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.5, self.main_loop)

        # Timer for driving the robot directly when it is stuck
        self.recovery_timer_ = self.create_timer(0.05, self.recovery_loop)

    def get_robot_xy(self):
        """Robot position in the map frame, or None if not yet available"""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def image_callback(self, image_msg):
        """Write the latest sample and the next sampling location on the camera image for the RViz Detections panel"""
        if image_msg.encoding not in ('rgb8', 'bgr8'):
            return
        if self.results_image_ is not None:
            out = Image()
            out.header = image_msg.header
            out.height, out.width = self.results_image_.shape[:2]
            out.encoding, out.is_bigendian, out.step = 'bgr8', 0, self.results_image_.shape[1] * 3
            out.data = self.results_image_.tobytes()
            self.overlay_pub_.publish(out)
            return
        image = np.frombuffer(bytes(image_msg.data), np.uint8).reshape(image_msg.height, image_msg.width, 3).copy()
        title = 'Adaptive sampling (GP)' if self.strategy_ == 'adaptive' else f'Baseline: {self.strategy_} sampling'
        lines = [f'{title}: {len(self.sampler_.sample_values_)}/{self.num_samples_} samples']
        if self.last_sample_ is not None:
            count, x, y, value, rmse, mean_std = self.last_sample_
            lines.append(f'Sample {count} at ({x:.1f}, {y:.1f}): {value:.2f} wt% H2O')
            lines.append(f'Map error (RMSE) {rmse:.3f}   mean uncertainty {mean_std:.3f}')
        lines.append(f'Distance travelled {self.travelled_:.0f} m')
        if self.finished_:
            lines.append('Finished')
        elif self.next_goal_info_ is not None:
            x, y, mean, std, fitted = self.next_goal_info_
            lines.append(f'Driving to ({x:.1f}, {y:.1f})' + (f': GP predicts {mean:.2f} +- {std:.2f}' if fitted else ''))
        height = 28 * len(lines) + 12
        band = image[:height].astype(np.float32)
        image[:height] = (0.35 * band).astype(np.uint8)
        for i, text in enumerate(lines):
            cv2.putText(image, text, (10, 28 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        out = Image()
        out.header = image_msg.header
        out.height, out.width, out.encoding = image_msg.height, image_msg.width, image_msg.encoding
        out.is_bigendian, out.step = 0, image_msg.width * 3
        out.data = image.tobytes()
        self.overlay_pub_.publish(out)

    def map_callback(self, map_msg: OccupancyGrid):
        """New map received"""
        self.resolution_ = map_msg.info.resolution
        self.origin_xy_ = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        self.grid_ = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
        self.map_info_ = map_msg.info
        self.map_frame_ = map_msg.header.frame_id

    def main_loop(self):
        """Take a sample when the robot has arrived, then send it to the next location"""
        robot_xy = self.get_robot_xy()
        if robot_xy is None or self.grid_ is None:
            self.get_logger().warn('Waiting for map and transform... Have you launched a SLAM node?',
                                   throttle_duration_sec=5.0)
            return

        # Keep track of distance travelled
        if self.last_xy_ is not None:
            self.travelled_ += math.hypot(robot_xy[0] - self.last_xy_[0], robot_xy[1] - self.last_xy_[1])
        self.last_xy_ = robot_xy
        if not self.path_ or math.hypot(robot_xy[0] - self.path_[-1][0], robot_xy[1] - self.path_[-1][1]) > 0.3:
            self.path_.append(robot_xy)

        if self.finished_:
            return

        if self.recovery_start_time_ is not None:
            return

        now = self.get_clock().now()

        if self.navigating_:
            # Stuck: the robot has not moved for a while, e.g. wedged on an object the laser cannot see
            if math.hypot(robot_xy[0] - self.progress_xy_[0], robot_xy[1] - self.progress_xy_[1]) > 0.5:
                self.progress_xy_ = robot_xy
                self.progress_time_ = now
            elif (now - self.progress_time_).nanoseconds / 1e9 > self.get_parameter('stuck_time').value:
                self.get_logger().warn(f'Stuck at [{robot_xy[0]:.1f}, {robot_xy[1]:.1f}], backing up')
                self.cancel_goal()
                self.sampler_.blacklist(robot_xy[0], robot_xy[1])
                self.goal_failed()
                self.recovery_start_time_ = now
                return

            # Give up on goals that take too long
            if (now - self.goal_sent_time_).nanoseconds / 1e9 > self.get_parameter('goal_timeout').value:
                self.get_logger().warn(f'Goal [{self.goal_xy_[0]:.1f}, {self.goal_xy_[1]:.1f}] timed out')
                self.cancel_goal()
                self.goal_failed()
            return

        # Nav2 aborts a goal that arrives while it is still finishing the previous one
        if self.navigation_ended_time_ is not None and \
                (now - self.navigation_ended_time_).nanoseconds / 1e9 < self.get_parameter('settle_time').value:
            return

        if self.retry_goal_:
            self.retry_goal_ = False
            self.go_to(self.goal_xy_, robot_xy)
            return

        self.take_sample(robot_xy)

        if len(self.sampler_.sample_values_) >= self.num_samples_:
            self.finish()
            return

        goal_xy = self.sampler_.choose_next(self.grid_, self.resolution_, self.origin_xy_, robot_xy)
        if goal_xy is None:
            self.get_logger().warn('No reachable location left to sample', throttle_duration_sec=5.0)
            return
        self.goal_attempts_ = 0
        self.go_to(goal_xy, robot_xy)

    def recovery_loop(self):
        """Free a stuck robot: reverse, then turn right on the spot"""
        if self.recovery_start_time_ is None:
            return

        elapsed = (self.get_clock().now() - self.recovery_start_time_).nanoseconds / 1e9
        command = Twist()
        if elapsed < 4.0:
            command.linear.x = -0.8
        elif elapsed < 6.0:
            command.angular.z = -1.0
        else:
            self.recovery_start_time_ = None
            self.navigation_ended_time_ = self.get_clock().now()
        self.cmd_vel_pub_.publish(command)

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.goal_handle_.cancel_goal_async()

    def take_sample(self, robot_xy):
        """Measure at the robot's position, unless it is too close to an existing sample"""
        samples = self.sampler_.sample_xy_
        if samples and min(math.hypot(robot_xy[0] - x, robot_xy[1] - y) for x, y in samples) \
                < 0.5 * self.sampler_.min_sample_spacing_:
            return

        value = self.field_.measure(robot_xy[0], robot_xy[1])
        self.sampler_.add_sample(robot_xy[0], robot_xy[1], value)

        rmse, mean_std = self.evaluate(robot_xy)
        count = len(self.sampler_.sample_values_)
        self.last_sample_ = (count, robot_xy[0], robot_xy[1], value, rmse, mean_std)
        self.log_rows_.append([count, self.get_clock().now().nanoseconds / 1e9,
                               robot_xy[0], robot_xy[1], value, self.travelled_, rmse, mean_std])
        self.get_logger().info(
            f'Sample {count}/{self.num_samples_} at [{robot_xy[0]:.1f}, {robot_xy[1]:.1f}]: '
            f'{value:.2f} wt% H2O | RMSE {rmse:.3f}, mean uncertainty {mean_std:.3f}, '
            f'travelled {self.travelled_:.0f} m')

    def evaluate(self, robot_xy):
        """Score the GP against ground truth over the explored cave, and publish the GP maps.

        Evaluation only: nothing computed here is passed to the sampler.
        """
        clearance, _, _, free, _ = cave_geometry.distance_transforms(self.grid_, self.resolution_)
        robot_cell = cave_geometry.world_to_cell(robot_xy[0], robot_xy[1], self.resolution_, self.origin_xy_)
        accessible = cave_geometry.accessible_region(free, clearance, self.sampler_.robot_radius_, robot_cell)
        mean, std = self.sampler_.predict_over(accessible, self.resolution_, self.origin_xy_)

        self.publish_grid(self.mean_pub_, mean)
        self.publish_grid(self.uncertainty_pub_, std)
        self.publish_markers()

        if not accessible.any():
            return float('nan'), float('nan')
        rows, cols = np.nonzero(accessible)
        x, y = cave_geometry.cell_to_world(rows, cols, self.resolution_, self.origin_xy_)
        truth = self.field_.ground_truth(x, y)

        # Show the true field too, for comparing against the GP mean by eye
        truth_grid = np.full(accessible.shape, np.nan)
        truth_grid[rows, cols] = truth
        self.publish_grid(self.ground_truth_pub_, truth_grid)
        self.last_maps_ = (self.grid_ > 50, accessible, mean, std, truth_grid)

        return float(np.sqrt(np.mean((mean[rows, cols] - truth)**2))), float(np.mean(std[rows, cols]))

    def publish_grid(self, publisher, values):
        """Publish a grid of values scaled to 0..100 over their own range, -1 where nan"""
        scaled = np.full(values.shape, -1, dtype=np.int8)
        valid = np.isfinite(values)
        if valid.any():
            low, high = values[valid].min(), values[valid].max()
            scaled[valid] = np.round(100.0 * (values[valid] - low) / max(high - low, 1e-9))

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame_
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_info_
        msg.data = scaled.ravel().tolist()
        publisher.publish(msg)

    def publish_markers(self):
        """Publish the sample locations coloured by measured value, and the path between them"""
        values = np.array(self.sampler_.sample_values_)
        low, high = values.min(), values.max()

        samples = Marker()
        samples.header.frame_id = self.map_frame_
        samples.ns = 'samples'
        samples.id = 0
        samples.type = Marker.SPHERE_LIST
        samples.action = Marker.ADD
        samples.pose.orientation.w = 1.0
        samples.scale.x = 1.5
        samples.scale.y = 1.5
        samples.scale.z = 1.5

        path = Marker()
        path.header.frame_id = self.map_frame_
        path.ns = 'sample_order'
        path.id = 0
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = 0.2
        path.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)

        markers = [samples, path]
        for i, ((x, y), value) in enumerate(zip(self.sampler_.sample_xy_, values)):
            samples.points.append(Point(x=x, y=y, z=0.5))
            samples.colors.append(value_colour((value - low) / max(high - low, 1e-9)))
            path.points.append(Point(x=x, y=y, z=0.3))

            label = Marker()
            label.header.frame_id = self.map_frame_
            label.ns = 'sample_values'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 2.5
            label.pose.orientation.w = 1.0
            label.scale.z = 1.6
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'{value:.1f}'
            markers.append(label)

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub_.publish(marker_array)

    def go_to(self, goal_xy, robot_xy):
        """Send the robot to the next sampling location, facing the direction of travel"""
        action_goal = NavigateToPose.Goal()
        action_goal.pose.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose.header.frame_id = 'map'
        heading = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
        action_goal.pose.pose.position.x = goal_xy[0]
        action_goal.pose.pose.position.y = goal_xy[1]
        action_goal.pose.pose.orientation.w = math.cos(heading / 2.0)
        action_goal.pose.pose.orientation.z = math.sin(heading / 2.0)

        # Publish visualisation
        self.goal_pose_vis_.publish(action_goal.pose)

        self.get_logger().info(f'Next sampling location [{goal_xy[0]:.1f}, {goal_xy[1]:.1f}]')
        mean, std = self.sampler_.predict(np.array([goal_xy]))
        self.next_goal_info_ = (goal_xy[0], goal_xy[1], float(mean[0]), float(std[0]), self.sampler_.gp_ is not None)
        self.navigating_ = True
        self.goal_xy_ = goal_xy
        self.goal_handle_ = None
        self.goal_attempts_ += 1
        self.goal_sent_time_ = self.get_clock().now()
        self.progress_xy_ = robot_xy
        self.progress_time_ = self.goal_sent_time_
        self.nav2_action_client_.send_goal_async(action_goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """The requested goal pose has been sent to the action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.goal_failed()
            return

        self.goal_handle_ = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda future: self.goal_result_callback(future, goal_handle))

    def goal_result_callback(self, future, goal_handle):
        """Navigation has ended, successfully or not"""
        if not self.navigating_ or goal_handle is not self.goal_handle_:
            # A goal that was already given up on (timeout or stuck)
            return

        now = self.get_clock().now()
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.navigating_ = False
            self.navigation_ended_time_ = now
        elif (now - self.goal_sent_time_).nanoseconds / 1e9 < 3.0 and self.goal_attempts_ < 3:
            # Aborted straight away: usually Nav2 still finishing the previous goal, so try again
            self.navigating_ = False
            self.navigation_ended_time_ = now
            self.retry_goal_ = True
        else:
            self.get_logger().warn(f'Could not reach [{self.goal_xy_[0]:.1f}, {self.goal_xy_[1]:.1f}]')
            self.goal_failed()

    def goal_failed(self):
        """Avoid this goal in future; the main loop then samples wherever the robot ended up"""
        self.sampler_.blacklist(self.goal_xy_[0], self.goal_xy_[1])
        self.navigating_ = False
        self.goal_handle_ = None
        self.navigation_ended_time_ = self.get_clock().now()

    def finish(self):
        """All samples taken: save the log"""
        self.finished_ = True

        directory = os.path.expanduser(self.get_parameter('log_directory').value)
        os.makedirs(directory, exist_ok=True)
        filename = os.path.join(directory, f'{self.strategy_}_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['sample', 'time_s', 'x', 'y', 'h2o_wt_percent', 'travelled_m', 'rmse', 'mean_std'])
            writer.writerows(self.log_rows_)

        self.get_logger().info(
            f'Finished: {len(self.log_rows_)} samples, travelled {self.travelled_:.0f} m, '
            f'final RMSE {self.log_rows_[-1][6]:.3f}. Log saved to {filename}')

        if self.last_maps_ is not None:
            self.results_image_ = self.results_map()
            picture = filename[:-4] + '.png'
            cv2.imwrite(picture, self.results_image_)
            self.get_logger().info(f'Results map saved to {picture}')
            if self.get_parameter('show_results').value:
                try:
                    subprocess.Popen(['xdg-open', picture], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError as exc:
                    self.get_logger().warn(f'Could not open the results map: {exc}')

    def results_map(self):
        """Picture of the result: true field, GP estimate and GP uncertainty side by side, with walls, path and samples"""
        walls, accessible, mean, std, truth = self.last_maps_
        rows, cols = np.nonzero(walls | accessible)
        top, bottom = max(rows.min() - 3, 0), min(rows.max() + 4, walls.shape[0])
        left, right = max(cols.min() - 3, 0), min(cols.max() + 4, walls.shape[1])
        scale = 560.0 / (right - left)                                    # each panel 560 pixels wide
        valid = np.isfinite(truth) & accessible
        low, high = np.nanmin(truth[valid]), np.nanmax(truth[valid])      # same colour scale for truth and estimate

        def to_pixel(x, y):
            r, c = cave_geometry.world_to_cell(x, y, self.resolution_, self.origin_xy_)
            return int((c - left + 0.5) * scale), int((bottom - 1 - r + 0.5) * scale)   # north up

        def panel(values, vmin, vmax, title, colormap):
            v = np.clip((values - vmin) / max(vmax - vmin, 1e-9), 0, 1)
            v = np.nan_to_num(v)
            image = cv2.applyColorMap((255 * v).astype(np.uint8), colormap)
            image[~accessible] = (60, 60, 60)
            image[walls] = (0, 0, 0)
            image = image[top:bottom, left:right][::-1]
            image = cv2.resize(image, (int(round((right - left) * scale)), int(round((bottom - top) * scale))),
                               interpolation=cv2.INTER_NEAREST)
            for a, b in zip(self.path_[:-1], self.path_[1:]):
                cv2.line(image, to_pixel(*a), to_pixel(*b), (255, 255, 255), 2, cv2.LINE_AA)
            for k, (x, y) in enumerate(self.sampler_.sample_xy_):
                cv2.circle(image, to_pixel(x, y), 6, (255, 255, 255), -1)
                cv2.circle(image, to_pixel(x, y), 6, (0, 0, 0), 2)
                cv2.putText(image, str(k + 1), (to_pixel(x, y)[0] + 7, to_pixel(x, y)[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            band = np.full((34, image.shape[1], 3), 255, np.uint8)
            cv2.putText(band, title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
            return np.vstack([band, image])

        panels = [panel(truth, low, high, f'True water (NASA data) {low:.1f}-{high:.1f} wt%', cv2.COLORMAP_VIRIDIS),
                  panel(mean, low, high, 'GP estimate (same colour scale)', cv2.COLORMAP_VIRIDIS),
                  panel(std, np.nanmin(std[accessible]), np.nanmax(std[accessible]), 'GP uncertainty (bright = unsure)',
                        cv2.COLORMAP_INFERNO)]
        gap = np.full((panels[0].shape[0], 8, 3), 255, np.uint8)
        figure = np.hstack([panels[0], gap, panels[1], gap, panels[2]])
        last = self.log_rows_[-1]
        lines = [f'Adaptive sampling result ({self.strategy_}): {len(self.log_rows_)} samples, {self.travelled_:.0f} m driven',
                 f'Map error (RMSE) {last[6]:.3f} wt% H2O, mean uncertainty {last[7]:.3f}. '
                 'White: path and numbered samples. Black: walls.']
        header = np.full((70, figure.shape[1], 3), 255, np.uint8)
        for k, text in enumerate(lines):
            cv2.putText(header, text, (10, 28 + 30 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.75 if k == 0 else 0.6, (0, 0, 0),
                        2 if k == 0 else 1, cv2.LINE_AA)
        return np.vstack([header, figure])


def main():
    # Initialise
    rclpy.init()

    adaptive_sampling_node = AdaptiveSamplingNode()

    while rclpy.ok():
        rclpy.spin(adaptive_sampling_node)

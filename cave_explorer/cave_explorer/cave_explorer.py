#!/usr/bin/env python3

import math
import random
from enum import Enum
from copy import deepcopy

import cv2  # OpenCV2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def wrap_angle(angle):
    """Function to wrap an angle between 0 and 2*Pi"""
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    """Convert a Pose2D to a full 3D Pose"""
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(pose_2d.theta / 2.0)
    pose.orientation.z = math.sin(pose_2d.theta / 2.0)

    return pose


class PlannerType(Enum):
    ERROR = 0
    MOVE_FORWARDS = 1
    RETURN_HOME = 2
    GO_TO_FIRST_ARTIFACT = 3
    RANDOM_WALK = 4
    RANDOM_GOAL = 5
    FRONTIER_EXPLORATION = 6
    # Add more!


class CaveExplorer(Node):
    def __init__(self):
        super().__init__('cave_explorer_node')

        # Variables/Flags for mapping
        self.xlim_ = [0.0, 0.0]
        self.ylim_ = [0.0, 0.0]
        self.map_msg_ = None
        self.visited_frontiers_ = []
        # Variables/Flags for perception
        self.artifact_found_ = False
        # Planning 2/3: explore -> inspect -> explore state machine.
        self.inspection_standoff_ = 1.5
        self.estimated_artifact_distance_ = 3.0  # Approximate range, NOT measured depth.
        self.camera_horizontal_fov_ = math.radians(60.0)  # Calibrate for actual camera.
        self.artifact_merge_radius_ = 2.0
        self.inspection_timeout_ = 45.0
        self.target_lost_timeout_ = 8.0
        self.max_inspection_retries_ = 1
        self.state_ = 'EXPLORE'
        self.artifact_locations_ = []
        self.artifact_status_ = []  # pending / inspected / abandoned
        self.active_artifact_index_ = None
        self.inspection_retries_ = 0
        self.inspection_started_at_ = None
        self.last_target_seen_at_ = None
        self.goal_kind_ = None
        self.goal_request_pending_ = False
        self.cancel_requested_ = False
        self.cancel_in_progress_ = False
        self.goal_generation_ = 0
        self.active_goal_generation_ = None
        self.detection_streak_ = 0
        self.artifact_found_ = False
        self.ready_for_next_goal_ = True
        self.current_goal_handle_ = None
        # Variables/Flags for planning
        self.planner_type_ = PlannerType.ERROR
        self.reached_first_artifact_ = False
        self.returned_home_ = False

        # Marker for artifact locations
        # See https://wiki.ros.org/rviz/DisplayTypes/Marker
        self.marker_artifacts_ = Marker()
        self.marker_artifacts_.header.frame_id = "map"
        self.marker_artifacts_.ns = "artifacts"
        self.marker_artifacts_.id = 0
        self.marker_artifacts_.type = Marker.SPHERE_LIST
        self.marker_artifacts_.action = Marker.ADD
        self.marker_artifacts_.pose.position.x = 0.0
        self.marker_artifacts_.pose.position.y = 0.0
        self.marker_artifacts_.pose.position.z = 0.0
        self.marker_artifacts_.pose.orientation.x = 0.0
        self.marker_artifacts_.pose.orientation.y = 0.0
        self.marker_artifacts_.pose.orientation.z = 0.0
        self.marker_artifacts_.pose.orientation.w = 1.0
        self.marker_artifacts_.scale.x = 0.7
        self.marker_artifacts_.scale.y = 0.7
        self.marker_artifacts_.scale.z = 0.7
        self.marker_artifacts_.color.a = 1.0
        self.marker_artifacts_.color.r = 0.0
        self.marker_artifacts_.color.g = 1.0
        self.marker_artifacts_.color.b = 0.2
        self.marker_pub_ = self.create_publisher(MarkerArray, 'marker_array_artifacts', 10)

        # Remember the artifact locations
        # Array of type geometry_msgs.Point
        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Prepare transformation to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for nav2
        self.nav2_action_client_ = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().warn('Waiting for navigate_to_pose action...')
        self.nav2_action_client_.wait_for_server()
        self.get_logger().warn('navigate_to_pose connected')
        self.declare_parameter('print_feedback', rclpy.Parameter.Type.BOOL)

        # Publisher for the goal pose visualisation
        self.goal_pose_vis_ = self.create_publisher(PoseStamped, 'goal_pose', 1)

        # Subscribe to the map topic to get current bounds
        self.map_sub_ = self.create_subscription(OccupancyGrid, 'map',  self.map_callback, 1)

        # Prepare image processing
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
        self.declare_parameter('computer_vision_model_filename', rclpy.Parameter.Type.STRING)
        self.computer_vision_model_ = cv2.CascadeClassifier(self.get_parameter('computer_vision_model_filename').value)
        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)

        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.2, self.main_loop)
    
    def get_pose_2d(self):
        """Get the 2d pose of the robot"""

        # Lookup the latest transform
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = t.transform.translation.x
        pose.y = t.transform.translation.y

        qw = t.transform.rotation.w
        qz = t.transform.rotation.z

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw))

        self.get_logger().warn(f'Pose: {pose}')

        return pose

    def map_callback(self, map_msg: OccupancyGrid):
        """New map received, so update x and y limits"""
        self.map_msg_ = map_msg
        # Extract data from message
        map_origin = [map_msg.info.origin.position.x, 
                      map_msg.info.origin.position.y]
        map_resolution = map_msg.info.resolution
        map_height = map_msg.info.height
        map_width = map_msg.info.width

        # Set current limits
        self.xlim_ = [map_origin[0], map_origin[0]+map_width*map_resolution]
        self.ylim_ = [map_origin[1], map_origin[1]+map_height*map_resolution]

        # self.get_logger().warn('Map received:')
        # self.get_logger().warn(f'  xlim = [{self.xlim_[0]:.2f}, {self.xlim_[1]:.2f}]')
        # self.get_logger().warn(f'  ylim = [{self.ylim_[0]:.2f}, {self.ylim_[1]:.2f}]')
    def find_frontiers(self):
        """Find free map cells next to unknown space."""

        if self.map_msg_ is None:
            return []

        width = self.map_msg_.info.width
        height = self.map_msg_.info.height
        data = self.map_msg_.data

        frontiers = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x

                # Only consider free cells
                if data[idx] != 0:
                    continue

                neighbours = [
                    data[idx - 1],
                    data[idx + 1],
                    data[idx - width],
                    data[idx + width]
                ]

                # Frontier = free cell beside unknown space
                if -1 in neighbours:
                    frontiers.append((x, y))

        return frontiers
    def planner_frontier_exploration(self):
        """Choose a frontier and navigate towards it."""

        if self.map_msg_ is None:
            self.get_logger().warn('No map received yet.')
            self.ready_for_next_goal_ = True
            return

        robot_pose = self.get_pose_2d()

        if robot_pose is None:
            self.ready_for_next_goal_ = True
            return

        frontiers = self.find_frontiers()

        if len(frontiers) == 0:
            self.get_logger().warn('No frontiers found.')
            self.ready_for_next_goal_ = True
            return

        resolution = self.map_msg_.info.resolution
        origin_x = self.map_msg_.info.origin.position.x
        origin_y = self.map_msg_.info.origin.position.y

        best_goal = None
        best_distance = float('inf')

        for x, y in frontiers:

            world_x = origin_x + (x + 0.5) * resolution
            world_y = origin_y + (y + 0.5) * resolution

            distance = math.sqrt(
                (world_x - robot_pose.x) ** 2 +
                (world_y - robot_pose.y) ** 2
            )

            too_close_to_old_goal = False

            for old_x, old_y in self.visited_frontiers_:
                old_distance = math.sqrt(
                    (world_x - old_x) ** 2 +
                    (world_y - old_y) ** 2
                )

                if old_distance < 1.5:
                    too_close_to_old_goal = True
                    break

            if too_close_to_old_goal:
                continue

            if distance < best_distance:
                best_distance = distance
                best_goal = (world_x, world_y)

        if best_goal is None:
            self.get_logger().warn('No unused frontier found.')
            self.ready_for_next_goal_ = True
            return

        goal_pose = Pose2D()
        goal_pose.x = best_goal[0]
        goal_pose.y = best_goal[1]

        goal_pose.theta = math.atan2(
            goal_pose.y - robot_pose.y,
            goal_pose.x - robot_pose.x
        )

        self.visited_frontiers_.append(best_goal)

        self.get_logger().info(
            f'Frontier goal: ({goal_pose.x:.2f}, {goal_pose.y:.2f})'
        )

        self.planner_go_to_pose2d(goal_pose)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def image_callback(self, image_msg):
        """Detect chosen artefact, estimate bearing, and maintain target visibility."""
        try:
            image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if self.computer_vision_model_.empty():
                self.get_logger().error('Cascade model not loaded; check model filename')
                return
            detections = self.computer_vision_model_.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=4, minSize=(20, 20))
        except Exception as exc:
            self.get_logger().warn(f'Image detection error: {exc}')
            return

        self.artifact_found_ = len(detections) > 0
        self.detection_streak_ = self.detection_streak_ + 1 if self.artifact_found_ else 0
        for x, y, w, h in detections:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        self.image_detections_pub_.publish(
            self.cv_bridge_.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), encoding='rgb8'))

        if not self.artifact_found_:
            return
        # Choose largest detected box; the cascade is for one configured artefact type.
        x, y, w, h = max(detections, key=lambda b: b[2] * b[3])
        center_x = x + w / 2.0
        relative_bearing = ((image.shape[1] / 2.0 - center_x) /
                            image.shape[1]) * self.camera_horizontal_fov_
        if self.state_ == 'INSPECT' and self.active_artifact_index_ is not None:
            self.last_target_seen_at_ = self.now_seconds()
        if self.state_ == 'EXPLORE' and self.detection_streak_ >= 2:
            self.localise_artifact(relative_bearing)

    def localise_artifact(self, bearing=0.0):
        """Approximate map-frame target location from camera bearing and assumed range."""
        robot = self.get_pose_2d()
        if robot is None:
            return
        theta = robot.theta + bearing
        p = Point()
        p.x = robot.x + self.estimated_artifact_distance_ * math.cos(theta)
        p.y = robot.y + self.estimated_artifact_distance_ * math.sin(theta)
        p.z = 0.5
        for i, old in enumerate(self.artifact_locations_):
            if math.hypot(old.x - p.x, old.y - p.y) < self.artifact_merge_radius_:
                # Do not revisit already inspected/abandoned targets.
                return
        self.artifact_locations_.append(p)
        self.artifact_status_.append('pending')
        self.active_artifact_index_ = len(self.artifact_locations_) - 1
        self.inspection_retries_ = 0
        self.last_target_seen_at_ = self.now_seconds()
        self.state_ = 'PREEMPT'
        self.get_logger().info(f'New artefact #{self.active_artifact_index_}: stop exploring')
        self.publish_artifact_markers()
        self.cancel_current_goal()

    def publish_artifact_markers(self):
        """Yellow=pending, green=inspected, red=abandoned in RViz."""
        array = MarkerArray()
        for i, (p, status) in enumerate(zip(self.artifact_locations_, self.artifact_status_)):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'artifact_inspection_status'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position = p
            m.scale.x = m.scale.y = m.scale.z = 0.8
            m.color.a = 1.0
            if status == 'inspected':
                m.color.g = 1.0
            elif status == 'abandoned':
                m.color.r = 1.0
            else:
                m.color.r = m.color.g = 1.0
            array.markers.append(m)
        self.marker_pub_.publish(array)

    def planner_go_to_pose2d(self, pose2d, kind='frontier'):
        """Send a single Nav2 goal; callbacks track the matching request."""
        if self.goal_request_pending_ or self.current_goal_handle_ is not None:
            return False
        action_goal = NavigateToPose.Goal()
        action_goal.pose.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose.header.frame_id = 'map'
        action_goal.pose.pose = pose2d_to_pose(pose2d)
        self.goal_pose_vis_.publish(action_goal.pose)
        self.goal_kind_ = kind
        self.goal_generation_ += 1
        generation = self.goal_generation_
        self.goal_request_pending_ = True
        self.ready_for_next_goal_ = False
        self.get_logger().info(f'Sending {kind} goal ({pose2d.x:.2f}, {pose2d.y:.2f})')
        try:
            feedback_cb = self.feedback_callback if self.get_parameter('print_feedback').value else None
            future = self.nav2_action_client_.send_goal_async(action_goal, feedback_callback=feedback_cb)
            future.add_done_callback(lambda f, gen=generation: self.goal_response_callback(f, gen))
        except Exception as exc:
            self.goal_request_pending_ = False
            self.ready_for_next_goal_ = True
            self.get_logger().error(f'Failed to send navigation goal: {exc}')
            return False
        return True

    def goal_response_callback(self, future, generation):
        self.goal_request_pending_ = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Goal request failed: {exc}')
            self.navigation_failed()
            return
        if not handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.navigation_failed()
            return
        self.current_goal_handle_ = handle
        self.active_goal_generation_ = generation
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, h=handle, gen=generation: self.goal_reached_callback(f, h, gen))
        if self.cancel_requested_:
            self.cancel_current_goal()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'{feedback_msg.feedback.distance_remaining:.2f} m remaining')

    def navigation_failed(self):
        self.current_goal_handle_ = None
        self.cancel_requested_ = False
        self.cancel_in_progress_ = False
        self.ready_for_next_goal_ = True
        if self.state_ == 'PREEMPT':
            self.state_ = 'INSPECT'
        elif self.state_ == 'INSPECT':
            self.retry_or_abandon('navigation failed')

    def goal_reached_callback(self, future, handle, generation):
        if handle is not self.current_goal_handle_ or generation != self.active_goal_generation_:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f'Navigation result error: {exc}')
            status = None
        kind = self.goal_kind_
        self.current_goal_handle_ = None
        self.active_goal_generation_ = None
        self.cancel_requested_ = False
        self.cancel_in_progress_ = False
        self.ready_for_next_goal_ = True
        if self.state_ == 'PREEMPT':
            self.state_ = 'INSPECT'
            return
        if kind == 'inspection' and self.state_ == 'INSPECT':
            if status == GoalStatus.STATUS_SUCCEEDED:
                i = self.active_artifact_index_
                self.artifact_status_[i] = 'inspected'
                self.reached_first_artifact_ = True
                self.get_logger().info(f'Planning 2 complete; inspected artefact #{i}; resume exploration')
                self.publish_artifact_markers()
                self.active_artifact_index_ = None
                self.state_ = 'EXPLORE'
                self.detection_streak_ = 0
            else:
                self.retry_or_abandon(f'Nav2 status {status}')
        elif kind == 'frontier' and status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Frontier navigation finished with status {status}')

    def cancel_current_goal(self):
        """Wait for frontier cancellation to complete before sending inspection goal."""
        self.cancel_requested_ = True
        if self.current_goal_handle_ is None or self.cancel_in_progress_:
            return
        self.cancel_in_progress_ = True
        try:
            f = self.current_goal_handle_.cancel_goal_async()
            f.add_done_callback(self.cancel_response_callback)
        except Exception as exc:
            self.get_logger().warn(f'Cancel request failed: {exc}')
            self.cancel_in_progress_ = False

    def cancel_response_callback(self, future):
        self.cancel_in_progress_ = False
        try:
            self.get_logger().info(f'Cancel requested: {len(future.result().goals_canceling)} goals cancelling')
        except Exception as exc:
            self.get_logger().warn(f'Cancel response failed: {exc}')
        # The navigation result callback releases the active goal.

    def retry_or_abandon(self, reason):
        if self.active_artifact_index_ is None:
            self.state_ = 'EXPLORE'
            return
        if self.inspection_retries_ < self.max_inspection_retries_:
            self.inspection_retries_ += 1
            self.inspection_started_at_ = None
            self.last_target_seen_at_ = self.now_seconds()
            self.get_logger().warn(f'Inspection {reason}; retry {self.inspection_retries_}/1')
            return
        i = self.active_artifact_index_
        self.artifact_status_[i] = 'abandoned'
        self.publish_artifact_markers()
        self.get_logger().warn(f'Inspection {reason} twice; abandon #{i} and explore')
        self.active_artifact_index_ = None
        self.inspection_started_at_ = None
        self.detection_streak_ = 0
        self.state_ = 'EXPLORE'

    def planner_move_forwards(self, distance):
        """Simply move forward by the specified distance"""

        pose_2d = self.get_pose_2d()

        pose_2d.x += distance * math.cos(pose_2d.theta)
        pose_2d.y += distance * math.sin(pose_2d.theta)

        self.planner_go_to_pose2d(pose_2d)
    def get_close_range_inspection_goal(self, artifact):
        """Stop at a standoff distance facing the estimated artefact."""
        robot = self.get_pose_2d()
        if robot is None:
            return None
        dx, dy = artifact.x - robot.x, artifact.y - robot.y
        distance = math.hypot(dx, dy)
        if distance < 0.1:
            return None
        # If already within standoff, rotate in place toward the target.
        travel = max(0.0, distance - self.inspection_standoff_)
        goal = Pose2D()
        goal.x = robot.x + travel * dx / distance
        goal.y = robot.y + travel * dy / distance
        goal.theta = math.atan2(dy, dx)
        return goal

    def planner_go_to_first_artifact(self):
        if self.active_artifact_index_ is None:
            return
        goal = self.get_close_range_inspection_goal(
            self.artifact_locations_[self.active_artifact_index_])
        if goal is None:
            self.retry_or_abandon('cannot compute approach pose')
            return
        if self.planner_go_to_pose2d(goal, kind='inspection'):
            self.inspection_started_at_ = self.now_seconds()
            self.last_target_seen_at_ = self.now_seconds()
            self.get_logger().info('Planning 2: approaching artefact, facing target')

    def planner_return_home(self):
        """Optional template method; NOT used for Planning 3 behaviour switching."""
        goal = Pose2D(x=0.0, y=0.0, theta=math.pi)
        self.planner_go_to_pose2d(goal, kind='return_home')

    def planner_random_walk(self):
        """Go to a random location, which may be invalid"""

        # Select a random location
        goal_pose2d = Pose2D(
            x = random.uniform(self.xlim_[0], self.xlim_[1]),
            y = random.uniform(self.ylim_[0], self.ylim_[1]),
            theta = random.uniform(0, 2*math.pi)
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def planner_random_goal(self):
        """Go to a random location out of a predefined set"""

        # Hand picked set of goal locations
        random_goals = [[15.2, 2.2],
                        [30.7, 2.2],
                        [43.0, 11.3],
                        [36.6, 21.9],
                        [33.0, 30.4],
                        [40.4, 44.3],
                        [51.5, 37.8],
                        [16.0, 24.1],
                        [3.4, 33.5],
                        [7.9, 13.8],
                        [14.2, 37.7]]

        # Select a random location
        goal_valid = False
        while not goal_valid:
            idx = random.randint(0,len(random_goals)-1)
            goal_x = random_goals[idx][0]
            goal_y = random_goals[idx][1]

            # Only accept this goal if it's within the current costmap bounds
            if goal_x > self.xlim_[0] and goal_x < self.xlim_[1] and \
               goal_y > self.ylim_[0] and goal_y < self.ylim_[1]:
                goal_valid = True
            else:
                self.get_logger().warn(f'Goal [{goal_x}, {goal_y}] out of bounds')

        goal_pose2d = Pose2D(
            x = goal_x,
            y = goal_y,
            theta = random.uniform(0, 2*math.pi)
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def main_loop(self):
        """Planning 3: exploration -> preemption -> inspection -> exploration."""
        if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            return
        if self.state_ == 'PREEMPT':
            if self.current_goal_handle_ is None and not self.goal_request_pending_:
                self.cancel_requested_ = False
                self.state_ = 'INSPECT'
            else:
                self.cancel_current_goal()
                return
        if self.state_ == 'INSPECT':
            if self.inspection_started_at_ is not None:
                elapsed = self.now_seconds() - self.inspection_started_at_
                lost = (self.last_target_seen_at_ is not None and
                        self.now_seconds() - self.last_target_seen_at_ > self.target_lost_timeout_)
                if elapsed > self.inspection_timeout_ or lost:
                    if self.current_goal_handle_ is None and not self.goal_request_pending_:
                        self.cancel_requested_ = False
                        self.inspection_started_at_ = None
                        self.ready_for_next_goal_ = True
                        self.retry_or_abandon('timeout or target lost')
                        return
                    if not self.cancel_requested_:
                        self.get_logger().warn('Inspection timeout or target lost; cancelling goal')
                        self.cancel_current_goal()
                    # Wait for Nav2 result; that callback retries or abandons.
                    return
            if self.ready_for_next_goal_ and not self.goal_request_pending_ and self.current_goal_handle_ is None:
                self.planner_type_ = PlannerType.GO_TO_FIRST_ARTIFACT
                self.planner_go_to_first_artifact()
            return
        if self.state_ == 'EXPLORE' and self.ready_for_next_goal_:
            self.planner_type_ = PlannerType.FRONTIER_EXPLORATION
            self.goal_kind_ = 'frontier'
            self.get_logger().info('Planning 1/3: frontier exploration')
            self.planner_frontier_exploration()


def main():
    # Initialise
    rclpy.init()

    # Create the cave explorer
    cave_explorer = CaveExplorer()

    while rclpy.ok():
        rclpy.spin(cave_explorer)

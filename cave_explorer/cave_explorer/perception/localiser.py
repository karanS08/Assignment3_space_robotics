"""Estimate map-frame artefact positions and merge repeated detections.

Owner: Andrew Than
Task: Perception 3
"""
class CaveExplorer(Node):
  def __init__(self):
        #detects bounding box
        self.artifact_centre_x_ = None
        self.artifact_centre_y_ = None
    
        #depth estimation
        self.depth_image_ = None
        self.depth_sub_ = self.create_subscription(Image, 'camera/depth/image', self.depth_image_callback, 1)
  
  def depth_image_callback(self, image_msg):
        #converts ROS to OpenCV
        self.depth_image_ = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

  def image_callback(self, image_msg):
    #inside perception 2 detection of artifact
    #calculates centre pixel of artifact
    x, y, w, h = cv2.boundingRect(contour)
    self.artifact_centre_x_ = int(x + w/2)
    self.artifact_centre_y_ = int(y + h/2)


    #end of perception 2 detection of artifacts
    #starts locailisation when artifact is detected
    if self.artifact_found_:
      self.get_logger().info('Artifact found!')
      self.localise_artifact()


  def localise_artifact(self):
        if self.depth_image_ is None:
            return

        if self.artifact_centre_x_ is None:
            return

        #reads depth value of artifact
        distance = self.depth_image_[self.artifact_centre_y_, self.artifact_centre_x_]
        self.get_logger().info(f'Artifact distance = {distance}')

        #reads at an angle for artifact when localising
        image_width = self.depth_image_.shape[1]
        pixel_offset = (self.artifact_centre_x_ - image_width / 2.0)
        camera_fov = math.radians(120.0)
        relative_angle = (pixel_offset / (image_width / 2.0)) * (camera_fov / 2.0)
        artifact_heading = (robot_pose.theta - relative_angle)

        #converts into map coordinates
        point.x = (robot_pose.x + distance * math.cos(artifact_heading))
        point.y = (robot_pose.y + distance * math.sin(artifact_heading))      
        point.z = 1.0

        #placed at the end of the code
        #check if this detection matches an existing marker
        artifact_exists = False

        for existing_point in self.artifact_locations_:

            distance_to_marker = math.sqrt((point.x - existing_point.x) ** 2 + (point.y - existing_point.y) ** 2)
            # If within 2 metres, assume it's the same artefact
            if distance_to_marker < 2.0:

                existing_point.x = (existing_point.x + point.x) / 2.0
                existing_point.y = (existing_point.y + point.y) / 2.0
                artifact_exists = True
                break

        # Otherwise create a new marker
        if not artifact_exists:
            self.artifact_locations_.append(point)

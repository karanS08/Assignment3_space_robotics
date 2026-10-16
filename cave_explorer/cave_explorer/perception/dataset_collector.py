"""Save selected camera frames to disk for building the vision dataset.

Owner: Andrew Than
Task: Perception 1
"""
import os #used to create/manage dataset, contrusct file paths
import time #used to control timing

class CaveExplorer(Node):
  def __init__(self):
      
    #Dataset collection of artifacts
    self.dataset_directory_ = os.path.expanduser("~/artifact_dataset") #create dataset
    os.makedirs(self.dataset_directory_, exist_ok = True) #creates folder
    self.last_image_save_time_ = 0.0 #last image save
    self.save_interval_ = 3.0 #saves images every 3 seconds

    def image_callback(self, image_msg):
        if current_time - self.last_image_save_time_ > self.save_interval_: #perception_1 loop every 3 seconds
        
            filename = f"image_{int(current_time)}.png" #name of image
            filepath = os.path.join(self.dataset_directory_, filename) #building path
            cv2.imwrite(filepath, image) #save image
            self.last_image_save_time_ = current_time #update time
            self.get_logger().info(f"Saved dataset image: {filepath}") #notifies terminal

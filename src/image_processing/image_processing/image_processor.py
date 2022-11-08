# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Colour segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
    #    'video_frames', 
      '/camera/image_raw/uncompressed', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self.objects = []

  
  # This funciton will helps to generate teh mask for object with different color
  # and return whether it is an object in the image
  def generate_mask(self, hsv_frame):
    light_blue = np.array([14, 90, 95])
    dark_blue = np.array([60, 255, 255])

    light_pink = np.array([50,80,150])
    dark_pink = np.array([179,255,255])

    light_green = np.array([38,23,45])
    dark_green = np.array([52,255,255])

    light_yellow = np.array([94,135,90])
    dark_yellow = np.array([130,255,255])

    blue_mask = cv2.inRange(hsv_frame, light_blue, dark_blue)
    pink_mask = cv2.inRange(hsv_frame, light_pink, dark_pink)
    green_mask = cv2.inRange(hsv_frame, light_green, dark_green)
    yellow_mask = cv2.inRange(hsv_frame, light_yellow, dark_yellow)

    image_result = cv2.bitwise_or(blue_mask, pink_mask)
    image_result = cv2.bitwise_or(image_result, green_mask)
    image_result = cv2.bitwise_or(image_result, yellow_mask)

    kernel = np.ones((3, 3), np.uint8)
    image_result = cv2.dilate(image_result, kernel, iterations=1)
    
    return image_result
  
  def detect_objects(self, image_result):
    output = cv2.connectedComponentsWithStats(image_result, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output

    cens = []
    objects = []
    for i in range(0, numLabels):
        if i != 0 and stats[i][4] > 150:
          cens.append(centroids[i])
          objects.append({"x": stats[i][0], "y": stats[i][1], "width": stats[i][2], "height": stats[i][3], "area": stats[i][4]})

    LEFT=0
    COMPLETE = 1
    RIGHT=2
    detected_objs = []
    for i, obj in enumerate(objects):
      # In the image, object is too close to the left
      if obj["x"] == 0:
          detected_objs.append({"centroid": (cens[i][0], cens[i][1]), "width": obj["width"], "height": obj["height"], "status": LEFT})
      else:
          # In the image, object is too close to the right
          if obj["x"] + obj["width"] == image_result.shape[1]:
              detected_objs.append({"centroid": (cens[i][0], cens[i][1]), "width": obj["width"], "height": obj["height"], "status": RIGHT})
          else:
              detected_objs.append({"centroid": (cens[i][0], cens[i][1]), "width": obj["width"], "height": obj["height"], "status": COMPLETE})
    return detected_objs


  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    # The following code is a simple example of colour segmentation
    # and connected components analysis
    
    # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

    image_with_mask = self.generate_mask(hsv_frame)
    output = cv2.connectedComponentsWithStats(image_with_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output

    self.objects = self.detect_objects(image_with_mask)

    for obj in self.objects:
      cv2.drawMarker(image_with_mask, (int(obj['centroid'][0]),int(obj['centroid'][1])), (0,0,255),cv2.MARKER_SQUARE,15,2)
 
    # Print statistics for each blob (connected component)
    # use these statistics to find the bounding box of each blob
    # and to ine up laser scan with centroid of the blob
    for i in range(1, numLabels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        print(x, y, w, h, area)

    # Display camera image
    cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow('camera', 400, 400) 
    cv2.imshow("camera", current_frame)
    
    # Display masked image
    cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow('mask', 400, 400)
    cv2.imshow("mask", image_with_mask)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

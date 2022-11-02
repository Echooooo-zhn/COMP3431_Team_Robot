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
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
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

  # This funciton will help to draw the contour and the centroid of the object
  # def find_center(self, image_binary):
  #   # Draw contour for the object
  #   contours, hierarchy = cv2.findContours(image_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  #   rect = cv2.minAreaRect(contours[0])
  #   points = cv2.boxPoints(rect)  
  #   result = cv2.drawContours(image_binary, [np.int0(points)], -1, (255,255,255), 2)
    
  #   # Plot the centroid of the object
  #   M = cv2.moments(contours[0])
  #   center_x = int(M["m10"] / M["m00"])
  #   center_y = int(M["m01"] / M["m00"])
  #   cv2.circle(result, (center_x, center_y), 7, 128, -1)
  #   return result
  
  # This funciton will helps to generate teh mask for object with different color
  # and return whether it is an object in the image
  def generate_mask(self, hsv_frame, current_frame):
    light_blue = np.array([75, 172, 123])
    dark_blue = np.array([179, 255, 255])

    light_pink = np.array([152,78,255])
    dark_pink = np.array([179,255,255])

    light_green = np.array([70,164,109])
    dark_green = np.array([179,255,255])

    light_yellow = np.array([16,162,144])
    dark_yellow = np.array([179,255,255])

    blue_mask = cv2.inRange(hsv_frame, light_blue, dark_blue)
    pink_mask = cv2.inRange(hsv_frame, light_pink, dark_pink)
    green_mask = cv2.inRange(hsv_frame, light_green, dark_green)
    yellow_mask = cv2.inRange(hsv_frame, light_yellow, dark_yellow)

    image_blue_mask = cv2.bitwise_and(current_frame, current_frame, mask=blue_mask)
    image_pink_mask = cv2.bitwise_and(current_frame, current_frame, mask=pink_mask)
    image_green_mask = cv2.bitwise_and(current_frame, current_frame, mask=green_mask)
    image_yellow_mask = cv2.bitwise_and(current_frame, current_frame, mask=yellow_mask)

    image_result = cv2.bitwise_and(image_blue_mask, image_pink_mask)
    image_result = cv2.bitwise_and(image_result, image_green_mask)
    image_result = cv2.bitwise_and(image_green_mask, image_yellow_mask)
    
    return image_result

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
    # Mask out everything except pixels in the range light white to dark white
    # light_white = np.array([0, 0, 200])
    # dark_white = np.array([145, 60, 255])

    # light_pink = np.array([0,50,50])
    # dark_pink = np.array([10,255,255])

    # light_green = np.array([35,43,46])
    # dark_green = np.array([77,255,255])

    # light_yellow = np.array([26,43,46])
    # dark_yellow = np.array([34,255,255])
    # result = cv2.bitwise_and(current_frame, current_frame, white_result, mask=mask)

    # Run 4-way connected components, with statistics
    # output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
    image_with_mask = self.generate_mask(self, hsv_frame, current_frame)
    output = cv2.connectedComponentsWithStats(image_with_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output

    if (numLabels > 1):
      print("Object detected !!!")
      cv2.circle(image_with_mask, (centroids[0][0], centroids[0][1]), 7, 128, -1)
 
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

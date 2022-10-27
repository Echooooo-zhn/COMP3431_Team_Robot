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
    light_white = (0, 0, 200)
    dark_white = (145, 60, 255)
    mask = cv2.inRange(hsv_frame, light_white, dark_white)
    result = cv2.bitwise_and(current_frame, current_frame, mask=mask)

    # Run 4-way connected components, with statistics
    output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
 
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
    cv2.imshow("camera", current_frame)
    # Display masked image
    cv2.imshow("mask", mask)
    
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

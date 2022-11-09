# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Colour segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
import rclpy.qos
from geometry_msgs.msg import Quaternion
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from nav_msgs.msg import Odometry
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
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
    self.LEFT=0
    self.COMPLETE = 1
    self.RIGHT=2
    self.CAMANGLE = 31.1
    self.REALSYLINDERWIDTH = 0.1
    self.marker_list = MarkerArray()
    self.marker_list.markers = []
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
    self.image_size = None

    self.qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
    self.odmetry_subscription = self.create_subscription(
      Odometry,
      "odom",
      callback=self.odometry_callback,
      qos_profile=self.qos
      )
    
    
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.plot_publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
  
  
  def generate_marker(self, coordinate):
        marker = Marker()
        # marker.header.frame_id = "map"
        marker.header.frame_id = "/map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0
        marker.pose.position.x = coordinate[0]
        marker.pose.position.y = coordinate[1]
        marker.pose.position.z = coordinate[2]
        # todo=
        # marker.scale.x = 1
        # marker.scale.y = 1
        # marker.scale.z = 1
        # marker.color.a = 0.4
        # marker.color.r = 1 - val
        # marker.color.g = val
        # marker.color.b = 0.2
        # marker.id = idx                                                                                                                                                                                                                                                                      scale: { x: 0.1, y: 0.1, z: 0.1 }, color : { a: 1.0, r: 0.0, g: 0.0, b: 1.0 } } ] }
        self.marker_list.markers.append(marker)
        self.plot_publisher.publish(self.marker_list)
  
  def odometry_callback(self, data):
    # print(data.pose.pose.position)
    # print(self.objects)
    if len(self.objects) == 0:
      return

    # a = data.pose.pose.orientation.x,
    # b= data.pose.pose.orientation.y,
    # c=data.pose.pose.orientation.z,
    # d=data.pose.pose.orientation.w
    # # print(a,b,c,d)
    
    transform = self.tf_buffer.lookup_transform(
    "camera_link",
    "odom",
    # "odom",
    time=rclpy.time.Time()).transform
    
    print(transform)
    
    # coordinate of box
    obj_in_cam = [0, 0, 0]
    
    for obj in self.objects:
      if obj["status"] == self.COMPLETE:
        xPos = obj["centroid"][0]
        half_total = self.image_size[0] / 2.0
        sideA = abs(xPos - half_total) 
        sideB = half_total / math.tan(math.degrees(self.CAMANGLE))
        cam_width = obj["width"]
        # print(f"{sideA}   {sideB}  {cam_width}")
        ratio = self.REALSYLINDERWIDTH / cam_width
        real_sideA = ratio * sideA
        real_sideB = ratio * sideB
        
        print(f"front line: {real_sideA}, distance: {real_sideB}")
        cam_x = real_sideB
        cam_y = 0
        if xPos > half_total:
          cam_y = -real_sideA
        else:
          cam_y = real_sideA
        obj_in_cam[0] = cam_x
        obj_in_cam[1] = cam_y
        # object in camera's frame: (cam_x, cam_y)
    
    # P = Quater (vector) * obj_in_cam + translation
    translation = np.mat([transform.translation.x], [transform.translation], [transform.translation.z])
    cam_coord = np.mat([obj_in_cam[0]], [obj_in_cam[1]], [obj_in_cam[2]],)
    quaternion = [transform.quaternion.x, transform.quaternion.y, transform.quaternion.z,transform.quaternion.w, ]
    r = R.from_quat(quaternion)
    rotation_matrix = r.as_matrix()
    final_coordinate = rotation_matrix * cam_coord + translation
    self.generate_marker(final_coordinate)

  
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
    
    return image_result, blue_mask, pink_mask, green_mask, yellow_mask
  
  def determine_mask_color(self, blue_mask, pink_mask, green_mask, yellow_mask):
    # MASK: [UPPER, LOWER]
    PINK = 0
    BLUE = 1
    GREEN = 2
    YELLOW = 3
    
    mask = []
    detected_mask_y = None
    
    (blueNumLabels, blueLabels, blueStats, blueCentroids) = cv2.connectedComponentsWithStats(blue_mask, 4, cv2.CV_32S)
    (pinkNumLabels, pinkLabels, pinkStats, pinkCentroids) = cv2.connectedComponentsWithStats(green_mask, 4, cv2.CV_32S)
    (greenNumLabels, greenLabels, greenStats, greenCentroids) = cv2.connectedComponentsWithStats(pink_mask, 4, cv2.CV_32S)
    (yellowNumLabels, yellowLabels, yellowStats, yellowCentroids) = cv2.connectedComponentsWithStats(yellow_mask, 4, cv2.CV_32S)
    
    for i in range(0, blueNumLabels):
        if i != 0 and blueStats[i][4] > 100:
          mask.append(BLUE)
          detected_mask_y = blueCentroids[i][1]
    
    for i in range(0, pinkNumLabels):
        if i != 0 and pinkStats[i][4] > 100:
          if detected_mask_y == None:
            mask.append(PINK)
            detected_mask_y = blueCentroids[i][1]
          elif detected_mask_y < blueCentroids[i][1]:
            mask.append(PINK)
            return mask
          else:
            return [PINK, BLUE]
            
    for i in range(0, greenNumLabels):
        if i != 0 and greenStats[i][4] > 100:
          if detected_mask_y == None:
            mask.append(GREEN)
            detected_mask_y = greenCentroids[i][1]
          elif detected_mask_y < greenCentroids[i][1]:
            mask.append(GREEN)
            return mask
          else:
            return [GREEN, mask[0]]
          
    for i in range(0, yellowNumLabels):
        if i != 0 and yellowStats[i][4] > 100:
          if detected_mask_y == None:
            mask.append(YELLOW)
            detected_mask_y = yellowCentroids[i][1]
          elif detected_mask_y < yellowCentroids[i][1]:
            mask.append(YELLOW)
            return mask
          else:
            return [YELLOW, mask[0]]
          
    return mask
    
    
  
  def detect_objects(self, image_result, blue_mask, pink_mask, green_mask, yellow_mask):
    output = cv2.connectedComponentsWithStats(image_result, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
    

    mask_color = self.determine_mask_color(blue_mask, pink_mask, green_mask, yellow_mask)

    cens = []
    objects = []
    for i in range(0, numLabels):
        if i != 0 and stats[i][4] > 150:
          cens.append(centroids[i])
          objects.append({"x": stats[i][0], "y": stats[i][1], "width": stats[i][2], "height": stats[i][3], "area": stats[i][4], "colors": mask_color})

    LEFT=0
    COMPLETE = 1
    RIGHT=2
    
    detected_objs = []
    for i, obj in enumerate(objects):
      # In the image, object is too close to the left
      if obj["x"] == 0:
          detected_objs.append({"centroid": (cens[i][0], cens[i][1]), "width": obj["width"], "height": obj["height"], "status": LEFT, "colors": obj["colors"]})
      else:
          # In the image, object is too close to the right
          if obj["x"] + obj["width"] == image_result.shape[1]:
              detected_objs.append({"centroid": (cens[i][0], cens[i][1]), "width": obj["width"], "height": obj["height"], "status": RIGHT, "colors": obj["colors"]})
          else:
              detected_objs.append({"centroid": (cens[i][0], cens[i][1]), "width": obj["width"], "height": obj["height"], "status": COMPLETE, "colors": obj["colors"]})
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
    self.image_size = hsv_frame.shape

    image_with_mask, blue_mask, pink_mask, green_mask, yellow_mask = self.generate_mask(hsv_frame)

    self.objects = self.detect_objects(image_with_mask, blue_mask, pink_mask, green_mask, yellow_mask)

    for obj in self.objects:
      cv2.drawMarker(image_with_mask, (int(obj['centroid'][0]),int(obj['centroid'][1])), (0,0,255),cv2.MARKER_SQUARE,15,2)

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

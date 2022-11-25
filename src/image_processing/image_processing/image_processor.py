# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Colour segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
import rclpy.qos
from pyquaternion import Quaternion
# from geometry_msgs.msg import Quaternion
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image,LaserScan # Image is the message type
from nav_msgs.msg import Odometry
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from visualization_msgs.msg import Marker, MarkerArray 
class ImageSubscriber(Node):
    PINK = 0
    BLUE = 1
    GREEN = 2
    YELLOW = 3
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
        self.REALSYLINDERWIDTH = 0.14
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

        # position listener
        self.qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )
        # transform lisnter
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plot_publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)
        
        # laser listener
        self.laser_angles = {}
        self.laser_catch = self.create_subscription(LaserScan, "/scan", 
                                                    callback=self.scan_callback,
                                                    qos_profile=self.qos)
        print("--------")
    
    def scan_callback(self, data):
        self.laser_angles = {}
        for i in range(0, 30):
            self.laser_angles[i] = data.ranges[i]
        for i in range(330, 360):
            self.laser_angles[i] = data.ranges[i]
    
    def check_whether_occur(self, point, new_color):
      for marker in self.marker_list.markers:
        if math.sqrt((point[0] - marker.pose.position.x)**2 + \
          (point[1] - marker.pose.position.y)**2) <= 0.5:
          r = marker.color.r * 255
          g = marker.color.g * 255
          b = marker.color.b * 255
          
          if new_color == self.BLUE and abs(r) <= 0.1 and abs(g - 191) <= 0.1 and abs(b - 255) < 0.1:
            return True
          elif new_color == self.YELLOW and abs(r - 255) <= 0.1 and abs(g - 234) <= 0.1 and abs(b ) < 0.1:
            return True
          elif new_color == self.GREEN and abs(r) <= 0.1 and abs(g - 100) <= 0.1 and abs(b) < 0.1:
            return True
      return False
    
    # # test code
    # def new_calculation(self, alpha):
    #     A_coordinate = [1,2,0]
    #     B_coordinate = [1,0,0]
    #     laser_scanned = self.laser_angles
    #     # AB = math.sqrt((A_coordinate[0] - B_coordinate[0])**2 - (A_coordinate[1] - B_coordinate[1]) **2)
    #     # hard cord by measurement
    #     AB = 0.14
    #     correct_laser_angle = -1
    #     for i in range(1, min(31, int(math.degrees(alpha)) + 2)):
    #         beta = i    
    #         if alpha >= 0:
    #             scanned_dist = laser_scanned[beta]
    #         else:
    #             scanned_dist = laser_scanned[360 - beta]
    #         # if scanned_dist <= 0.0001:
    #         #     continue    
    #         beta = math.radians(i)
    #         left = (math.cos(beta) - (math.sin(beta) / math.tan(alpha)))
    #         BC = AB / (math.cos(beta) - (math.sin(beta) / math.tan(alpha)))
    #         # match
    #         # print(scanned_dist, BC)
    #         print(f"BC{BC} dis{scanned_dist } {math.degrees(beta)} {math.degrees(alpha)}")
    #         if i < 3 and abs(BC + 0.14 - scanned_dist) <= 0.01:
    #             correct_laser_angle = i
    #             break
            
    #         if (abs(BC - scanned_dist) <= 0.01):
    #             correct_laser_angle = i
    #             break
    
    #     print(f"laser calculated angle: {(correct_laser_angle)}")
    #     return
    def add_new_point(self, coordinate, color, special_color_up):
        coordinate[0] = float(coordinate[0])
        coordinate[1] = float(coordinate[1])
        coordinate[2] = float(coordinate[2])
        print("add_new_point")
        if self.check_whether_occur(coordinate, color):
          return     
        
        
        print("add_new_point2")
        self.generate_marker(coordinate, color, special_color_up)
        print(f"-----------Current length{len(self.marker_list.markers)}----------")
        self.plot_publisher.publish(self.marker_list)
    
    def generate_marker(self, coordinate, color, special_color_up):
        
        # down cylinder
        marker = Marker()
        # marker.header.frame_id = "map"
        marker.header.frame_id = "/map"
        marker.id = len(self.marker_list.markers) + 1
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(coordinate[0])
        marker.pose.position.y = float(coordinate[1])
        marker.pose.position.z = float(coordinate[2]) + 0.1
        marker.scale.x = 0.14
        marker.scale.y = 0.14
        marker.scale.z = 0.2
        marker.color.a = 1.0
        if special_color_up != True:
          rgb = (255,192,203)
        elif color == self.YELLOW:
          rgb = (255,234,0)
        elif color == self.BLUE:
          rgb = (0,191,255)
        else:
          rgb = (0,100,0)
        marker.color.r = rgb[0] / 255.0
        marker.color.g = rgb[1] / 255.0
        marker.color.b = rgb[2] / 255.0
        self.marker_list.markers.append(marker)

        # up cylinder
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = len(self.marker_list.markers) + 1
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(coordinate[0])
        marker.pose.position.y = float(coordinate[1])
        marker.pose.position.z = float(coordinate[2]) + 0.3
        marker.scale.x = 0.14
        marker.scale.y = 0.14
        marker.scale.z = 0.2
        if special_color_up == True:
          rgb = (255,192,203)
        elif color == self.YELLOW:
          rgb = (255,234,0)
        elif color == self.BLUE:
          rgb = (0,191,255)
        else:
          rgb = (0,100,0)
        marker.color.a = 1.0
        marker.color.r = rgb[0] / 255.0
        marker.color.g = rgb[1] / 255.0
        marker.color.b = rgb[2] / 255.0
        self.marker_list.markers.append(marker)
  
  
    def transform_frame(self, target, source , translation, quaternion):
        try:
          transform = self.tf_buffer.lookup_transform(target_frame=target, source_frame=source, time=rclpy.time.Time()).transform
          translation[0] = translation[0] + transform.translation.x
          translation[1] = translation[1] + transform.translation.y
          translation[2] = translation[2] + transform.translation.z
          quaternion[0] = transform.rotation.w
          quaternion[1] = quaternion[1] + transform.rotation.x
          quaternion[2] = quaternion[2] + transform.rotation.y
          quaternion[3] = quaternion[3] + transform.rotation.z
          return (translation, quaternion)
        except Exception:
          return ([0],[0])

    def draw_cylinder(self):
        if len(self.objects) == 0:
            return
        
        # coordinate of box
        obj_in_cam = [0, 0, 0]
        
        obj = self.objects[0]
        if obj["status"] == self.COMPLETE and obj["width"] > 10:
          # print("Once")
          xPos = obj["centroid"][0]
          half_total = self.image_size[0] / 2.0
          sideA = abs(xPos - half_total) 
          sideB = half_total / math.tan(math.radians(self.CAMANGLE))
          # print(f"xpos: {xPos} half:{half_total}")
          cam_width = obj["width"]
          ratio = self.REALSYLINDERWIDTH / cam_width
          real_sideA = ratio * sideA
          real_sideB = ratio * sideB
          real_distance = math.sqrt(real_sideA**2 + real_sideB**2) 
          print("line 236")
          if real_distance > 0.7:
            return
          print("line 238")
          print(f"REAL{real_sideA}   {real_sideB} {real_distance} {cam_width} {self.image_size[0]}")
          cam_x = real_sideB
          cam_y = 0
          if xPos > half_total:
              cam_y = -real_sideA
          else:
              cam_y = real_sideA
          obj_in_cam[0] = cam_x
          obj_in_cam[1] = cam_y
      # object in camera's frame: (cam_x, cam_y)
          translation = [0,0,0]
          quaternion = [1,0,0,0]
          translation, quaternion = self.transform_frame("map", "camera_link", translation, quaternion)
          print("line 252")
          if len(translation) != 3:
            return
          print("line 254")
          my_quater = Quaternion(quaternion[0], quaternion[1], quaternion[2],quaternion[3])
          rotation = my_quater.rotate(obj_in_cam)
          final_coordinate = [0,0,0]
          final_coordinate[0] = rotation[0] + translation[0]
          final_coordinate[1] = rotation[1] + translation[1]
          final_coordinate[2] = rotation[2] + translation[2]
          if len(obj["colors"]) < 2:
            color = self.PINK
            special_color_up = True
          elif obj["colors"][0] == self.PINK:
            special_color_up = True
            color = obj["colors"][1]
          else:
            special_color_up = False
            color = obj["colors"][0]
          print(f"coordinate{final_coordinate}")
          self.add_new_point(final_coordinate, color, special_color_up)

  
  # This funciton will helps to generate teh mask for object with different color
  # and return whether it is an object in the image
    def generate_mask(self, hsv_frame):
      light_blue = np.array([14, 90, 95])
      dark_blue = np.array([37, 255, 255])

      light_pink = np.array([50,80,150])
      dark_pink = np.array([179,255,255])

      light_green = np.array([38,23,45])
      dark_green = np.array([52,255,255])

      light_yellow = np.array([94,135,90])
      dark_yellow = np.array([130,255,255])

      kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
      blue_mask = cv2.inRange(hsv_frame, light_blue, dark_blue)
      blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel,iterations=1)
      pink_mask = cv2.inRange(hsv_frame, light_pink, dark_pink)
      pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_OPEN, kernel,iterations=1)
      green_mask = cv2.inRange(hsv_frame, light_green, dark_green)
      green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel,iterations=1)
      yellow_mask = cv2.inRange(hsv_frame, light_yellow, dark_yellow)
      yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel,iterations=1)

      image_result = cv2.bitwise_or(blue_mask, pink_mask)
      image_result = cv2.bitwise_or(image_result, green_mask)
      image_result = cv2.bitwise_or(image_result, yellow_mask)

      image_result = cv2.morphologyEx(image_result, cv2.MORPH_OPEN, kernel,iterations=1)
      kernel = np.ones((3, 3), np.uint8)
      image_result = cv2.dilate(image_result, kernel, iterations=1)
    
      return image_result, blue_mask, pink_mask, green_mask, yellow_mask
    
    def determine_mask_color(self, image_result, blue_mask, pink_mask, green_mask, yellow_mask):
      # MASK: [UPPER, LOWER]
      
      mask = []
      detected_mask_y = None
      
      blue_mask = cv2.bitwise_and(blue_mask, image_result)
      green_mask = cv2.bitwise_and(green_mask, image_result)
      blue_mask = cv2.bitwise_and(blue_mask, image_result)
      pink_mask = cv2.bitwise_and(pink_mask, image_result)
      
      (blueNumLabels, blueLabels, blueStats, blueCentroids) = cv2.connectedComponentsWithStats(blue_mask, 4, cv2.CV_32S)
      (pinkNumLabels, pinkLabels, pinkStats, pinkCentroids) = cv2.connectedComponentsWithStats(pink_mask, 4, cv2.CV_32S)
      (greenNumLabels, greenLabels, greenStats, greenCentroids) = cv2.connectedComponentsWithStats(green_mask, 4, cv2.CV_32S)
      (yellowNumLabels, yellowLabels, yellowStats, yellowCentroids) = cv2.connectedComponentsWithStats(yellow_mask, 4, cv2.CV_32S)
      
      for i in range(0, blueNumLabels):
          if i != 0 and blueStats[i][4] > 100:
            mask.append(self.BLUE)
            detected_mask_y = blueCentroids[i][1]
      
      for i in range(0, pinkNumLabels):
          if i != 0 and pinkStats[i][4] > 100:
            if detected_mask_y == None:
              mask.append(self.PINK)
              detected_mask_y = pinkCentroids[i][1]
            elif detected_mask_y < pinkCentroids[i][1]:
              mask.append(self.PINK)
              return mask
            else:
              return [self.PINK, self.BLUE]
              
      for i in range(0, greenNumLabels):
          if i != 0 and greenStats[i][4] > 100:
            if detected_mask_y == None:
              mask.append(self.GREEN)
              detected_mask_y = greenCentroids[i][1]
            elif detected_mask_y < greenCentroids[i][1]:
              mask.append(self.GREEN)
              return mask
            else:
              return [self.GREEN, mask[0]]
            
      for i in range(0, yellowNumLabels):
          if i != 0 and yellowStats[i][4] > 100:
            if detected_mask_y == None:
              mask.append(self.YELLOW)
              detected_mask_y = yellowCentroids[i][1]
            elif detected_mask_y < yellowCentroids[i][1]:
              mask.append(self.YELLOW)
              return mask
            else:
              return [self.YELLOW, mask[0]]
            
      return mask
      
      
    
    def detect_objects(self, image_result, blue_mask, pink_mask, green_mask, yellow_mask):
      
      output = cv2.connectedComponentsWithStats(image_result, 4, cv2.CV_32S)
      (numLabels, labels, stats, centroids) = output
      

      mask_color = self.determine_mask_color(image_result, blue_mask, pink_mask, green_mask, yellow_mask)

      cens = []
      objects = []
      # y_min = 1000
      for i in range(0, numLabels):
          if i != 0 and stats[i][4] > 150:
            # y_min = min(stats[i][1], y_min)
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
      # self.get_logger().info('Receiving video frame')
  
      # Convert ROS Image message to OpenCV image
      current_frame = self.br.imgmsg_to_cv2(data)

      # The following code is a simple example of colour segmentation
      # and connected components analysis
      
      # Convert BGR image to HSV
      hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
      self.image_size = hsv_frame.shape

      image_with_mask, blue_mask, pink_mask, green_mask, yellow_mask = self.generate_mask(hsv_frame)

      self.objects = self.detect_objects(image_with_mask, blue_mask, pink_mask, green_mask, yellow_mask)

      self.draw_cylinder()

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

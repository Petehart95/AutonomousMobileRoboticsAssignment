import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("mask", 1)
    cv2.namedWindow("hsv", 2)
    cv2.namedWindow("bgr", 3)

    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.twist = Twist()
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    
    lower_red = numpy.array([35,70,35]) #BGR Mask
    upper_red = numpy.array([80,255,150])
    
    
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    h, w, d = image.shape
    #search_top = h*1/2
    #search_bot = h*1/2 + 20
    #mask[0:search_top, 0:w] = 0
    #mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      
    cv2.imshow("mask", mask)
    cv2.imshow("hsv", hsv)
    cv2.imshow("bgr", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()

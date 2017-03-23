#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from math import radians, sqrt
from move_base_msgs.msg import MoveBaseActionFeedback
from rosgraph_msgs.msg import Clock
from actionlib_msgs.msg import GoalStatusArray

class path_planner:
  def __init__(self):
      #self.goalPoint == each node present in the topological map, whereby each tuple represents
      # the x and y coordinates of the node relative to the map
    self.goalPoint = [[1.8,-4.39],[1.45,-2.58],[-0.349,-0.48],[-0.742,2.16],
                      [0.743,4.46],[-0.742,2.16],[0.758,2.01],[3.02,1.25],[2.71,-0.171],
                      [1.43,-1.42],[-0.511,-1.55],[-4.12,-0.512],[-3.35,0.038],[-4.05,1.92],
                      [-4.14,4.56],[-3.16,3.26]]
    
    self.posX = 0.0
    self.posY = 0.0
    self.global_mean = 0
    self.currentNode = 0
    #Variables defining 
    self.R_track = True
    self.G_track = True
    self.B_track = True
    self.Y_track = True
    
    self.R_selected = False
    self.Y_selected = False
    
    self.search_active = False
    self.rotateSearch = False
    self.enRoute = False
    
    #Notify the turtlebot of its starting position
    self.initialPose = PoseWithCovarianceStamped()
    self.initialPose.header.frame_id = "map"
    self.initialPose.pose.pose.position.x = -3.96
    self.initialPose.pose.pose.position.y = -4.51
    self.initialPose.pose.pose.orientation.w = 1.0    
    #Initialise relevant rostopic subscribers and publishers
    self.clock_sub = rospy.Subscriber('/clock', Clock)
    self.feedback_sub = rospy.Subscriber('/turtlebot/move_base/feedback', MoveBaseActionFeedback)    
    self.status_sub = rospy.Subscriber('/turtlebot/move_base/status', GoalStatusArray)
    self.amcl_sub = rospy.Subscriber('/turtlebot/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
    self.laserscan_sub = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laser_callback)        
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw',Image,self.image_callback) 
    #self.depth_sub = rospy.Subscriber('/turtlebot/camera/depth/image_raw',Image,self.image_callback) 
    self.initialPose_pub = rospy.Publisher('/turtlebot/initialpose', PoseWithCovarianceStamped,queue_size=10)    
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    self.goalSimple_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped,queue_size=10)

    self.bridge = cv_bridge.CvBridge()
    self.twist = Twist()
    self.rotateTwist = Twist()
    
    self.rotateTwist.linear.x = 0
    #Only move when rotating
    rospy.Rate(5)
    
    #For each node that is present in the list of nodes of the topological map
    #(range = n - (m-1))
    
    size = 17
    
    for i in range (0,size):
        #Turtlebot should follow this route
        self.goToNode(i)
        
        #When the turtlebot arrives at the goal node, perform a rotationSearch and look for one of the objects
        self.rotateSearch = True
        self.object_found = False
        ctr = 0
        T1 = 0.005
        T2 = 1.8
        
        #While the robot hasn't completed a full rotation
        while ctr < 360:
            if (self.R_track == True or self.G_track == True or self.B_track == True or self.Y_track) and self.global_mean > T1 and self.global_mean < T2:
                #Object found! Pause rotation
                self.rotateSearch = False
                self.currentNode = i
                self.search_active = True
            elif self.object_found == True:
                #Do nothing and wait for the object to be found
                ctr = ctr
            else:
                #Nothing found here, continue rotating
                ctr += 45
        # Node searched, move to the next one
        self.rotateSearch = False
        
        #If all of the objects have been found, stop
        if (self.R_track == False and self.G_track == False and self.B_track == False and self.Y_track == False):
            print "Finished!"
        if (i + 1) > size and (self.R_track == True or self.G_track == True or self.B_track == True or self.Y_track == True):
            #If all nodes have been visited, restart
            i = 0
            
    
  def goToNode(self, i):
      #Reset route completion check variable
      route = False
      #while the route to the currently selected node position is not completed
      while route != True:
         #Turtlebot should follow this route
         route = self.doRoute(i)
         
  def amcl_callback(self,pose):
    self.posX = pose.pose.pose.position.x
    self.posY = pose.pose.pose.position.y
    
  def laser_callback(self,scan):
      depths = []
      for dist in scan.ranges:
          if not numpy.isnan(dist):
              depths.append(dist)
      scan.ranges[len(scan.ranges)/2]
      
  def doRoute(self, i):
      #Current position of the turtlebot relative to the map
      currentPosX = self.posX
      currentPosY = self.posY
      
      #Threshold for judging distance away from an object
      T = 0.5
      
      #Position of the currently selected Node
      goalPosX = self.goalPoint[i][0]
      goalPosY = self.goalPoint[i][1]
      
      #calculate the absolute distance between turtlebot and the current goal
      distance_x = abs(goalPosX - currentPosX)
      distance_y = abs(goalPosY - currentPosY)
      
      #if the turtlebot is close enough to the goal (~1m)
      if (distance_x < T) and (distance_y < T):
          #Return goal reached = True
          return True
      else:
          #Otherwise, keep moving towards the current goal
          currentGoal = PoseStamped()
          currentGoal.header.frame_id = "map"
          currentGoal.pose.position.x = goalPosX
          currentGoal.pose.position.y = goalPosY
          currentGoal.pose.orientation.w = 1.0
          self.goalSimple_pub.publish(currentGoal)
          
          #allow turtlebot time to compute Dijkstra's algorithm
          time.sleep(1)
        
  def image_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')

        #Threshold for defining maximum distance or mean which the turtlebot will actually consider travelling towards
        #(If object is too far away, ignore it)
        # Lower mean = lower presence on the camera, so it can be assumed that the object is further away
        T1 = 0.475
        # threshold for defining when the turtlebot is close enough to one of the objects
        # (~1m away from object)
        T2 = 1.8
        
        #convert rgb image callback into hsv format
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        ##########################################################################
        ############################# COLOUR SLICING #############################
        ##########################################################################
        
        # RED MASK
        lower_red = numpy.array([0,100,100]) 
        upper_red = numpy.array([2,255,255])
        maskR = cv2.inRange(hsv,lower_red,upper_red)
        
        # BLUE MASK
        lower_blue = numpy.array([103,100,100])
        upper_blue = numpy.array([125,255,255])
        maskB = cv2.inRange(hsv,lower_blue,upper_blue)
        
        # GREEN MASK
        lower_green = numpy.array([40,70,70]) 
        upper_green = numpy.array([80,255,255]) 
        maskG = cv2.inRange(hsv,lower_green,upper_green)        
        
        #YELLOW MASK
        lower_yellow = numpy.array([30,100,100]) 
        upper_yellow = numpy.array([35,255,255])
        maskY = cv2.inRange(hsv,lower_yellow,upper_yellow)

        # Get the dimensions of the image (height, width, dimensions/channels)
        # This will be used later to further define properties of each mask
        h, w, d = image.shape
        
        #Limit view of the mask to reduce noise input and to help stop the turtlebot from
        #seeing an object too far away. 
        
        #So this will operate by enforcing a letter-box view with the turtlebot's mask
        #situated around the centre of the camera.
        search_top = h*1/2
        search_bot = h*1/2 + 20
        
        maskR[0:search_top, 0:w] = 0
        maskR[search_bot:h, 0:w] = 0
        
        maskG[0:search_top, 0:w] = 0
        maskG[search_bot:h, 0:w] = 0
        
        maskB[0:search_top, 0:w] = 0
        maskB[search_bot:h, 0:w] = 0
        
        maskY[0:search_top, 0:w] = 0
        maskY[search_bot:h, 0:w] = 0
        
        M_R = cv2.moments(maskR)
        M_R_mean = numpy.mean(maskR)
        
        M_G = cv2.moments(maskG)
        M_G_mean = numpy.mean(maskG)        
        
        M_B = cv2.moments(maskB)
        M_B_mean = numpy.mean(maskB)        
        
        M_Y = cv2.moments(maskY)
        M_Y_mean = numpy.mean(maskY)
        
        self.global_mean = M_R_mean + M_G_mean + M_B_mean + M_Y_mean        
        
        # RED MASK
        if M_R['m00'] > 0 and M_R_mean > T1 and self.R_track == True and self.search_active == True:
          self.object_found = True
          #print "Moving towards red object..."
          self.R_selected = True
          cx = int(M_R['m10']/M_R['m00'])
          cy = int(M_R['m01']/M_R['m00'])
    
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.2
          self.twist.angular.z = -float(err) /100
          self.cmd_vel_pub.publish(self.twist)
          if M_R_mean >= T2:
              self.R_track = False
              print "Red object found!"
              #Unselect colour
              self.search_active = False
              
          self.object_found = False

        # GREEN MASK
        elif M_G['m00'] > 0 and M_G_mean > T1 and self.G_track == True and self.search_active == True:
          self.object_found = True
          print "Moving towards green object..."
          self.G_selected = True
          cx = int(M_G['m10']/M_G['m00'])
          cy = int(M_G['m01']/M_G['m00'])
    
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.2
          self.twist.angular.z = -float(err) /100
          self.cmd_vel_pub.publish(self.twist)
          if M_G_mean >= T2:
              self.G_track = False
              print "Green object found!"
              #Unselect colour
              self.search_active = False
          self.object_found = False

        # BLUE MASK
        elif M_B['m00'] > 0 and M_B_mean > T1 and self.B_track == True and self.search_active == True:
          self.object_found = True
        #  print "Moving towards blue object..."
          self.B_selected = True
          cx = int(M_B['m10']/M_B['m00'])
          cy = int(M_B['m01']/M_B['m00'])
    
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.2
          self.twist.angular.z = -float(err) /100
          self.cmd_vel_pub.publish(self.twist)
          
          print M_B_mean          
          if M_B_mean >= T2:
              self.B_track = False
              print "Blue object found!"
              #Unselect colour
              self.search_active = False
          self.object_found = False
          
        # YELLOW MASK
        elif M_Y['m00'] > 0 and M_Y_mean > T1 and self.Y_track == True and self.search_active == True:
          self.object_found = True
          print "Moving towards yellow object..."
          self.Y_selected = True
          cx = int(M_Y['m10']/M_Y['m00'])
          cy = int(M_Y['m01']/M_Y['m00'])
    
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.2
          self.twist.angular.z = -float(err) /100
          self.cmd_vel_pub.publish(self.twist)
          if M_Y_mean >= T2:
              self.Y_track = False
              print "Yellow object found!"
              #Unselect colour
              self.search_active = False
          
          self.object_found = False
          # END CONTROL
        cv2.imshow("window", image)
        cv2.waitKey(3)

#rospy.init_node('object_search')

rospy.init_node('path_planner')

path_planner()
rospy.spin()

  #  object_search()

# END ALL
 #!/usr/bin/env python3.6
import rospy, time, collections, math
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import imutils
from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist

def shutdown():
    rospy.loginfo("Shutting Down")
    exit()

def land():
   start=time.time()
   pub = rospy.Publisher("bebop/land", Empty, queue_size=10 )
   #rospy.init_node('land', anonymous=True)
   rate = rospy.Rate(10) # 10hz
   while not rospy.is_shutdown():
       pub.publish(Empty())
       rate.sleep()
       end = time.time()
       if (end-start) >= 3:
            rospy.loginfo("Landing")
            shutdown()

def takeoff():
   start=time.time()
   pub = rospy.Publisher("bebop/takeoff", Empty, queue_size=10 )
   rospy.init_node('takeoff', anonymous=True)
   rate = rospy.Rate(10) # 10hz
   while not rospy.is_shutdown():
       pub.publish(Empty())
       rate.sleep()
       end = time.time()
       if (end-start) >= 3:
           rospy.loginfo("Takeoff Successful")
           break

def show_image(num, img):
     cv2.imshow(str(num), img)
     cv2.waitKey(3)

def turnanticlockwise(angle):
    start=time.time()
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.3
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
            end = time.time()
            if (end-start) >= 0.0104*angle: #0.013 for simulated drone
                rospy.loginfo("Turned "+str(angle)+" degrees")
                break

def turnclockwise(angle):
    start=time.time()
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -0.3
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
            end = time.time()
            if (end-start) >= 0.0104*angle: #0.013 for simulated drone
                rospy.loginfo("Turned "+str(angle)+" degrees")
                break

# Define a callback for the Image message
def image_callback(img_msg):
     #rospy.loginfo(img_msg.header)

     # Try to convert the ROS Image message to a CV2 Image
     try:
         global live_image
         live_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
     except CvBridgeError, e:
         rospy.logerr("CvBridge Error: {0}".format(e))

#rospy.init_node('opencv', anonymous=True)
bridge = CvBridge()
sub_image = rospy.Subscriber("/bebop/image_raw", Image, image_callback)
cv2.namedWindow("Image Window", 1)
Lower = (115, 179, 55)
Upper = (125, 240, 170)
pts = collections.deque(maxlen=40)
thickness =3
font = cv2.FONT_HERSHEY_SIMPLEX
takeoff()
i = 0
while not rospy.is_shutdown():
    if  cv2.waitKey(33) == 27:
        print("User Interrupt")
        land()
    i = i+1
    blurred = cv2.GaussianBlur(live_image.copy(), (11,11),0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    RGB_image = cv2.cvtColor(live_image, cv2.COLOR_BGR2RGB)
    show_image(2,hsv)
    curves = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    curves = imutils.grab_contours(curves)
    center = None
    if len(curves) > 0:
		c = max(curves, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    pts.appendleft(center)
    try:
        topleft = ((center[0]-90),(center[1]+30))
        bottomright = ((center[0]+90),(center[1]+70) )
        cv2.putText(RGB_image, ('x=' + str(center[0]) + ', y=' + str(480-center[1])), ((center[0]-80),(center[1]+55)), font, 0.7, (255, 255, 255),2,cv2.LINE_AA)
        cv2.rectangle(RGB_image, topleft, bottomright, (255,255,255), 3)
        cv2.circle(RGB_image, center, 10, (255, 0, 0), -1)
        if i % 5 == 0:
            if  (center[0] < 350):
                turnanticlockwise(1)
            if  (center[0] > 556):
                turnclockwise(1)
    except:
        cv2.putText(RGB_image, ('No Object Found'), (350,480/2), font, 0.7, (255, 255, 255),2,cv2.LINE_AA)
    show_image(1, RGB_image)

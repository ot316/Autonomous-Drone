import rospy, time
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

def move_forwards(distance):
    start=time.time()
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    twist = Twist()
    twist.linear.x = 1; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
            end = time.time()
            if (end-start) >= distance*0.4:
                rospy.loginfo("Moved "+str(distance)+" units")
                break

def move_upwards(distance):
    start=time.time()
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 1
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
            end = time.time()
            if (end-start) >= distance*0.4:
                rospy.loginfo("Moved "+str(distance)+" units")
                break

def move_sideways(distance):
    start=time.time()
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 1; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
            end = time.time()
            if (end-start) >= distance*0.4:
                rospy.loginfo("Moved "+str(distance)+" units")
                break

def turn(angle):
    start=time.time()
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 3
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
            end = time.time()
            if (end-start) >= 0.0104*angle: #0.013 for simulated drone
                rospy.loginfo("Turned "+str(angle)+" degrees")
                break

def flip(direction):
    start=time.time()
    pub = rospy.Publisher("bebop/flip", UInt8, queue_size=10 )
    rospy.init_node('takeoff', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(UInt8(direction))
        rate.sleep()
        end = time.time()
        if (end-start) >= 1:
            rospy.loginfo("Flipped")
            break

if __name__ == '__main__':
    try:
      takeoff()
      move_upwards(10)
      move_forwards(2)
      turn(90)
      move_forwards(2)
      turn(90)
      move_forwards(2)
      turn(90)
      move_forwards(2)
      turn(90)
      time.sleep(1)
      flip(2)
      land()
    except rospy.ROSInterruptException:
       pass

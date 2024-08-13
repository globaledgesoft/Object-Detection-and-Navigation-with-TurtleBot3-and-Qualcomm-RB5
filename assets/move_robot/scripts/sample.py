#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from object_detection.msg import FloatList
from turtlebot3_msgs.msg import Sound  # Importing only the Sound message
from sensor_msgs.msg import LaserScan  
from move_robot.msg import ObjectFound 
from ros_speech_recognization.msg import data
from collections import deque
import numpy as np
import time
import math
from gtts import gTTS
import os
import numpy as np
from numpy import inf
import random
from functools import reduce


# Publishers: send velocity and sound to TurtleBot
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Updated velocity command topic
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub_sound = rospy.Publisher('/sound', Sound, queue_size=10)  # Updated sound command topic
pub_object_found = rospy.Publisher('/Object_found', ObjectFound, queue_size=10)  # Use the custom message type


x_linear_vel = 0.0      # Linear velocity x
z_angular_vel = 0.0     # Angular velocity z
last_offset = 0.0       # Store last offset detected

area_detected = 0.3     # Threshold area for sending sound
time_diff = 0.0
detections = deque(maxlen=7)        # Queue which contains last 7 detections
last_sound_time = time.time()       # Time for spiral sound
last_detection_time = time.time()   # Time for detection sound
spiraling_sound = False             # Spiral sound

maneuvering = False         # If robot is maneuvering after a collision
orientation = 0.0           # Robot orientation

searching = True            # State for initial searching

rate = None

front_range   = 20
front_limit   = 0.5 #0.8
side_limit    = 0.3 #0.3

move_flag     = 1
linear_speed  = 0.2
angular_speed = 0.2
kp   = 2
y_l  = 0
l_l  = 0
r_l  = 0
fast = 1
y_b  = 0

# Set up Google Cloud Speech-to-Text credentials
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/root/OBJ_WS/src/ros_speech_recognization/scripts/google_secret_key.json'



## Publishes the message for the toy detection
def play_sound(value):
    sound_msg = Sound()
    sound_msg.value = value
    pub_sound.publish(sound_msg)

## Generate and play the speech
def say_object_found():
    generated_text = "Object found. Ready to listen for a new object."
    tts = gTTS(generated_text)
    tts.save('output.mp3')
    time.sleep(0.1)
    if os.path.exists("./output.mp3"):
        os.system("ffplay -nodisp -autoexit -loglevel quiet output.mp3")
        os.system("rm output.mp3")
    else:
        print("Error: Could not generate speech.")


## Returns a value remaped from one range to another
def remap(old_val, old_min, old_max, new_min, new_max):
    return (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min

## Returns True if there is a majority of detections, False otherwise
def detected_majority(detections):
    pos = 0
    for i in range(0, len(detections)):
        pos = pos + 1 if detections[i] == 1 else pos
    return pos >= len(detections)/2

## Converts a quaternion to euler coordinates
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

## Called each time the odometry is updated
## Update TurtleBot orientation
def odom_callback(data):
    global orientation
    orientation_q = data.pose.pose.orientation
    euler = quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    orientation = euler[0]  # Save yaw

## Called each time the detection module detects something
## Calculate and updates linear and angular velocities
def detection_callback(data):
    global x_linear_vel, z_angular_vel, detections, last_sound_time, area_detected, last_offset, last_detection_time, spiraling_sound, maneuvering, orientation, searching, time_diff

    # Add a detection to the queue
    detections.append(data.detected)

    # If it is not solving a bump collision
    if not maneuvering:
        # If there is a majority in the queue
        if detected_majority(detections):

            searching = False
            last_detection_time = time.time()   # Start timer for exploration
            z_angular_vel = -remap(data.xOffset, -320, 320, -0.3, 0.3)  # Set angular velocity scaled to the offset

            # If the detected box is centered -> Stop rotating
            if data.xOffset >= -50 and data.xOffset <= 50:
                print("STOP ROTATING !!")
                z_angular_vel = 0.0

            last_offset = data.xOffset
            # Go forward scaled to the relative box area
            x_linear_vel = remap(data.area, area_detected, 0, 0.04, 0.1)
            print("x_linear_vel : ", x_linear_vel)
            # If the area is greater than the threshold area for detection: stop and beep
            if data.area > area_detected:
                x_linear_vel = 0.0
                if time.time() - last_sound_time > 5.0:
                    last_sound_time = time.time()
                    play_sound(6)
                    #say_object_found()
                print("OBJECT FOUND !! ")

                object_found_msg = ObjectFound()
                object_found_msg.detected = True
                object_found_msg.object_name = "object"  # You can change this to a specific object name if needed
                pub_object_found.publish(object_found_msg)  # Publish the object found message

                # Say object found, please input another object, by saying find <class_name>
                # // accept new input - pub once
        # If there is no majority in the queue
        else:
            # If object lost, turn towards last seen orientation
            searching = False
            if last_offset > 0:
                z_angular_vel = -0.2
            else:
                z_angular_vel = 0.2
            x_linear_vel = x_linear_vel - 0.02 if x_linear_vel > 0.0 else 0.0

            # If object not found in some time, turn into a spiral to explore
            print("lastdetect time:\t", last_detection_time)
            time_diff = time.time() - last_detection_time
            print("time_diff:\t", time_diff)
            if time_diff > 30.0:
                if not spiraling_sound:
                    play_sound(5)
                    spiraling_sound = True
                z_angular_vel, x_linear_vel = 0.0,0.0
                searching = True

            else:
                spiraling_sound = False

        print("detected:\t", bool(data.detected))
        print("offset:\t\t", data.xOffset)
        print("last offset:\t", last_offset)
        print("rotvel:\t\t", z_angular_vel)
        print("area:\t\t" , data.area)
        print("linvel:\t\t", x_linear_vel)
        print("detections:\t", detections)
        print("majority:\t", detected_majority(detections))

        print("--------------------------------------------------")
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %f and %f', data.xOffset, data.area)
    else:
        pass

def Average(lst): 
    return reduce(lambda a, b: a + b, lst) / len(lst)

def callback(data):
    global y_l, front_range, front_limit,l_l,r_l,y_b

    x  = list(data.ranges)
    for i in range(360):
        if x[i] == inf:
            x[i] = 7
        if x[i] == 0:
            x[i] = 6


    y_l   = min(min(x[0:front_range//2],x[360-front_range//2:359]))
    l_l   = min(x[10:50])
    r_l   = min(x[310:350])
    y_b   = min(x[165:195])

    print('all distance',format(y_l),' ',format(l_l),' ',format(r_l),' ',format(y_b))  


def wander_controller_move():
    global y_l,front_limit,l_l,r_l,fast,linear_speed,angular_speed,side_limit
    while not rospy.is_shutdown():

        linear_vel = np.clip((y_l-front_limit+0.2)/5,-0.2,0.5)
        #linear_vel = linear_speed*fast
        angular_vel_add1 = 0
        angular_vel_add2 = 0
                
        #turn a bit in opposite direction if object is on right or left side
        if l_l < side_limit:
            angular_vel_add1 = -angular_speed

        if r_l < side_limit:
            angular_vel_add2 = angular_speed
                
        if l_l < side_limit and r_l < side_limit:
            angular_vel_add2 =0
            angular_vel_add1 =0

        angular_vel = (0 + angular_vel_add1 + angular_vel_add2)*fast

        vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_vel))
        velocity_publisher.publish(vel_msg)

        print('moving turtlebot with speed =',format(linear_vel))
        print('distance from front obstacle =',format(y_l))
        rate.sleep()

        if y_l < front_limit:
            break

    velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
    print('rotating turtlebot')
    wander_controller_rotate()

def wander_controller_rotate():
    global y_l,fast, y_b,angular_speed,linear_speed,front_limit
      
    v = random.randint(1,101)
    while not rospy.is_shutdown():
        turn = 0
        if v > 40:
            turn = -1
        if v < 40:
            turn = 1
        turn = 1

        angular_vel = angular_speed*turn*fast
        linear_vel  = np.clip((y_l - front_limit*2/3),-0.2,linear_speed)  
        vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_vel))

        velocity_publisher.publish(vel_msg)

        print('rotating turtlebot with angular speed =',format(angular_vel))
        print('rotating turtlebot with speed =',format(linear_vel))
        print('distance from front obstacle =',format(y_l))
        print('distance from back obstacle =',format(y_b))
        rate.sleep()

        if y_l<front_limit/4 or y_b < 0.1:
            stuck()

        if y_l>front_limit:
            if l_l>0.15:
                if r_l >0.15:
                    break

    velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
    print('moving turtlebot')
    wander_controller_move()

def stuck():
    global y_l,fast,linear_speed,angular_speed

    while not rospy.is_shutdown():

        sign =1
        if y_b < 0.1:
            sign = -1

        vel_msg = Twist(Vector3(-0.1*sign,0,0), Vector3(0,0,0))
        velocity_publisher.publish(vel_msg)
        print('oops')
        rate.sleep()

        if y_l > 0.4:
            if y_b > 0.1:
                break
        
    velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))       
    wander_controller_rotate()


## Publishes updated velocities to the robot controller
def control_robot():
    global rate
    rospy.loginfo("To stop TurtleBot CTRL + C")

    # Initialize node
    rospy.init_node('control_robot', anonymous=True)

    rate = rospy.Rate(10)   # How often should move (10 HZ)
    move_cmd = Twist()      # Data type for velocity

    rospy.Subscriber('control_robot/camera_detection', FloatList, detection_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    scan_subscriber = rospy.Subscriber('/scan', LaserScan, callback) 
    
    linear_velocity = 0.0
    angular_velocity = 0.0
    spiral_start_time = time.time()

    while not rospy.is_shutdown():
                    
        if searching or time.time() - last_detection_time > 15.0:                               
            wander_controller_move()
        else:

            # Send velocity to TurtleBot
            move_cmd.linear.x = x_linear_vel
            move_cmd.angular.z = z_angular_vel
        
        print("x ,z : ", move_cmd.linear.x, move_cmd.angular.z)
        pub.publish(move_cmd)
        # Wait for 0.1 seconds, 10 HZ
        rate.sleep()

if __name__ == '__main__':
    try:
        control_robot()
    except Exception as e:
        rospy.loginfo(f"control_robot node terminated: {e}")



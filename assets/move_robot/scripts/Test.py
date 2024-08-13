#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
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

# Publishers: send velocity and sound to TurtleBot
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Updated velocity command topic
pub_sound = rospy.Publisher('/sound', Sound, queue_size=10)  # Updated sound command topic
pub_object_found = rospy.Publisher('/Object_found', ObjectFound, queue_size=10)  # Use the custom message type
pub_custom_string = rospy.Publisher('/detection', String, queue_size=10)  # New publisher for custom string message

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

sound_counter = 0           # Counter for sound plays
searching = True            # State for initial searching

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/root/OBJ_WS/src/ros_speech_recognization/scripts/google_secret_key.json'

def play_sound(value):
    sound_msg = Sound()
    sound_msg.value = value
    pub_sound.publish(sound_msg)

def say_object_found():
    global sound_counter
    if sound_counter < 2:
        generated_text = "Object found. Ready to listen for a new object."
        tts = gTTS(generated_text)
        tts.save('output.mp3')
        time.sleep(0.1)
        if os.path.exists("./output.mp3"):
            os.system("ffplay -nodisp -autoexit -loglevel quiet output.mp3")
            os.system("rm output.mp3")
            sound_counter += 1
        else:
            print("Error: Could not generate speech.")
    else:
        sound_counter = 0

def remap(old_val, old_min, old_max, new_min, new_max):
    return (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min

def detected_majority(detections):
    pos = 0
    for i in range(0, len(detections)):
        pos = pos + 1 if detections[i] == 1 else pos
    return pos >= len(detections)/2

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

def odom_callback(data):
    global orientation
    orientation_q = data.pose.pose.orientation
    euler = quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    orientation = euler[0]  # Save yaw

def detection_callback(data):
    global x_linear_vel, z_angular_vel, detections, last_sound_time, area_detected, last_offset, last_detection_time, spiraling_sound, maneuvering, orientation, searching, time_diff

    # Add a detection to the queue
    detections.append(data.detected)

    # If it is not solving a bump collision
    if not maneuvering:
        # If there is a majority in the queue
        if detected_majority(detections):

            # Publish the custom string message
            custom_string_msg = String()
            custom_string_msg.data = "OBJDETECTED"
            pub_custom_string.publish(custom_string_msg)

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
                searching = True
        else:
            # If object lost, turn towards last seen orientation
            searching = False
            
            # Publish the custom string message
            custom_string_msg = String()
            custom_string_msg.data = "OBJDETECTED"
            pub_custom_string.publish(custom_string_msg)

            if last_offset > 0:
                z_angular_vel = -0.2
            else:
                z_angular_vel = 0.2
            x_linear_vel = x_linear_vel - 0.02 if x_linear_vel > 0.0 else 0.0

            # If object not found in some time, turn into a spiral to explorea
            time_diff = time.time() - last_detection_time
            if time_diff > 20.0:
                if not spiraling_sound:
                    play_sound(5)
                    spiraling_sound = True  
                searching = True 
            else:
                searching = True
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

## Publishes updated velocities to the robot controller
def control_robot():

    rospy.loginfo("To stop TurtleBot CTRL + C")
    # Initialize node
    rospy.init_node('control_robot', anonymous=True)
    rate = rospy.Rate(10)   # How often should move (10 HZ)
    move_cmd = Twist()      # Data type for velocity

    rospy.Subscriber('control_robot/camera_detection', FloatList, detection_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    while not rospy.is_shutdown():
        if not searching and time.time() - last_detection_time < 2.0:  
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
#!/usr/bin/env python

import rospy
import sys
import math
import tf
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class dance:
   def __init__ (self):
      self.zig_state = 160
      self.linear_speed = 0.2
      self.angular_speed = math.pi/4
      # init node
      rospy.init_node('dancer')

      # subscribers/publishers
      self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)

      # RUN rosrun prrexamples key_publisher.py to get /keys
      self.key_sub = rospy.Subscriber('keys', String, self.key_cb)
      self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
      self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

      # start in state halted and grab the current time
      self.state = "H"
      self.last_key_press_time = rospy.Time.now()

      # set rate
      self.rate = rospy.Rate(10)
      self.t = Twist()
      self.stop_flag = False
      self.obstacle = 0
      self.isFront = False
      self.isBehind = False
      self.go_up = '\x1b[1A' 
      self.erase_line = '\x1b[2K' 
      self.x = 0
      self.y = 0


   # fill in scan callback
   def scan_cb(self,msg):
      self.obstacle = msg.ranges[0]
      if msg.ranges[0]< 0.2 and self.isFront == True:
         self.stop_flag = True
         self.state = "h"
         self.set_state("h")
      else:
         self.stop_flag = False


   # it is not necessary to add more code here but it could be useful
   def key_cb(self,msg):
      # global state; global last_key_press_time
      self.state = msg.data
      self.last_key_press_time = rospy.Time.now()

   # odom is also not necessary but very useful
   def odom_cb(self,odom):
      self.x = odom.pose.pose.position.x
      self.y = odom.pose.pose.position.y
      # self.rot_q = odom.pose.pose.orientation
      # (self.roll, self.pitch, self.theta) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])




   def set_state(self,str):
      if (str == "l" or str == "L"):
         self.isFront = False
         self.t.angular.z = self.angular_speed
         self.t.linear.x = 0
      elif(str == "r" or str == "R"):
         self.isFront = False
         self.t.angular.z = -self.angular_speed
         self.t.linear.x = 0
      elif(str == "f" or str == "F"):
         self.isFront = True
         if(self.stop_flag == False):
            self.t.linear.x = self.linear_speed
            self.t.angular.z = 0
         else:
            self.t.linear.x = 0
            self.t.angular.z = 0
            self.state = "h"
            self.set_state("h")
      elif(str == "b" or str == "B"):
         self.isFront = False
         self.t.linear.x = -self.linear_speed
         self.t.angular.z = 0
      elif(str == "h" or str == "H"):
         self.isFront = False
         self.t.angular.z = 0
         self.t.linear.x = 0
      elif(str == "s" or str == "L"):
         self.isFront = True
         if(self.stop_flag == False):
            self.t.angular.z = self.angular_speed
            self.t.linear.x = self.linear_speed
         else:
            self.t.linear.x = 0
            self.t.angular.z = 0
            self.state = "h"
            self.set_state("h")
      elif(str == "z" or str == "Z"):
         self.isFront = True
         if(self.stop_flag == False):
            if(self.zig_state > 110):
               self.t.angular.z = 0
               self.t.linear.x = self.linear_speed
               self.zig_state -= 1
            elif(self.zig_state > 80):
               self.t.angular.z = self.angular_speed
               self.t.linear.x = 0
               self.zig_state -= 1
            elif(self.zig_state > 30):
               self.t.angular.z = 0
               self.t.linear.x = self.linear_speed
               self.zig_state -= 1
            elif(self.zig_state > 0):
               self.t.angular.z = self.angular_speed * -1
               self.zig_state -= 1
               self.t.linear.x = 0
            else: 
               self.t.angular.z = 0
               self.t.linear.x = 0
               self.zig_state = 160
         else:
            self.t.linear.x = 0
            self.t.angular.z = 0
            self.state = "h"
            self.set_state("h")
      elif(str == "r" or str == "R"):
         self.t.linear.x = random.choice[0.2, 0, -0.2]
         self.t.angular.z = random.choice[self.angular_speed, 0 , -self.angular_speed]

   

   def delete_lines(self, n=1): 
      for _ in range(n):
        sys.stdout.write(self.go_up) 
        sys.stdout.write(self.erase_line) 


   # print the state of the robot
   def print_state(self):
      print("|--------------------------------------------------------------|")
      print("|       STATE: " + self.state + "                  ")

      print("|       Location x: " + str(self.x) + " y: " + str(self.y))
      # calculate time since last key stroke
      time_since = rospy.Time.now() - self.last_key_press_time
      print("|       SECS SINCE LAST KEY PRESS: " + str(time_since.secs) + "      ")
      print("|       linear speed :" +str(self.t.linear.x) + "                  ")
      print("|       angular speed :" +str(self.t.angular.z) + "                  ")
      print("|       obstable ahead: " + str(self.obstacle) + "m         ")
      print("|--------------------------------------------------------------|")
      

   


  
   # Wait for published topics, exit on ^c
   def loop(self):
      while not rospy.is_shutdown():

         # print out the current state and time since last key press
         self.print_state()
         self.delete_lines(8)

         # publish cmd_vel from here 
         self.set_state(self.state)
         # one idea: use vector-2 to represent linear and angular velocity
         # velocity_vector = [linear_component, angular_component]
         # then represent:
         # twist.linear.x = LINEAR_SPEED * linear_component
         # twist.angular.z = ANGULAR_SPEED * angular_component 
         # where for example:
         # LINEAR_SPEED = 0.2, ANGULAR_SPEED = pi/4
         # velocity_vector = [1, 0] for positive linear and no angular movement
         # velocity_vector = [-1, 1] for negative linear and positive angular movement
         # we can then create a dictionary state: movement_vector to hash the current position to get the movement_vector
         # in order to get the zig zag and spiral motion you could you something like this:
         # twist.linear.x = LINEAR_SPEED * linear_component * linear_transform
         # twist.angular.z = ANGULAR_SPEED * angular_component * angular_transform
         # where the [linear_transform, angular_transform] is derived from another source that is based on the clock
         # now you can change the velocity of the robot at every step of the program based on the state and the time
         self.cmd_vel_pub.publish(self.t)

         # run at 10hz
         self.rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        robot = dance()
        robot.loop()
    except rospy.ROSInterruptException: pass

#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Byte
import math
import time

goal_flag = 0 # 0:to continue motion - 1:to stop motion and reset variables

# Global Variables for PID of right motor
wr_actual = 0 # Actual Velocity
wr_target = 0 # Target Velocity
e_wr = 0 #Error in Velocity
e_wr_prev = 0 #Derivative term
e_wr_sum = 0 #Intergal Term
action_r = 0 #Control action on right motor
# PID Gains for right motor
Kp_r = 0.45
Ki_r = 0.15
Kd_r = 0

# Global Variables for PID of left motor
wl_actual = 0
wl_target = 0
e_wl = 0
e_wl_prev = 0
e_wl_sum = 0
action_l = 0
# PID Gains for left motor
Kp_l = 0.45
Ki_l = 0.08
Kd_l = 0

#Defining the node and publishers
rospy.init_node('vel_pid_rob1')
r_pwr = rospy.Publisher('rmotor_pwr_rob1', Float32, queue_size = 10)
l_pwr = rospy.Publisher('lmotor_pwr_rob1', Float32, queue_size = 10)
rate = rospy.Rate(30)

# The function resets all PID variables for a clean start
def resetPID():
        global wr_actual, wr_target, e_wr, e_wr_prev, e_wr_sum, action_r
        global wl_actual, wl_target, e_wl, e_wl_prev, e_wl_sum, action_l
        # Reset terms to start cleanly next time
        e_wr = 0
        e_wr_prev = 0
        e_wr_sum = 0
        action_r = 0
        e_wl = 0
        e_wl_prev = 0
        e_wl_sum = 0
        action_l = 0

        wr_target = 0
        wl_target = 0

        r_pwr.publish(action_r)
        l_pwr.publish(action_l)

def PID():
    global goal_flag
    global wr_actual, wr_target, e_wr, e_wr_prev, e_wr_sum
    global wl_actual, wl_target, e_wl, e_wl_prev, e_wl_sum

    if ( goal_flag == 0 ):
        # Calculate error
        e_wr = wr_target - wr_actual
        e_wl = wl_target - wl_actual

        # Calculate Output
        action_r = (e_wr * Kp_r) + (e_wr_sum * Ki_r) + (e_wr_prev * Kd_r)
        action_l = (e_wl * Kp_l) + (e_wl_sum * Ki_l) + (e_wl_prev * Kd_l)

        # Saturating Output to -100 , 100
        action_r = max(min(100, action_r), -100)
        action_l = max(min(100, action_l), -100)

        # Update Derivative Term
        e_wr_prev = e_wr
        e_wl_prev = e_wl

        # Update Integral Term
        e_wr_sum += e_wr
        e_wl_sum += e_wl

        # Saturating Integral Term to Prevent Integral Windup
        e_wr_sum = max(min(70, e_wr_sum), -70)
        e_wl_sum = max(min(70, e_wl_sum), -70)

        r_pwr.publish(action_r)
        l_pwr.publish(action_l)

        rate.sleep()

    elif( goal_flag == 1 ):
        resetPID()


def wr_act_callback(data):
    global wr_actual
    wr_actual = data.data

def wl_act_callback(data):
    global wl_actual
    wl_actual = data.data

def wr_targ_callback(data):
    global wr_target
    wr_target = data.data

def wl_targ_callback(data):
    global wl_target
    wl_target = data.data

def goal_callback(data):
    global goal_flag
    goal_flag = data.data

def pid_listener():
    rospy.loginfo("%s started" % rospy.get_name())
    while not rospy.is_shutdown():
        rospy.Subscriber('rwheel_spd_rob1', Float32, wr_act_callback)
        rospy.Subscriber('lwheel_spd_rob1', Float32, wl_act_callback)
        rospy.Subscriber('rwheel_vtarget_rob1', Float32, wr_targ_callback)
        rospy.Subscriber('lwheel_vtarget_rob1', Float32, wl_targ_callback)
        rospy.Subscriber('gflag_rob1', Byte, goal_callback)
        PID()

if __name__ == '__main__':
    try:
        pid_listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        resetPID()
        time.sleep(1)
        rospy.loginfo("%s closed. All PID Vars are now reset" % rospy.get_name())

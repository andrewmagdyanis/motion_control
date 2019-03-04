#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import math
import time

import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Encoder pins and counters
#Right Motor
encA1 = 23
encB1 = 22
encoder1_ticks = 0
encoder1_ticks_calc = 0
encoder1_ticks_prev = 0

#Left Motor
encA2 = 27
encB2 = 24
encoder2_ticks = 0
encoder2_ticks_calc = 0
encoder2_ticks_prev = 0

# Distance Variables
rwheel_dist = 0
lwheel_dist = 0

# Speed Variables
rwheel_spd = 0
lwheel_spd = 0

# Delta Time for Calculations
dt = 0.05
###########################################
# Kinematic Info About Robot
# Wheel Diameter (D) = 65 mm = 0.065 m
# Wheel Radius (R) = 32.5mm = 0.0325 m
# Wheel Base (L) = 167 mm = 0.167 m
# GR = 21.3 : 1
# PPR = 11 (Motor Side)
############################################

# dist_const/2 is used to facilitate dist/speed calculations respectively = (distance per rev) / (PPR * GR)
dist_const = ( math.pi * 6.5 ) / ( 11 * 21.3 ) # Used to get distance covered in CM
rad_const = ( 2 * math.pi ) / (11 * 21.3 * dt) # Used to get speed in rad/s
#rev_const = (60) / ( 11 * 21.3 * dt)

def encoders_talker():
    global encoder1_ticks
    global encoder2_ticks

    global rwheel_dist
    global lwheel_dist

    global rwheel_spd
    global lwheel_spd

    global encoder1_ticks_prev
    global encoder2_ticks_prev

    # Initialize the node
    rospy.init_node('encoders_rob1')
    rospy.loginfo("%s started" % rospy.get_name())

    # Initialize encoder publishers
    enc1= rospy.Publisher('enc1_ticks_rob1',Int32, queue_size=10)
    enc2= rospy.Publisher('enc2_ticks_rob1',Int32, queue_size=10)

    #Speed/Position Publishers
    rwheelSpeed = rospy.Publisher('rwheel_spd_rob1', Float32, queue_size=10)
    lwheelSpeed = rospy.Publisher('lwheel_spd_rob1', Float32, queue_size=10)

    rwheelDist = rospy.Publisher('rwheel_dist_rob1', Float32, queue_size=10)
    lwheelDist = rospy.Publisher('lwheel_dist_rob1', Float32, queue_size=10)

    while not rospy.is_shutdown():
        # Calculating Velocities and Distances to Publish
        encoder1_ticks_calc = encoder1_ticks
        encoder2_ticks_calc = encoder2_ticks

        rwheel_dist =  encoder1_ticks * dist_const
        lwheel_dist =  encoder2_ticks * dist_const

        #Rad Speeds
        rwheel_spd = ( encoder1_ticks - encoder1_ticks_prev ) * rad_const
        lwheel_spd = ( encoder2_ticks - encoder2_ticks_prev ) * rad_const

        encoder1_ticks_prev = encoder1_ticks_calc
        encoder2_ticks_prev = encoder2_ticks_calc

        # Publishing Calculated Variables in dt
        # Encoder ticks since the beginning
        enc1.publish(encoder1_ticks)
        enc2.publish(encoder2_ticks)

        # Wheel Velocities
        rwheelSpeed.publish(rwheel_spd)
        lwheelSpeed.publish(lwheel_spd)

        # Distances covered by wheels since the beginning
        rwheelDist.publish(rwheel_dist)
        lwheelDist.publish(rwheel_dist)

        # Wait dt
        time.sleep(dt)

# Updating encoder counters for each interrupt
def do_encoder1(channel1):  #encoder1
	global encoder1_ticks
	if GPIO.input(encB1) == 1:
		encoder1_ticks -= 1
	else:
		encoder1_ticks += 1

def do_encoder2(channel2):  #encoder2
	global encoder2_ticks
	if GPIO.input(encB2) == 1:
		encoder2_ticks += 1
	else:
		encoder2_ticks -= 1

# Encoder 1 GPIO
GPIO.setup (encA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.setup (encB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.add_event_detect (encA1, GPIO.FALLING, callback=do_encoder1)   # Encoder 1 interrupt

# Encoder 2 GPIO
GPIO.setup (encA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.setup (encB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.add_event_detect (encA2, GPIO.FALLING, callback=do_encoder2)   # Encoder 2 interrupt

if __name__ == '__main__':
    try:
        encoders_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
        rospy.loginfo("%s closed. GPIO Cleaned" % rospy.get_name())
